/**
 * @file spi_connector.c
 * @brief Crazyflie SPI Slave Connector
 *
 * This module handles SPI slave communication with the Crazyflie STM32.
 * It uses a dedicated thread to handle SPI transactions, completely decoupled
 * from the ranging/superframe execution.
 *
 * Architecture:
 * - Digest processors call send_measurement_to_crazyflie() which queues measurements
 * - A dedicated connector thread waits for the Crazyflie master to initiate transfers
 * - The thread uses synchronous SPI API (can block indefinitely waiting for master)
 * - READY GPIO signals to master that data is available
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <app/lib/crazyflie/connector.h>
#include <app/drivers/ieee802154/dw1000.h>
#include <app/lib/node_table/node_table.h>

LOG_MODULE_REGISTER(spi_connector, CONFIG_SPI_CONNECTOR_LOG_LEVEL);

/* ========================================================================== */
/* Device Tree and Hardware Configuration                                     */
/* ========================================================================== */

#define HCI_SPI_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(synchrofly_hci_slave)

static const struct device *const spi_dev = DEVICE_DT_GET(DT_BUS(HCI_SPI_NODE));
static struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_OP_MODE_SLAVE,
};
static const struct gpio_dt_spec ready_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, irq_gpios);

/* ========================================================================== */
/* Protocol Constants - Keep in sync with synchrofly_hci_protocol.h           */
/* ========================================================================== */

/* Protocol v5: bidirectional CRTP over single transaction
 * Master sends: [CMD_READ] [crtp_tx_count] [crtp_packets...]
 * Slave sends:  [meas_count | measurements... | crtp_count | crtp_packets...]
 */
#define SYNCHROFLY_HCI_STATUS_MASTER_HEADER_CMD_READ   0x01
#define SYNCHROFLY_HCI_MAX_CRTP_PACKETS                16  /* Increased from 8 for TOC fetch throughput */
#define SYNCHROFLY_HCI_CRTP_MAX_DATA_SIZE              30

/* Idle polling timeout - triggers SPI exchange to receive CRTP from Crazyflie
 * Reduced from 50ms to 10ms for faster CRTP response during active operations */
#define IDLE_POLL_TIMEOUT_MS  10

/* SPI slave timeout - maximum time to wait for master to initiate transaction
 * If master doesn't respond within this time, we abort and retry.
 * This prevents the connector thread from blocking indefinitely. */
#define SPI_SLAVE_TIMEOUT_MS  50

/* ========================================================================== */
/* Measurement Message Structure                                              */
/* ========================================================================== */

/* Measurement types - use uint8_t in struct for ABI stability across compilers */
enum cf_measurement_type {
    CF_MEASUREMENT_TYPE_POS,
    CF_MEASUREMENT_TYPE_DIST,
    CF_MEASUREMENT_TYPE_TDOA,
    CF_MEASUREMENT_TYPE_PHASE,
    CF_MEASUREMENT_TYPE_CFO,
    CF_MEASUREMENT_TYPE_DIST_MM,
    CF_MEASUREMENT_TYPE_DIST_DUAL,
    CF_MEASUREMENT_TYPE_RADIAL_VEL,
};

struct synchrofly_measurement_msg {
    deca_short_addr_t node_addr;
    uint8_t type;  /* cf_measurement_type - use uint8_t for fixed size across compilers */
    uint8_t _reserved;  /* Padding for alignment */
    float x, y, z;         /* Anchor/node position in meters */
    union {
        float m;           /* Distance in meters (for DIST type) */
        struct {
            float distance_mm;
            float distance_twr;
            float confidence;
            float phase_chan_5;
            float phase_chan_3;
            uint8_t channels;
        } mm_data __attribute__((packed));
        struct {
            float distance;
            float confidence;
            float phase_diff;
            uint8_t method;
        } dual_data __attribute__((packed));
        struct {
            float radial_velocity;  /* m/s, positive = moving away from point */
        } vel_data __attribute__((packed));
    } __attribute__((packed));
    float stdDev;
} __attribute__((packed));

/* ========================================================================== */
/* CRTP Message Structure                                                     */
/* ========================================================================== */

struct synchrofly_crtp_msg {
    uint8_t header;                                /* CRTP header: port(4) | channel(2) | reserved(2) */
    uint8_t size;                                  /* Payload size (0-30) */
    uint8_t data[SYNCHROFLY_HCI_CRTP_MAX_DATA_SIZE]; /* CRTP payload */
} __attribute__((packed));

/* ========================================================================== */
/* Measurement Container                                                      */
/* ========================================================================== */

#define MAX_MEASUREMENTS_PER_PACKET 10

struct crazyflie_measurements {
    uint8_t capacity;
    uint8_t count;
    struct synchrofly_measurement_msg data[MAX_MEASUREMENTS_PER_PACKET];
};

/* ========================================================================== */
/* Thread and Queue Configuration                                             */
/* ========================================================================== */

#define CONNECTOR_THREAD_STACK_SIZE 2048
#define CONNECTOR_THREAD_PRIORITY   5
#define MEASUREMENT_QUEUE_DEPTH     4

/* Message queue for measurements - producers (digest processors) never block */
K_MSGQ_DEFINE(measurement_queue, sizeof(struct crazyflie_measurements),
              MEASUREMENT_QUEUE_DEPTH, 4);

/* Semaphore to signal connector thread when CRTP packets are queued */
K_SEM_DEFINE(exchange_sem, 0, 1);

/* CRTP packet queue - ring buffer with spinlock protection */
static struct synchrofly_crtp_msg crtp_queue[SYNCHROFLY_HCI_MAX_CRTP_PACKETS];
static volatile uint8_t crtp_queue_head = 0;
static volatile uint8_t crtp_queue_tail = 0;
static struct k_spinlock crtp_lock;

/* Thread stack and data */
K_THREAD_STACK_DEFINE(connector_thread_stack, CONNECTOR_THREAD_STACK_SIZE);
static struct k_thread connector_thread_data;
static k_tid_t connector_thread_id;

/* ========================================================================== */
/* SPI Buffers                                                                */
/* ========================================================================== */

/* Buffer must fit: 1 byte header + N measurements + 1 byte CRTP count + CRTP packets
 * Each measurement is ~44 bytes, so 512 bytes supports ~10 measurements */
#define SPI_BUFFER_SIZE 512

static uint8_t spi_tx_buf[SPI_BUFFER_SIZE];
static uint8_t spi_rx_buf[SPI_BUFFER_SIZE];

static struct spi_buf tx_spi_buf = { .buf = spi_tx_buf, .len = 0 };
static struct spi_buf rx_spi_buf = { .buf = spi_rx_buf, .len = 0 };
static const struct spi_buf_set tx_buf_set = { .buffers = &tx_spi_buf, .count = 1 };
static const struct spi_buf_set rx_buf_set = { .buffers = &rx_spi_buf, .count = 1 };

/* ========================================================================== */
/* CRTP RX Callback                                                           */
/* ========================================================================== */

/* Callback for CRTP packets received from Crazyflie (Crazyflie -> UWB direction) */
static crazyflie_crtp_rx_callback_t crtp_rx_callback = NULL;

/* ========================================================================== */
/* Statistics                                                                 */
/* ========================================================================== */

static struct {
    uint32_t packets_sent;
    uint32_t packets_dropped;
    uint32_t protocol_errors;
    uint32_t crtp_sent;
    uint32_t crtp_dropped;
    uint32_t crtp_received;
    uint32_t idle_polls;
    uint32_t spi_timeouts;
} stats;

/* Signal for async SPI with timeout */
static struct k_poll_signal spi_done_sig;
static struct k_poll_event spi_done_event;

/* ========================================================================== */
/* Protocol Handler                                                           */
/* ========================================================================== */

/**
 * @brief Handle a complete SPI exchange with the Crazyflie
 *
 * Protocol v5 bidirectional CRTP:
 * 1. Slave asserts READY GPIO to signal it wants to communicate
 * 2. Master sends: [CMD_READ] [crtp_tx_count] [crtp_packets...]
 *    Slave sends:  [meas_count | meas... | crtp_count | crtp_packets...]
 * 3. Slave parses RX buffer for CRTP packets from master
 * 4. Slave deasserts READY GPIO
 *
 * This function blocks until the exchange completes or an error occurs.
 *
 * @param measurements Measurement container (may have count=0)
 * @param have_measurements True if measurements contains valid data
 */
static int handle_spi_exchange(const struct crazyflie_measurements *measurements,
                               bool have_measurements)
{
    int ret;
    uint8_t tx_data[SPI_BUFFER_SIZE];
    uint8_t rx_data[SPI_BUFFER_SIZE];
    size_t offset;
    uint8_t meas_count = have_measurements ? measurements->count : 0;
    uint8_t crtp_count = 0;

    /* Build TX buffer: [meas_count | measurements...] */
    offset = 1 + meas_count * sizeof(struct synchrofly_measurement_msg);

    if (offset > SPI_BUFFER_SIZE - 1) {
        LOG_ERR("Measurement data too large: %zu", offset);
        return -ENOMEM;
    }

    tx_data[0] = meas_count;
    if (have_measurements && meas_count > 0) {
        memcpy(tx_data + 1, measurements->data,
               meas_count * sizeof(struct synchrofly_measurement_msg));
    }

    /* Append CRTP packets: [crtp_count | crtp_packets...] */
    k_spinlock_key_t key = k_spin_lock(&crtp_lock);
    LOG_DBG("CRTP queue check: head=%d tail=%d", crtp_queue_head, crtp_queue_tail);
    while (crtp_queue_tail != crtp_queue_head &&
           crtp_count < SYNCHROFLY_HCI_MAX_CRTP_PACKETS &&
           offset + 1 + (crtp_count + 1) * sizeof(struct synchrofly_crtp_msg) <= SPI_BUFFER_SIZE) {
        memcpy(&tx_data[offset + 1 + crtp_count * sizeof(struct synchrofly_crtp_msg)],
               &crtp_queue[crtp_queue_tail], sizeof(struct synchrofly_crtp_msg));
        crtp_queue_tail = (crtp_queue_tail + 1) % SYNCHROFLY_HCI_MAX_CRTP_PACKETS;
        crtp_count++;
    }
    k_spin_unlock(&crtp_lock, key);

    tx_data[offset] = crtp_count;
    size_t tx_len = offset + 1 + crtp_count * sizeof(struct synchrofly_crtp_msg);

    if (crtp_count > 0) {
        LOG_DBG("SPI TX: %d meas + %d CRTP packets (%zu bytes)", meas_count, crtp_count, tx_len);
    } else {
        LOG_DBG("SPI TX: %d measurements (%zu bytes)", meas_count, tx_len);
    }

    /* CRITICAL: The master (Crazyflie) always sends a fixed-size transaction.
     * We must use the same size or the SPI slave will have buffer issues.
     * Master uses: min(512, 1 + MAX_MEASUREMENTS * sizeof(measurement))
     * which is 512 bytes (capped by SYNCHROFLY_HCI_MAX_SPI_TRANSACTION_SIZE).
     */
    size_t exchange_len = SPI_BUFFER_SIZE;  /* Must match master's transaction size */

    /* Set up SPI buffers */
    memset(spi_tx_buf, 0, exchange_len);
    memcpy(spi_tx_buf, tx_data, tx_len);
    /* Fill RX buffer with marker to detect if DMA actually writes to it */
    memset(spi_rx_buf, 0xAA, exchange_len);
    tx_spi_buf.len = exchange_len;
    rx_spi_buf.len = exchange_len;

    /* CRITICAL: Order of operations matters for SPI slave!
     * 1. First arm the SPI peripheral (spi_transceive_signal)
     * 2. THEN assert READY GPIO to tell master we're ready
     * If we assert READY before arming, master may start clocking before
     * the slave peripheral is ready, causing data corruption.
     */
    memset(rx_data, 0, sizeof(rx_data));

    /* Reset signal and re-init event for timeout support */
    k_poll_signal_reset(&spi_done_sig);
    k_poll_event_init(&spi_done_event, K_POLL_TYPE_SIGNAL,
                      K_POLL_MODE_NOTIFY_ONLY, &spi_done_sig);

    /* Arm SPI peripheral FIRST */
    ret = spi_transceive_signal(spi_dev, &spi_cfg, &tx_buf_set, &rx_buf_set, &spi_done_sig);
    if (ret < 0) {
        LOG_ERR("SPI transceive_signal failed: %d", ret);
        return ret;
    }

    /* NOW assert READY - master can safely start clocking */
    gpio_pin_set_dt(&ready_gpio, 1);

    /* Wait for completion with timeout */
    uint32_t start_time = k_uptime_get_32();
    ret = k_poll(&spi_done_event, 1, K_MSEC(SPI_SLAVE_TIMEOUT_MS));
    uint32_t elapsed = k_uptime_get_32() - start_time;

    /* Deassert READY immediately after transaction (or timeout) */
    gpio_pin_set_dt(&ready_gpio, 0);

    if (ret == -EAGAIN) {
        /* Timeout - master didn't respond */
        LOG_WRN("SPI slave timeout waiting for master (%ums)", elapsed);
        stats.spi_timeouts++;
        /* Release the SPI peripheral */
        spi_release(spi_dev, &spi_cfg);
        return -ETIMEDOUT;
    } else if (ret < 0) {
        LOG_ERR("k_poll failed: %d", ret);
        return ret;
    }

    /* Check the result from the signal */
    unsigned int signaled;
    int result;
    k_poll_signal_check(&spi_done_sig, &signaled, &result);
    if (!signaled) {
        LOG_ERR("Signal not raised after k_poll success");
        return -EIO;
    }
    if (result < 0) {
        LOG_ERR("SPI transceive failed: %d", result);
        return result;
    }

    /* Log timing and received data for debugging bidirectional comms */
    LOG_DBG("SPI done: ret=%d, %ums, spi_rx_buf[0..3]=0x%02x 0x%02x 0x%02x 0x%02x",
            result, elapsed, spi_rx_buf[0], spi_rx_buf[1], spi_rx_buf[2], spi_rx_buf[3]);

    /* Copy received data */
    memcpy(rx_data, spi_rx_buf, exchange_len);

    /* Parse RX buffer: [CMD_READ] [crtp_tx_count] [crtp_packets...] */
    uint8_t rx_cmd = rx_data[0];
    if (rx_cmd != SYNCHROFLY_HCI_STATUS_MASTER_HEADER_CMD_READ) {
        LOG_WRN("Expected READ (0x%02x), got 0x%02x",
                SYNCHROFLY_HCI_STATUS_MASTER_HEADER_CMD_READ, rx_cmd);
        stats.protocol_errors++;
        return -EPROTO;
    }

    /* Parse CRTP packets from Crazyflie (v5 bidirectional) */
    uint8_t crtp_rx_count = rx_data[1];
    if (crtp_rx_count > 0 && crtp_rx_count <= SYNCHROFLY_HCI_MAX_CRTP_PACKETS) {
        LOG_INF("SPI RX: %d CRTP packets from Crazyflie", crtp_rx_count);

        for (int i = 0; i < crtp_rx_count; i++) {
            size_t pkt_offset = 2 + i * sizeof(struct synchrofly_crtp_msg);
            if (pkt_offset + sizeof(struct synchrofly_crtp_msg) > exchange_len) {
                LOG_WRN("CRTP RX packet %d truncated", i);
                break;
            }

            struct synchrofly_crtp_msg *pkt = (struct synchrofly_crtp_msg *)&rx_data[pkt_offset];
            uint8_t port = (pkt->header >> 4) & 0x0F;
            uint8_t channel = pkt->header & 0x03;
            LOG_INF("  CRTP RX[%d]: port=%d ch=%d size=%d", i, port, channel, pkt->size);

            /* Deliver to callback if registered */
            if (crtp_rx_callback != NULL && pkt->size <= SYNCHROFLY_HCI_CRTP_MAX_DATA_SIZE) {
                /* Reconstruct full CRTP packet: [header] [data...] */
                uint8_t crtp_pkt[SYNCHROFLY_HCI_CRTP_MAX_DATA_SIZE + 1];
                crtp_pkt[0] = pkt->header;
                if (pkt->size > 0) {
                    memcpy(&crtp_pkt[1], pkt->data, pkt->size);
                }
                crtp_rx_callback(crtp_pkt, pkt->size + 1);
                stats.crtp_received++;
            }
        }
    }

    stats.packets_sent++;
    stats.crtp_sent += crtp_count;
    LOG_DBG("Sent %d measurements, %d CRTP packets; received %d CRTP packets",
            meas_count, crtp_count, crtp_rx_count);

    return 0;
}

/* ========================================================================== */
/* Connector Thread                                                           */
/* ========================================================================== */

/**
 * @brief Check if there are pending CRTP packets
 */
static bool have_pending_crtp(void)
{
    k_spinlock_key_t key = k_spin_lock(&crtp_lock);
    bool have_crtp = (crtp_queue_tail != crtp_queue_head);
    k_spin_unlock(&crtp_lock, key);
    return have_crtp;
}

/**
 * @brief Main connector thread function
 *
 * This thread:
 * 1. Waits for measurements, CRTP packets, or idle timeout
 * 2. Handles the SPI exchange with the Crazyflie (can block)
 * 3. Repeats forever
 *
 * The thread is triggered by:
 * - Measurements being queued (via measurement_queue)
 * - CRTP packets being queued (via exchange_sem)
 * - Idle timeout (IDLE_POLL_TIMEOUT_MS) - enables receiving CRTP from Crazyflie
 *   even when the UWB deck has nothing to send
 */
static void connector_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct crazyflie_measurements measurements;
    int ret;

    LOG_INF("Connector thread started (protocol v5 bidirectional CRTP)");

    while (1) {
        bool have_measurements = false;
        bool have_crtp = false;
        bool idle_poll = false;

        /* Try to get measurements (non-blocking) */
        ret = k_msgq_get(&measurement_queue, &measurements, K_NO_WAIT);
        if (ret == 0) {
            have_measurements = true;
            LOG_DBG("Dequeued %d measurements", measurements.count);
        }

        /* Check for CRTP packets to send */
        have_crtp = have_pending_crtp();

        if (!have_measurements && !have_crtp) {
            /* Nothing to send - wait for measurements or CRTP signal
             *
             * When IDLE_POLL_TIMEOUT_MS is 0, we wait forever (only wake on
             * measurements or CRTP signal). When non-zero, we periodically
             * poll the Crazyflie to receive CRTP responses.
             */
#if IDLE_POLL_TIMEOUT_MS > 0
            ret = k_msgq_get(&measurement_queue, &measurements, K_MSEC(IDLE_POLL_TIMEOUT_MS));
            if (ret == 0) {
                have_measurements = true;
                LOG_DBG("Dequeued %d measurements (after wait)", measurements.count);
            } else {
                /* Check if semaphore was signaled during wait */
                ret = k_sem_take(&exchange_sem, K_NO_WAIT);
                if (ret == 0) {
                    have_crtp = have_pending_crtp();
                }

                /* If still nothing to send, do an idle poll anyway
                 * This allows us to receive CRTP packets from the Crazyflie */
                if (!have_measurements && !have_crtp) {
                    idle_poll = true;
                    stats.idle_polls++;
                    LOG_DBG("Idle poll triggered");
                }
            }
#else
            /* Idle polling disabled - wait forever for measurements or use semaphore */
            ret = k_sem_take(&exchange_sem, K_FOREVER);
            if (ret == 0) {
                /* Semaphore signaled - check for CRTP packets */
                have_crtp = have_pending_crtp();
                /* Also check for measurements that may have arrived */
                int mret = k_msgq_get(&measurement_queue, &measurements, K_NO_WAIT);
                if (mret == 0) {
                    have_measurements = true;
                }
            }
#endif
        }
        /* Perform the SPI exchange (even if idle - to receive CRTP from master) */
        if (have_measurements || have_crtp) {
            LOG_DBG("SPI exchange: meas=%d crtp=%d", have_measurements, have_crtp);
        }
        ret = handle_spi_exchange(&measurements, have_measurements);
        if (ret < 0 && !idle_poll) {
            /* Only warn on errors when we actually had data to send */
            LOG_WRN("Exchange failed: %d", ret);
        }

        /* Reset the semaphore in case multiple signals accumulated */
        k_sem_reset(&exchange_sem);
    }
}

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

int crazyflie_send_crtp(const uint8_t *data, size_t len)
{
    if (data == NULL || len < 1 || len > SYNCHROFLY_HCI_CRTP_MAX_DATA_SIZE + 1) {
        LOG_WRN("Invalid CRTP packet: len=%zu (must be 1-%d)",
                len, SYNCHROFLY_HCI_CRTP_MAX_DATA_SIZE + 1);
        return -EINVAL;
    }

    uint8_t header = data[0];
    uint8_t port = (header >> 4) & 0x0F;
    uint8_t channel = header & 0x03;

    LOG_INF("CRTP TX queue: port=%d ch=%d len=%zu", port, channel, len);

    k_spinlock_key_t key = k_spin_lock(&crtp_lock);

    /* Check if queue is full */
    uint8_t next = (crtp_queue_head + 1) % SYNCHROFLY_HCI_MAX_CRTP_PACKETS;
    if (next == crtp_queue_tail) {
        k_spin_unlock(&crtp_lock, key);
        stats.crtp_dropped++;
        LOG_WRN("CRTP queue full, packet dropped (port=%d ch=%d)", port, channel);
        return -EAGAIN;
    }

    /* Add packet to queue */
    struct synchrofly_crtp_msg *msg = &crtp_queue[crtp_queue_head];
    msg->header = data[0];
    msg->size = (len > 1) ? len - 1 : 0;
    if (msg->size > 0) {
        memcpy(msg->data, &data[1], msg->size);
    }
    crtp_queue_head = next;

    k_spin_unlock(&crtp_lock, key);

    LOG_DBG("CRTP queued: port=%d ch=%d size=%d", port, channel, msg->size);

    /* Signal the connector thread that there's data to send */
    k_sem_give(&exchange_sem);

    return 0;
}

void crazyflie_register_crtp_rx_callback(crazyflie_crtp_rx_callback_t callback)
{
    crtp_rx_callback = callback;
    LOG_INF("CRTP RX callback %s", callback ? "registered" : "unregistered");
}

int crazyflie_measurements_alloc(int capacity, crazyflie_measurements_t **measurements)
{
    /* We now use static allocation with fixed max size */
    static struct crazyflie_measurements static_measurements;

    if (capacity > MAX_MEASUREMENTS_PER_PACKET) {
        LOG_WRN("Requested capacity %d exceeds max %d, clamping",
                capacity, MAX_MEASUREMENTS_PER_PACKET);
    }

    static_measurements.capacity = MIN(capacity, MAX_MEASUREMENTS_PER_PACKET);
    static_measurements.count = 0;

    *measurements = &static_measurements;
    return static_measurements.capacity;
}

int crazyflie_measurements_free(crazyflie_measurements_t *measurements)
{
    /* Static allocation - nothing to free */
    if (measurements) {
        measurements->count = 0;
    }
    return 0;
}

size_t crazyflie_measurements_length(const crazyflie_measurements_t *measurements)
{
    return measurements ? measurements->count : 0;
}

int send_measurement_to_crazyflie(crazyflie_measurements_t *measurements, k_timeout_t timeout)
{
    ARG_UNUSED(timeout); /* We always use non-blocking put now */

    if (!measurements || measurements->count == 0) {
        LOG_DBG("No measurements to send");
        return -EINVAL;
    }

    /* Non-blocking put to queue - if full, drop oldest or this one */
    int ret = k_msgq_put(&measurement_queue, measurements, K_NO_WAIT);
    if (ret == -ENOMSG || ret == -EAGAIN) {
        /* Queue full - drop this measurement set */
        stats.packets_dropped++;
        LOG_DBG("Queue full, dropped measurements");
        return -EAGAIN;
    } else if (ret != 0) {
        LOG_ERR("Failed to queue measurements: %d", ret);
        return ret;
    }

    LOG_DBG("Queued %d measurements", measurements->count);

    /* Signal the connector thread that there's data to send */
    k_sem_give(&exchange_sem);

    /* Clear the measurements for reuse */
    measurements->count = 0;

    return 0;
}

int crazyflie_measurements_add_tof(deca_short_addr_t addr, float tof,
                                   crazyflie_measurements_t *measurements)
{
    if (!measurements || measurements->count >= measurements->capacity) {
        return -ENOMEM;
    }

    struct synchrofly_measurement_msg *m = &measurements->data[measurements->count];
    m->node_addr = addr;
    m->type = CF_MEASUREMENT_TYPE_DIST;
    m->m = time_to_dist(tof);
    m->stdDev = 0.25f;

    /* Look up anchor position from node table */
    struct node_entry *entry = node_table_get(addr);
    if (entry != NULL) {
        m->x = entry->pos_x;
        m->y = entry->pos_y;
        m->z = entry->pos_z;
    } else {
        m->x = 0.0f;
        m->y = 0.0f;
        m->z = 0.0f;
    }

    measurements->count++;
    return 0;
}

int crazyflie_measurements_add_distance(deca_short_addr_t addr, float distance_m,
                                        crazyflie_measurements_t *measurements)
{
    if (!measurements || measurements->count >= measurements->capacity) {
        return -ENOMEM;
    }

    struct synchrofly_measurement_msg *m = &measurements->data[measurements->count];
    m->node_addr = addr;
    m->type = CF_MEASUREMENT_TYPE_DIST;
    m->m = distance_m;
    m->stdDev = 0.25f;

    /* Look up anchor position from node table */
    struct node_entry *entry = node_table_get(addr);
    if (entry != NULL) {
        m->x = entry->pos_x;
        m->y = entry->pos_y;
        m->z = entry->pos_z;
    } else {
        m->x = 0.0f;
        m->y = 0.0f;
        m->z = 0.0f;
    }

    measurements->count++;
    return 0;
}

int crazyflie_measurements_add_phase(deca_short_addr_t addr, float phase,
                                     crazyflie_measurements_t *measurements)
{
    if (!measurements || measurements->count >= measurements->capacity) {
        return -ENOMEM;
    }

    struct synchrofly_measurement_msg *m = &measurements->data[measurements->count];
    m->node_addr = addr;
    m->type = CF_MEASUREMENT_TYPE_PHASE;
    m->m = phase;
    m->stdDev = 0.0f;

    measurements->count++;
    return 0;
}

int crazyflie_measurements_add_cfo(deca_short_addr_t addr, float cfo,
                                   crazyflie_measurements_t *measurements)
{
    if (!measurements || measurements->count >= measurements->capacity) {
        return -ENOMEM;
    }

    struct synchrofly_measurement_msg *m = &measurements->data[measurements->count];
    m->node_addr = addr;
    m->type = CF_MEASUREMENT_TYPE_CFO;
    m->m = cfo;
    m->stdDev = 0.0f;

    measurements->count++;
    return 0;
}

int crazyflie_measurements_add_position(deca_short_addr_t addr, float position[3],
                                        crazyflie_measurements_t *measurements)
{
    if (!measurements || measurements->count >= measurements->capacity) {
        return -ENOMEM;
    }

    struct synchrofly_measurement_msg *m = &measurements->data[measurements->count];
    m->node_addr = addr;
    m->type = CF_MEASUREMENT_TYPE_POS;
    m->x = position[0];
    m->y = position[1];
    m->z = position[2];
    m->m = 0.0f;  /* Not used for position type */
    m->stdDev = 0.25f;

    measurements->count++;
    return 0;
}

int crazyflie_measurements_add_mm_accurate(deca_short_addr_t addr,
                                           float distance_mm, float distance_twr,
                                           float phase_chan_5,
                                           float phase_chan_3, uint8_t channels,
                                           crazyflie_measurements_t *measurements)
{
    if (!measurements || measurements->count >= measurements->capacity) {
        return -ENOMEM;
    }

    struct synchrofly_measurement_msg *m = &measurements->data[measurements->count];
    m->node_addr = addr;
    m->type = CF_MEASUREMENT_TYPE_DIST_MM;

    /* Look up anchor position from node table */
    struct node_entry *entry = node_table_get(addr);
    if (entry != NULL) {
        m->x = entry->pos_x;
        m->y = entry->pos_y;
        m->z = entry->pos_z;
    } else {
        m->x = 0.0f;
        m->y = 0.0f;
        m->z = 0.0f;
    }

    m->mm_data.distance_mm = distance_mm;
    m->mm_data.distance_twr = distance_twr;
    m->mm_data.confidence = 0.0f;  /* Unused, kept for SPI protocol compat */
    m->mm_data.phase_chan_5 = phase_chan_5;
    m->mm_data.phase_chan_3 = phase_chan_3;
    m->mm_data.channels = channels;
    m->stdDev = 0.001f;

    measurements->count++;
    return 0;
}

int crazyflie_measurements_add_dual_channel(deca_short_addr_t addr,
                                            float distance,
                                            float phase_diff, uint8_t method,
                                            crazyflie_measurements_t *measurements)
{
    if (!measurements || measurements->count >= measurements->capacity) {
        return -ENOMEM;
    }

    struct synchrofly_measurement_msg *m = &measurements->data[measurements->count];
    m->node_addr = addr;
    m->type = CF_MEASUREMENT_TYPE_DIST_DUAL;

    /* Look up anchor position from node table */
    struct node_entry *entry = node_table_get(addr);
    if (entry != NULL) {
        m->x = entry->pos_x;
        m->y = entry->pos_y;
        m->z = entry->pos_z;
    } else {
        m->x = 0.0f;
        m->y = 0.0f;
        m->z = 0.0f;
    }

    m->dual_data.distance = distance;
    m->dual_data.confidence = 0.0f;  /* Unused, kept for SPI protocol compat */
    m->dual_data.phase_diff = phase_diff;
    m->dual_data.method = method;
    m->stdDev = 0.001f;

    measurements->count++;
    return 0;
}

int crazyflie_measurements_add_radial_velocity(deca_short_addr_t addr,
                                               float radial_velocity_ms,
                                               crazyflie_measurements_t *measurements)
{
    if (!measurements || measurements->count >= measurements->capacity) {
        return -ENOMEM;
    }

    struct synchrofly_measurement_msg *m = &measurements->data[measurements->count];
    m->node_addr = addr;
    m->type = CF_MEASUREMENT_TYPE_RADIAL_VEL;
    m->vel_data.radial_velocity = radial_velocity_ms;
    m->stdDev = 0.25f;

    /* Look up anchor position from node table */
    struct node_entry *entry = node_table_get(addr);
    if (entry != NULL) {
        m->x = entry->pos_x;
        m->y = entry->pos_y;
        m->z = entry->pos_z;
    } else {
        m->x = 0.0f;
        m->y = 0.0f;
        m->z = 0.0f;
    }

    measurements->count++;
    return 0;
}

/* ========================================================================== */
/* Initialization                                                             */
/* ========================================================================== */

static int connector_init(void)
{
    LOG_DBG("Initializing Crazyflie connector");

    /* Verify SPI device */
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device %s not ready", spi_dev->name);
        return -ENODEV;
    }

    /* Configure READY GPIO */
    if (!gpio_is_ready_dt(&ready_gpio)) {
        LOG_ERR("READY GPIO not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&ready_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure READY GPIO: %d", ret);
        return ret;
    }

    /* Initialize SPI poll signal and event for timeout support */
    k_poll_signal_init(&spi_done_sig);
    k_poll_event_init(&spi_done_event, K_POLL_TYPE_SIGNAL,
                      K_POLL_MODE_NOTIFY_ONLY, &spi_done_sig);

    /* Start connector thread */
    connector_thread_id = k_thread_create(
        &connector_thread_data,
        connector_thread_stack,
        K_THREAD_STACK_SIZEOF(connector_thread_stack),
        connector_thread_fn,
        NULL, NULL, NULL,
        CONNECTOR_THREAD_PRIORITY,
        0,
        K_NO_WAIT);

    k_thread_name_set(connector_thread_id, "cf_connector");

    LOG_DBG("Crazyflie connector initialized");
    return 0;
}

SYS_INIT(connector_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
