/**
 * @file swarm_uwb_interface.c
 * @brief UWB driver abstraction layer implementation
 */

#include <app/lib/swarm_ranging/swarm_uwb_interface.h>
#include <zephyr/logging/log.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/lib/system/node.h>
#include <string.h>

#include <app/lib/system/radio_arbiter.h>

LOG_MODULE_REGISTER(swarm_uwb, LOG_LEVEL_INF);

/* UWB device pointer */
static const struct device *uwb_device = NULL;
static const uwb_driver_t *uwb_driver = NULL;

/* Callback pointers */
static swarm_uwb_rx_callback_t rx_callback = NULL;
static swarm_uwb_tx_callback_t tx_callback = NULL;

/* Local address */
static uint16_t local_address = 0;

/* Last timestamps */
static uint64_t last_rx_timestamp = 0;
static uint64_t last_tx_timestamp = 0;

/* Radio access synchronization */
static struct k_mutex radio_mutex;
static volatile bool tx_in_progress = false;

/* RX buffer */
static uint8_t rx_buffer[256];

/* Message sequence number */
static uint16_t tx_sequence = 0;

/* 40-bit timestamp mask */
#define UWB_TIMESTAMP_MASK 0xFFFFFFFFFFULL

/**
 * Extract 40-bit timestamp and mask to valid range
 */
static inline uint64_t extract_timestamp(uint64_t raw_ts)
{
    return raw_ts & UWB_TIMESTAMP_MASK;
}

int swarm_uwb_init(const struct device *dev)
{
    if (dev == NULL || !device_is_ready(dev)) {
        LOG_ERR("UWB device not ready");
        return -ENODEV;
    }

    uwb_device = dev;
    uwb_driver = uwb_driver_get(dev);

    if (uwb_driver == NULL) {
        LOG_ERR("Failed to get UWB driver");
        return -ENODEV;
    }

    /* Initialize radio access mutex */
    k_mutex_init(&radio_mutex);

    /* Disable frame filtering for broadcast ranging */
    uwb_driver->set_frame_filter(uwb_device, 0, 0);

    /* Get local address from node system */
    local_address = get_node_addr();

    LOG_INF("UWB interface initialized, address: 0x%04x", local_address);

    return 0;
}

uint16_t swarm_uwb_get_address(void)
{
    return local_address;
}

int swarm_uwb_send_packet_blocking(uint8_t *data, uint16_t length)
{
    if (uwb_device == NULL || uwb_driver == NULL) {
        return -ENODEV;
    }

    if (data == NULL || length == 0 || length > 127) {
        return -EINVAL;
    }

    if (!radio_arbiter_is_available()) {
        return -EBUSY;
    }

    /* Acquire radio mutex to prevent RX thread interference */
    k_mutex_lock(&radio_mutex, K_FOREVER);

    /* Re-check after acquiring mutex to close the TOCTOU window.
     * The superframe may have claimed the radio between our first
     * check and the mutex acquisition. */
    if (!radio_arbiter_is_available()) {
        LOG_INF("tx: blocked (post-lock)");
        k_mutex_unlock(&radio_mutex);
        return -EBUSY;
    }

    tx_in_progress = true;  // Signal RX thread to stay out

    /* Wrap data in UWB_Packet_t structure */
    UWB_Packet_t packet;
    packet.header.srcAddress = local_address;
    packet.header.destAddress = UWB_DEST_ANY;  // Broadcast
    packet.header.type = UWB_RANGING_MESSAGE;
    packet.header.length = sizeof(UWB_Packet_Header_t) + length;

    memcpy(packet.payload, data, length);

    /* Acquire dev_lock for SPI access during TX setup */
    uwb_driver->acquire_device(uwb_device);

    /* Setup TX frame */
    uint16_t total_length = packet.header.length;
    uwb_driver->setup_tx_frame(uwb_device, (uint8_t *)&packet, total_length);

    /* Disable transceiver to stop any ongoing RX */
    uwb_driver->disable_txrx(uwb_device);

    /* Transmit immediately (no delayed TX) */
    int ret = uwb_driver->start_tx(uwb_device, 0);

    uwb_driver->release_device(uwb_device);

    if (ret < 0) {
        LOG_ERR("TX failed: %d", ret);
        tx_in_progress = false;
        k_mutex_unlock(&radio_mutex);
        return ret;
    }

    /* Release mutex ONLY during IRQ wait (but keep tx_in_progress flag set) */
    k_mutex_unlock(&radio_mutex);
    uwb_irq_state_e irq_state = uwb_driver->wait_for_irq(uwb_device);
    k_mutex_lock(&radio_mutex, K_FOREVER);

    if (irq_state == UWB_IRQ_TX) {
        /* Read TX timestamp (SPI access needs dev_lock) */
        uwb_driver->acquire_device(uwb_device);
        last_tx_timestamp = extract_timestamp(uwb_driver->read_tx_timestamp(uwb_device));
        uwb_driver->release_device(uwb_device);

        /* Call TX callback if registered */
        if (tx_callback != NULL) {
            tx_sequence++;
            tx_callback(last_tx_timestamp, tx_sequence);
        }

        tx_in_progress = false;
        k_mutex_unlock(&radio_mutex);
        return 0;
    } else {
        LOG_ERR("Unexpected IRQ state after TX: %s", irq_state_to_string(irq_state));
        tx_in_progress = false;
        k_mutex_unlock(&radio_mutex);
        return -EIO;
    }
}

int swarm_uwb_start_rx(void)
{
    if (uwb_device == NULL || uwb_driver == NULL) {
        return -ENODEV;
    }

    /* Enable RX with no timeout (continuous) */
    int ret = uwb_driver->enable_rx(uwb_device, 0, 0);
    if (ret < 0) {
        LOG_ERR("Failed to enable RX: %d", ret);
        return ret;
    }

    LOG_DBG("RX mode enabled");
    return 0;
}

/**
 * RX processing thread
 * Continuously waits for RX interrupts and processes received packets
 */
static void swarm_uwb_rx_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("UWB RX thread started");

    while (1) {
        /* Acquire mutex to check TX state and enable RX */
        k_mutex_lock(&radio_mutex, K_FOREVER);

        /* Check if TX is in progress - if so, skip this iteration */
        if (tx_in_progress) {
            k_mutex_unlock(&radio_mutex);
            k_yield();  // Let TX finish
            continue;
        }

        /* Check radio availability (arbiter) */
        if (!radio_arbiter_is_available()) {
            k_mutex_unlock(&radio_mutex);
            k_msleep(5);
            continue;
        }

        /* Acquire dev_lock for SPI access -- prevents concurrent SPI
         * with radio_arbiter_claim() on the timesync work queue. */
        uwb_driver->acquire_device(uwb_device);

        /* Re-check after acquiring dev_lock to close TOCTOU window.
         * The arbiter may have claimed between our check above and
         * the lock acquisition here. */
        if (!radio_arbiter_is_available()) {
            uwb_driver->release_device(uwb_device);
            k_mutex_unlock(&radio_mutex);
            k_msleep(5);
            continue;
        }

        /* Disable transceiver before enabling RX (clean state transition) */
        uwb_driver->disable_txrx(uwb_device);

        /* Enable RX with timeout so wait_for_irq() never blocks forever.
         * Without this, a preemption race between enable_rx() and
         * wait_for_irq() can cause the cancel signal to be lost,
         * leaving the thread stuck in k_sem_take(K_FOREVER). */
        int ret = uwb_driver->enable_rx(uwb_device, 50000, 0);

        uwb_driver->release_device(uwb_device);

        if (ret < 0) {
            LOG_ERR("Failed to enable RX: %d", ret);
            k_mutex_unlock(&radio_mutex);
            k_msleep(100);
            continue;
        }

        /* Release mutex before waiting for IRQ (allows TX to preempt) */
        k_mutex_unlock(&radio_mutex);

        /* Wait for IRQ */
        uwb_irq_state_e irq_state = uwb_driver->wait_for_irq(uwb_device);

        if (irq_state == UWB_IRQ_CANCELLED
#if IS_ENABLED(CONFIG_SYNCHROFLY_RADIO_ARBITER)
            || !radio_arbiter_is_available()
#endif
        ) {
            LOG_DBG("rx: yield irq=%d", irq_state);
            k_msleep(5);
            continue;
        }

        /* Reacquire mutex to process RX */
        k_mutex_lock(&radio_mutex, K_FOREVER);

        if (irq_state == UWB_IRQ_RX) {
            /* Acquire dev_lock for SPI access to read frame data */
            uwb_driver->acquire_device(uwb_device);

            /* Get frame length */
            uint16_t frame_len = uwb_driver->get_rx_frame_length(uwb_device);

            if (frame_len > 0 && frame_len <= sizeof(rx_buffer)) {
                /* Read frame */
                uwb_driver->read_rx_frame(uwb_device, rx_buffer, frame_len, 0);

                /* Read RX timestamp */
                last_rx_timestamp = extract_timestamp(
                    uwb_driver->read_rx_timestamp(uwb_device, NULL)
                );

                /* Release dev_lock -- SPI reads done */
                uwb_driver->release_device(uwb_device);

                /* Parse packet */
                UWB_Packet_t *packet = (UWB_Packet_t *)rx_buffer;

                /* Filter for ranging messages */
                if (packet->header.type == UWB_RANGING_MESSAGE &&
                    packet->header.destAddress == UWB_DEST_ANY) {

                    /* Call RX callback if registered */
                    if (rx_callback != NULL) {
                        swarm_uwb_rx_event_t event;
                        event.data = packet->payload;
                        event.length = packet->header.length - sizeof(UWB_Packet_Header_t);
                        event.rx_timestamp = last_rx_timestamp;

                        rx_callback(&event);
                    }
                }
            } else {
                /* Release dev_lock -- no valid frame to read */
                uwb_driver->release_device(uwb_device);
            }
        } else if (irq_state == UWB_IRQ_ERR) {
            LOG_WRN("RX error");
        } else if (irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT) {
            LOG_DBG("RX timeout");
        }

        k_mutex_unlock(&radio_mutex);
        k_yield();
    }
}

/* RX thread stack and data */
#ifndef CONFIG_SWARM_UWB_RX_STACK_SIZE
#define CONFIG_SWARM_UWB_RX_STACK_SIZE 2048
#endif

K_THREAD_STACK_DEFINE(swarm_uwb_rx_stack, CONFIG_SWARM_UWB_RX_STACK_SIZE);
static struct k_thread swarm_uwb_rx_thread_data;

void swarm_uwb_register_rx_callback(swarm_uwb_rx_callback_t callback)
{
    static bool rx_thread_started = false;

    rx_callback = callback;

    /* Start RX thread on first callback registration */
    if (!rx_thread_started && uwb_device != NULL) {
        k_thread_create(&swarm_uwb_rx_thread_data,
                        swarm_uwb_rx_stack,
                        K_THREAD_STACK_SIZEOF(swarm_uwb_rx_stack),
                        swarm_uwb_rx_thread,
                        NULL, NULL, NULL,
                        6, 0, K_NO_WAIT);

        k_thread_name_set(&swarm_uwb_rx_thread_data, "swarm_uwb_rx");
        rx_thread_started = true;

        LOG_INF("RX thread started");
    }
}

void swarm_uwb_register_tx_callback(swarm_uwb_tx_callback_t callback)
{
    tx_callback = callback;
}

uint64_t swarm_uwb_read_rx_timestamp(void)
{
    return last_rx_timestamp;
}

uint64_t swarm_uwb_read_tx_timestamp(void)
{
    return last_tx_timestamp;
}
