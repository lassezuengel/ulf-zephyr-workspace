/*
 * Copyright (c) 2025 SynchroFly Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * CRTP Bridge BLE Service
 *
 * Provides a bidirectional BLE interface for CRTP communication with the
 * Crazyflie STM32:
 * - TX characteristic: BLE client writes CRTP packets to send to Crazyflie
 * - RX characteristic: BLE client receives CRTP packets from Crazyflie via notify
 *
 * This enables BLE clients to access Crazyflie log/param services through
 * the UWB deck acting as a CRTP bridge.
 *
 * RX packets from Crazyflie are queued in a ring buffer and sent via a
 * work queue to ensure thread safety with the BLE stack.
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <app/lib/crazyflie/connector.h>

#include "crtp_service.h"

LOG_MODULE_REGISTER(crtp_service, CONFIG_CRTP_SERVICE_LOG_LEVEL);

/* Maximum CRTP packet size: 1 byte header + 30 bytes payload */
#define CRTP_MAX_PACKET_SIZE 31

/* Ring buffer for CRTP RX packets
 * Each entry: 1 byte length + up to 31 bytes data = 32 bytes max
 * Increased to 1536 bytes (~48 packets) for TOC fetch throughput
 * Note: Keep conservative for DWM1001's 64KB RAM constraint */
#define CRTP_RX_RING_SIZE 1536
static uint8_t crtp_rx_ring_data[CRTP_RX_RING_SIZE];
static struct ring_buf crtp_rx_ring;

/* Delayable work for sending CRTP RX notifications
 * Using system workqueue to run in BT-safe context.
 * Delayable allows retry with backoff when BLE TX buffer is full. */
static struct k_work_delayable crtp_rx_work;

/* Service UUID */
static struct bt_uuid_128 crtp_service_uuid = BT_UUID_INIT_128(
    CRTP_SERVICE_UUID_VAL);

/* TX characteristic UUID (BLE -> Crazyflie) */
static struct bt_uuid_128 crtp_tx_uuid = BT_UUID_INIT_128(
    CRTP_TX_CHAR_UUID_VAL);

/* RX characteristic UUID (Crazyflie -> BLE) */
static struct bt_uuid_128 crtp_rx_uuid = BT_UUID_INIT_128(
    CRTP_RX_CHAR_UUID_VAL);

/* Statistics */
static uint32_t crtp_packets_sent_to_cf = 0;
static uint32_t crtp_packets_from_cf = 0;
static uint32_t crtp_packets_dropped = 0;

/* RX notification state */
static bool rx_notify_enabled = false;
static const struct bt_gatt_attr *rx_attr = NULL;

/**
 * Write handler for CRTP TX characteristic.
 * Forwards the CRTP packet to the Crazyflie via SPI HCI.
 */
static ssize_t write_crtp_tx(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len,
                             uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    /* Validate packet size */
    if (len < 1 || len > CRTP_MAX_PACKET_SIZE) {
        LOG_WRN("Invalid CRTP packet size: %d (expected 1-%d)",
                len, CRTP_MAX_PACKET_SIZE);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = buf;
    uint8_t header = data[0];
    uint8_t port = (header >> 4) & 0x0F;
    uint8_t channel = header & 0x03;

    LOG_DBG("CRTP TX (BLE->CF): port=%d ch=%d len=%d", port, channel, len);

    /* Forward to Crazyflie via SPI connector */
    int ret = crazyflie_send_crtp(data, len);
    if (ret < 0) {
        crtp_packets_dropped++;
        if (ret == -EAGAIN) {
            LOG_WRN("CRTP queue full, packet dropped");
            return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
        }
        LOG_ERR("Failed to send CRTP packet: %d", ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    crtp_packets_sent_to_cf++;
    LOG_INF("CRTP TX: port=%d ch=%d size=%d -> queued for CF", port, channel, len - 1);

    return len;
}

/**
 * CCC (Client Characteristic Configuration) changed callback for RX notify.
 */
static void rx_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    rx_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("CRTP RX notifications %s", rx_notify_enabled ? "enabled" : "disabled");

    /* Clear ring buffer when notifications are disabled to avoid stale data */
    if (!rx_notify_enabled) {
        ring_buf_reset(&crtp_rx_ring);
    }
}

/**
 * Work handler that sends queued CRTP packets via BLE GATT notify.
 * Runs in system workqueue context which is safe for BLE operations.
 */
static void crtp_rx_work_handler(struct k_work *work_item)
{
    uint8_t packet[CRTP_MAX_PACKET_SIZE + 1]; /* +1 for length byte */
    uint32_t sent = 0;
    uint32_t dropped = 0;

    if (!rx_notify_enabled || rx_attr == NULL) {
        /* Notifications disabled, drain the buffer */
        ring_buf_reset(&crtp_rx_ring);
        return;
    }

    /* Process all queued packets */
    while (ring_buf_size_get(&crtp_rx_ring) > 0) {
        /* Peek at length byte */
        uint8_t len;
        if (ring_buf_peek(&crtp_rx_ring, &len, 1) != 1) {
            break;
        }

        /* Validate length */
        if (len < 1 || len > CRTP_MAX_PACKET_SIZE) {
            LOG_ERR("Invalid packet length in ring buffer: %d", len);
            /* Try to recover by skipping this byte */
            ring_buf_get(&crtp_rx_ring, &len, 1);
            continue;
        }

        /* Check if full packet is available */
        if (ring_buf_size_get(&crtp_rx_ring) < (1 + len)) {
            /* Partial packet, wait for more data (shouldn't happen) */
            LOG_WRN("Partial packet in ring buffer");
            break;
        }

        /* Read length byte */
        ring_buf_get(&crtp_rx_ring, &len, 1);

        /* Read packet data */
        uint32_t read = ring_buf_get(&crtp_rx_ring, packet, len);
        if (read != len) {
            LOG_ERR("Ring buffer read mismatch: expected %d, got %d", len, read);
            break;
        }

        /* Send via BLE GATT notify */
        int ret = bt_gatt_notify(NULL, rx_attr, packet, len);
        if (ret < 0) {
            if (ret == -ENOMEM) {
                /* BLE TX buffer full - put packet back and retry later.
                 * We need to preserve packet order, so put the length byte back first,
                 * then the packet data. Since ring_buf doesn't have push_front, we use
                 * a small delay and reschedule instead. */
                LOG_DBG("BLE TX buffer full, retrying in 5ms");
                /* Re-insert the packet at the front is not possible with ring_buf,
                 * so we schedule a delayed retry. The packet is already consumed,
                 * so we need to re-queue it. */
                uint8_t len_byte = len;
                if (ring_buf_space_get(&crtp_rx_ring) >= (1 + len)) {
                    ring_buf_put(&crtp_rx_ring, &len_byte, 1);
                    ring_buf_put(&crtp_rx_ring, packet, len);
                } else {
                    LOG_WRN("Cannot re-queue packet, ring buffer full");
                    dropped++;
                }
                /* Delay before retry to let BLE drain */
                k_work_reschedule(&crtp_rx_work, K_MSEC(5));
                break;  /* Stop processing, will retry later */
            } else {
                LOG_WRN("Failed to send CRTP notify: %d", ret);
                dropped++;
            }
        } else {
            sent++;
        }
    }

    if (sent > 0 || dropped > 0) {
        crtp_packets_from_cf += sent;
        crtp_packets_dropped += dropped;
        LOG_DBG("CRTP RX work: sent=%d dropped=%d", sent, dropped);
    }
}

/**
 * Callback for CRTP packets received from Crazyflie via SPI.
 * Queues the packet in ring buffer and schedules work to send via BLE.
 * Called from SPI connector thread context.
 */
static void crtp_from_crazyflie_callback(const uint8_t *data, size_t len)
{
    if (!rx_notify_enabled) {
        LOG_DBG("CRTP RX: notifications not enabled, dropping packet");
        return;
    }

    if (len < 1 || len > CRTP_MAX_PACKET_SIZE) {
        LOG_WRN("Invalid CRTP packet from CF: len=%zu", len);
        return;
    }

    uint8_t header = data[0];
    uint8_t port = (header >> 4) & 0x0F;
    uint8_t channel = header & 0x03;

    LOG_INF("CRTP RX (CF->BLE): port=%d ch=%d len=%zu", port, channel, len);

    /* Check if there's space in the ring buffer (need len + 1 for length byte) */
    if (ring_buf_space_get(&crtp_rx_ring) < (len + 1)) {
        LOG_WRN("CRTP RX ring buffer full, dropping packet");
        crtp_packets_dropped++;
        return;
    }

    /* Write length byte followed by packet data */
    uint8_t len_byte = (uint8_t)len;
    ring_buf_put(&crtp_rx_ring, &len_byte, 1);
    ring_buf_put(&crtp_rx_ring, data, len);

    /* Schedule work to send the notification (immediate, no delay) */
    k_work_reschedule(&crtp_rx_work, K_NO_WAIT);
}

/* Service Definition */
BT_GATT_SERVICE_DEFINE(crtp_svc,
    BT_GATT_PRIMARY_SERVICE(&crtp_service_uuid),

    /* TX Characteristic - write without response for low latency (BLE -> Crazyflie) */
    BT_GATT_CHARACTERISTIC(&crtp_tx_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, write_crtp_tx, NULL),

    /* RX Characteristic - notify for responses from Crazyflie (Crazyflie -> BLE) */
    BT_GATT_CHARACTERISTIC(&crtp_rx_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(rx_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/**
 * Initialize the CRTP service.
 * Registers the callback to receive CRTP packets from the Crazyflie.
 */
static int crtp_service_init(void)
{
    /* Initialize ring buffer */
    ring_buf_init(&crtp_rx_ring, sizeof(crtp_rx_ring_data), crtp_rx_ring_data);

    /* Initialize delayable work item */
    k_work_init_delayable(&crtp_rx_work, crtp_rx_work_handler);

    /* Find the RX characteristic attribute for notifications */
    for (int i = 0; i < crtp_svc.attr_count; i++) {
        const struct bt_gatt_attr *attr = &crtp_svc.attrs[i];
        if (bt_uuid_cmp(attr->uuid, &crtp_rx_uuid.uuid) == 0) {
            rx_attr = attr;
            break;
        }
    }

    if (rx_attr == NULL) {
        LOG_ERR("Failed to find CRTP RX characteristic");
        return -ENOENT;
    }

    /* Register callback to receive CRTP packets from Crazyflie via SPI */
    crazyflie_register_crtp_rx_callback(crtp_from_crazyflie_callback);

    LOG_INF("CRTP service initialized (bidirectional, work queue enabled)");
    return 0;
}

/* Initialize after spi_connector (priority 90) */
SYS_INIT(crtp_service_init, APPLICATION, 91);
