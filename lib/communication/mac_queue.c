/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/lib/communication/mac_queue.h>
#include <app/lib/system/node.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ieee802154.h>
#include <string.h>

LOG_MODULE_REGISTER(mac_queue, LOG_LEVEL_INF);

/*
 * Configuration validation
 */
BUILD_ASSERT(CONFIG_MAC_QUEUE_TX_DEPTH > 0, "TX queue depth must be positive");
BUILD_ASSERT(CONFIG_MAC_QUEUE_RX_DEPTH > 0, "RX queue depth must be positive");
BUILD_ASSERT(CONFIG_MAC_QUEUE_TX_DEPTH <= 255, "TX queue depth too large");
BUILD_ASSERT(CONFIG_MAC_QUEUE_RX_DEPTH <= 255, "RX queue depth too large");

/*
 * Internal state
 */
static bool initialized = false;

/* Message queues for TX and RX frames */
K_MSGQ_DEFINE(tx_queue, sizeof(struct mac_queue_frame), CONFIG_MAC_QUEUE_TX_DEPTH, 4);
K_MSGQ_DEFINE(rx_queue, sizeof(struct mac_queue_frame), CONFIG_MAC_QUEUE_RX_DEPTH, 4);

/* Statistics tracking */
static struct mac_queue_stats stats;
static struct k_spinlock stats_lock;

/*
 * Internal helper functions
 */

static bool validate_frame(const struct mac_queue_frame *frame)
{
    if (!frame) {
        return false;
    }

    if (frame->length == 0 || frame->length > IEEE802154_MAX_PHY_PACKET_SIZE) {
        LOG_WRN("Invalid frame length: %zu", frame->length);
        return false;
    }

    /* Basic IEEE 802.15.4 frame validation */
    if (frame->length < 3) {  /* FCF (2) + Sequence (1) minimum */
        LOG_WRN("Frame too short for IEEE 802.15.4: %zu bytes", frame->length);
        return false;
    }

    return true;
}

static void update_stats_tx_enqueue(bool success)
{
    k_spinlock_key_t key = k_spin_lock(&stats_lock);

    if (success) {
        stats.tx_enqueued++;
    } else {
        stats.tx_dropped++;
    }
    stats.tx_current_depth = k_msgq_num_used_get(&tx_queue);

    k_spin_unlock(&stats_lock, key);
}

static void update_stats_tx_dequeue(void)
{
    k_spinlock_key_t key = k_spin_lock(&stats_lock);

    stats.tx_dequeued++;
    stats.tx_current_depth = k_msgq_num_used_get(&tx_queue);

    k_spin_unlock(&stats_lock, key);
}

static void update_stats_rx_enqueue(bool success)
{
    k_spinlock_key_t key = k_spin_lock(&stats_lock);

    if (success) {
        stats.rx_enqueued++;
    } else {
        stats.rx_dropped++;
    }
    stats.rx_current_depth = k_msgq_num_used_get(&rx_queue);

    k_spin_unlock(&stats_lock, key);
}

static void update_stats_rx_dequeue(void)
{
    k_spinlock_key_t key = k_spin_lock(&stats_lock);

    stats.rx_dequeued++;
    stats.rx_current_depth = k_msgq_num_used_get(&rx_queue);

    k_spin_unlock(&stats_lock, key);
}

static void update_stats_invalid_frame(void)
{
    k_spinlock_key_t key = k_spin_lock(&stats_lock);
    stats.invalid_frames++;
    k_spin_unlock(&stats_lock, key);
}

/*
 * Public API implementation
 */

int mac_queue_init(void)
{
    if (initialized) {
        return 0;
    }

    /* Clear message queues */
    k_msgq_purge(&tx_queue);
    k_msgq_purge(&rx_queue);

    /* Reset statistics */
    memset(&stats, 0, sizeof(stats));

    initialized = true;

    LOG_INF("MAC queue initialized: TX depth=%d, RX depth=%d",
            CONFIG_MAC_QUEUE_TX_DEPTH, CONFIG_MAC_QUEUE_RX_DEPTH);

    return 0;
}

int mac_queue_tx_push(const struct mac_queue_frame *frame, k_timeout_t timeout)
{
    if (!initialized) {
        return -ENODEV;
    }

    if (!validate_frame(frame)) {
        update_stats_invalid_frame();
        return -EINVAL;
    }

    int ret = k_msgq_put(&tx_queue, frame, timeout);

    if (ret == 0) {
        LOG_DBG("TX frame enqueued: len=%zu, dest=0x%04x, flags=0x%02x",
                frame->length, frame->dest_addr, frame->flags);
        LOG_HEXDUMP_DBG(frame->psdu, frame->length, "TX enqueue payload:");
        update_stats_tx_enqueue(true);
    } else {
        if (ret == -ENOMSG) {
            LOG_DBG("TX queue full, frame dropped");
        }
        update_stats_tx_enqueue(false);
    }

    return ret;
}

int mac_queue_tx_pop(struct mac_queue_frame *frame)
{
    if (!initialized) {
        return -ENODEV;
    }

    if (!frame) {
        return -EINVAL;
    }

    int ret = k_msgq_get(&tx_queue, frame, K_NO_WAIT);

    if (ret == 0) {
        LOG_DBG("TX frame dequeued: len=%zu, dest=0x%04x, flags=0x%02x",
                frame->length, frame->dest_addr, frame->flags);
        LOG_HEXDUMP_DBG(frame->psdu, frame->length, "TX dequeue payload:");
        update_stats_tx_dequeue();
    }

    return ret;
}

int mac_queue_rx_push(const struct mac_queue_frame *frame)
{
    if (!initialized) {
        return -ENODEV;
    }

    if (!validate_frame(frame)) {
        update_stats_invalid_frame();
        return -EINVAL;
    }

    int ret = k_msgq_put(&rx_queue, frame, K_NO_WAIT);

    if (ret == 0) {
        LOG_DBG("RX frame enqueued: len=%zu, dest=0x%04x, flags=0x%02x",
                frame->length, frame->dest_addr, frame->flags);
        LOG_HEXDUMP_DBG(frame->psdu, frame->length, "RX enqueue payload:");
        update_stats_rx_enqueue(true);
    } else {
        LOG_DBG("RX queue full, frame dropped");
        update_stats_rx_enqueue(false);
    }

    return ret;
}

int mac_queue_rx_pop(struct mac_queue_frame *frame, k_timeout_t timeout)
{
    if (!initialized) {
        return -ENODEV;
    }

    if (!frame) {
        return -EINVAL;
    }

    int ret = k_msgq_get(&rx_queue, frame, timeout);

    if (ret == 0) {
        LOG_DBG("RX frame dequeued: len=%zu, dest=0x%04x, flags=0x%02x",
                frame->length, frame->dest_addr, frame->flags);
        LOG_HEXDUMP_DBG(frame->psdu, frame->length, "RX dequeue payload:");
        update_stats_rx_dequeue();
    }

    return ret;
}

size_t mac_queue_tx_depth(void)
{
    if (!initialized) {
        return 0;
    }

    return k_msgq_num_used_get(&tx_queue);
}

size_t mac_queue_rx_depth(void)
{
    if (!initialized) {
        return 0;
    }

    return k_msgq_num_used_get(&rx_queue);
}

size_t mac_queue_tx_clear(void)
{
    if (!initialized) {
        return 0;
    }

    size_t cleared = k_msgq_num_used_get(&tx_queue);
    k_msgq_purge(&tx_queue);

    LOG_DBG("TX queue cleared: %zu frames removed", cleared);

    /* Update statistics */
    k_spinlock_key_t key = k_spin_lock(&stats_lock);
    stats.tx_current_depth = 0;
    k_spin_unlock(&stats_lock, key);

    return cleared;
}

size_t mac_queue_rx_clear(void)
{
    if (!initialized) {
        return 0;
    }

    size_t cleared = k_msgq_num_used_get(&rx_queue);
    k_msgq_purge(&rx_queue);

    LOG_DBG("RX queue cleared: %zu frames removed", cleared);

    /* Update statistics */
    k_spinlock_key_t key = k_spin_lock(&stats_lock);
    stats.rx_current_depth = 0;
    k_spin_unlock(&stats_lock, key);

    return cleared;
}

void mac_queue_get_stats(struct mac_queue_stats *out_stats)
{
    if (!out_stats || !initialized) {
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&stats_lock);

    /* Update current depths */
    stats.tx_current_depth = k_msgq_num_used_get(&tx_queue);
    stats.rx_current_depth = k_msgq_num_used_get(&rx_queue);

    /* Copy statistics */
    memcpy(out_stats, &stats, sizeof(stats));

    k_spin_unlock(&stats_lock, key);
}

void mac_queue_reset_stats(void)
{
    if (!initialized) {
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&stats_lock);

    /* Preserve current depths */
    uint32_t tx_depth = stats.tx_current_depth;
    uint32_t rx_depth = stats.rx_current_depth;

    memset(&stats, 0, sizeof(stats));

    stats.tx_current_depth = tx_depth;
    stats.rx_current_depth = rx_depth;

    k_spin_unlock(&stats_lock, key);

    LOG_INF("MAC queue statistics reset");
}

/*
 * Helper function implementations
 */

int mac_queue_prepare_broadcast(struct mac_queue_frame *frame,
                               const uint8_t *payload, size_t len)
{
    if (!frame || !payload) {
        return -EINVAL;
    }

    /* Check payload size limits */
    size_t header_size = 9; /* FCF(2) + SEQ(1) + DEST_PAN(2) + DEST_ADDR(2) + SRC_ADDR(2) */
    if (len > (IEEE802154_MAX_PHY_PACKET_SIZE - header_size)) {
        return -E2BIG;
    }

    /* Clear frame structure */
    memset(frame, 0, sizeof(*frame));

    /* Build IEEE 802.15.4 header for broadcast data frame */
    uint8_t *psdu = frame->psdu;
    size_t offset = 0;

    /* Frame Control Field:
     * Bits 0-2: Frame Type = 001 (Data)
     * Bit 6: Intra-PAN = 1
     * Bits 10-11: Dest Addr Mode = 10 (Short)
     * Bits 14-15: Src Addr Mode = 10 (Short)
     * FCF = 0x8841
     */
    psdu[offset++] = 0x41; /* Frame Type: Data, Intra-PAN */
    psdu[offset++] = 0x88; /* Dest: Short, Src: Short */

    /* Sequence Number */
    psdu[offset++] = 0x00; /* TODO: Use proper sequence numbering */

    /* Destination PAN ID */
    uint16_t pan_id = 0xDECA; /* Default PAN ID - TODO: Make configurable */
    psdu[offset++] = pan_id & 0xFF;
    psdu[offset++] = (pan_id >> 8) & 0xFF;

    /* Destination Address (broadcast) */
    psdu[offset++] = 0xFF;
    psdu[offset++] = 0xFF;

    /* Source Address */
    uint16_t src_addr = get_node_addr();
    psdu[offset++] = src_addr & 0xFF;
    psdu[offset++] = (src_addr >> 8) & 0xFF;

    /* Copy payload */
    memcpy(&psdu[offset], payload, len);
    offset += len;

    /* Set frame fields */
    frame->length = offset;
    frame->dest_pan = pan_id;
    frame->dest_addr = 0xFFFF;
    frame->flags = MAC_QUEUE_FLAG_BROADCAST;
    frame->metadata = 0;

    return 0;
}

bool mac_queue_is_broadcast(const struct mac_queue_frame *frame)
{
    if (!frame) {
        return false;
    }

    return (frame->flags & MAC_QUEUE_FLAG_BROADCAST) != 0;
}

int mac_queue_extract_payload(const struct mac_queue_frame *frame,
                             uint8_t *buf, size_t *len)
{
    if (!frame || !buf || !len) {
        return -EINVAL;
    }

    if (!validate_frame(frame)) {
        return -EINVAL;
    }

    /* Parse IEEE 802.15.4 header to find payload start */
    const uint8_t *psdu = frame->psdu;

    if (frame->length < 3) {
        return -EINVAL;
    }

    /* Frame Control Field */
    uint16_t fcf = psdu[0] | (psdu[1] << 8);
    size_t offset = 3; /* FCF(2) + SEQ(1) */

    /* Skip destination addressing if present */
    if (fcf & 0x0800) { /* Dest addressing mode != none */
        if (offset + 4 > frame->length) return -EINVAL;
        offset += 4; /* PAN(2) + ADDR(2) */
    }

    /* Skip source addressing if present */
    if (fcf & 0x8000) { /* Src addressing mode != none */
        if (!(fcf & 0x0040)) { /* Not intra-PAN */
            if (offset + 2 > frame->length) return -EINVAL;
            offset += 2; /* SRC_PAN(2) */
        }
        if (offset + 2 > frame->length) return -EINVAL;
        offset += 2; /* SRC_ADDR(2) */
    }

    /* Check if we have payload */
    if (offset >= frame->length) {
        *len = 0;
        return 0;
    }

    size_t payload_len = frame->length - offset;

    if (*len < payload_len) {
        *len = payload_len;
        return -E2BIG;
    }

    memcpy(buf, &psdu[offset], payload_len);
    *len = payload_len;

    return 0;
}

int mac_queue_prepare_rx_frame(struct mac_queue_frame *frame,
                               uint16_t src_addr,
                               const uint8_t *payload, size_t len)
{
    if (!frame || !payload) {
        return -EINVAL;
    }

    /* Check payload size limits */
    size_t header_size = 9; /* FCF(2) + SEQ(1) + DEST_PAN(2) + DEST_ADDR(2) + SRC_ADDR(2) */
    if (len > (IEEE802154_MAX_PHY_PACKET_SIZE - header_size)) {
        return -E2BIG;
    }

    /* Clear frame structure */
    memset(frame, 0, sizeof(*frame));

    /* Build IEEE 802.15.4 header for received broadcast frame */
    uint8_t *psdu = frame->psdu;
    size_t offset = 0;

    /* Frame Control Field:
     * Bits 0-2: Frame Type = 001 (Data)
     * Bit 6: Intra-PAN = 1
     * Bits 10-11: Dest Addr Mode = 10 (Short)
     * Bits 14-15: Src Addr Mode = 10 (Short)
     * FCF = 0x8841
     */
    psdu[offset++] = 0x41; /* Frame Type: Data, Intra-PAN */
    psdu[offset++] = 0x88; /* Dest: Short, Src: Short */

    /* Sequence Number */
    psdu[offset++] = 0x00;

    /* Destination PAN ID */
    uint16_t pan_id = 0xDECA;
    psdu[offset++] = pan_id & 0xFF;
    psdu[offset++] = (pan_id >> 8) & 0xFF;

    /* Destination Address (broadcast) */
    psdu[offset++] = 0xFF;
    psdu[offset++] = 0xFF;

    /* Source Address - use provided sender address */
    psdu[offset++] = src_addr & 0xFF;
    psdu[offset++] = (src_addr >> 8) & 0xFF;

    /* Copy payload */
    memcpy(&psdu[offset], payload, len);
    offset += len;

    /* Set frame fields */
    frame->length = offset;
    frame->dest_pan = pan_id;
    frame->dest_addr = 0xFFFF;
    frame->flags = MAC_QUEUE_FLAG_BROADCAST;
    frame->metadata = 0;

    return 0;
}

int mac_queue_extract_src_addr(const struct mac_queue_frame *frame, uint16_t *src_addr)
{
    if (!frame || !src_addr) {
        return -EINVAL;
    }

    if (!validate_frame(frame)) {
        return -EINVAL;
    }

    const uint8_t *psdu = frame->psdu;

    if (frame->length < 3) {
        return -EINVAL;
    }

    /* Frame Control Field */
    uint16_t fcf = psdu[0] | (psdu[1] << 8);
    size_t offset = 3; /* FCF(2) + SEQ(1) */

    /* Skip destination addressing if present */
    if (fcf & 0x0800) { /* Dest addressing mode != none */
        if (offset + 4 > frame->length) return -EINVAL;
        offset += 4; /* PAN(2) + ADDR(2) */
    }

    /* Check if source addressing is present */
    if (!(fcf & 0x8000)) { /* Src addressing mode == none */
        return -ENOENT; /* No source address in frame */
    }

    /* Skip source PAN if not intra-PAN */
    if (!(fcf & 0x0040)) { /* Not intra-PAN */
        if (offset + 2 > frame->length) return -EINVAL;
        offset += 2; /* SRC_PAN(2) */
    }

    /* Extract source address */
    if (offset + 2 > frame->length) return -EINVAL;
    *src_addr = psdu[offset] | (psdu[offset + 1] << 8);

    return 0;
}

/* System initialization hook */
SYS_INIT(mac_queue_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
