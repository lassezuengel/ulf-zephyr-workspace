/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_APP_LIB_COMMUNICATION_MAC_QUEUE_H_
#define ZEPHYR_INCLUDE_APP_LIB_COMMUNICATION_MAC_QUEUE_H_

#include <zephyr/kernel.h>
#include <zephyr/net/ieee802154.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MAC Frame Queue Abstraction
 *
 * Provides a unified interface for MAC-level frame exchange between
 * SynchroFly blocks and applications. Supports dual TX/RX queues with
 * ISR-safe operation and back-pressure handling.
 */

/**
 * @brief Frame flags for MAC queue frames
 */
#define MAC_QUEUE_FLAG_BROADCAST    (1 << 0)  /* Frame is broadcast */
#define MAC_QUEUE_FLAG_NEEDS_ACK    (1 << 1)  /* Frame requires acknowledgment */
#define MAC_QUEUE_FLAG_GLOSSY_HINT  (1 << 2)  /* Prefer glossy transmission */
#define MAC_QUEUE_FLAG_URGENT       (1 << 3)  /* High priority MAC protocol frame */

/**
 * @brief MAC queue frame structure
 *
 * Represents a frame in raw PSDU form for transmission or reception.
 * The PSDU includes the MAC header but excludes the FCS (hardware handled).
 */
struct mac_queue_frame {
    size_t length;                              /* PSDU length in bytes */
    uint8_t psdu[IEEE802154_MAX_PHY_PACKET_SIZE]; /* Raw IEEE 802.15.4 frame */
    uint16_t dest_pan;                          /* Destination PAN ID (host order) */
    uint16_t dest_addr;                         /* Destination short addr (0xffff = broadcast) */
    uint8_t flags;                              /* Frame flags (MAC_QUEUE_FLAG_*) */
    uint32_t metadata;                          /* Timestamp, sequence, or user tag */
};

/**
 * @brief MAC queue statistics
 */
struct mac_queue_stats {
    /* TX queue statistics */
    uint32_t tx_enqueued;                       /* Total frames enqueued for TX */
    uint32_t tx_dequeued;                       /* Total frames dequeued for TX */
    uint32_t tx_dropped;                        /* Frames dropped due to full TX queue */
    uint32_t tx_current_depth;                  /* Current TX queue depth */

    /* RX queue statistics */
    uint32_t rx_enqueued;                       /* Total frames enqueued for RX */
    uint32_t rx_dequeued;                       /* Total frames dequeued for RX */
    uint32_t rx_dropped;                        /* Frames dropped due to full RX queue */
    uint32_t rx_current_depth;                  /* Current RX queue depth */

    /* Error statistics */
    uint32_t invalid_frames;                    /* Frames rejected due to validation errors */
    uint32_t oversized_frames;                  /* Frames too large for queue */
};

/**
 * @brief Initialize the MAC queue system
 *
 * Must be called before using any other MAC queue functions.
 * This function is idempotent and can be called multiple times safely.
 *
 * @return 0 on success, negative error code on failure
 */
int mac_queue_init(void);

/**
 * @brief Push a frame to the TX queue
 *
 * Enqueues a frame for transmission during the next available communication
 * opportunity. If the queue is full, the behavior depends on the timeout:
 * - K_NO_WAIT: Returns -ENOMSG immediately
 * - K_FOREVER: Blocks until space is available
 * - Finite timeout: Blocks up to the specified time
 *
 * @param frame Pointer to frame to enqueue (copied)
 * @param timeout Maximum time to wait for queue space
 * @return 0 on success, -ENOMSG if queue full (K_NO_WAIT), -EAGAIN if timeout
 */
int mac_queue_tx_push(const struct mac_queue_frame *frame, k_timeout_t timeout);

/**
 * @brief Pop a frame from the TX queue
 *
 * Retrieves the next frame to be transmitted. This function is non-blocking
 * and is typically called by block handlers during communication opportunities.
 *
 * @param frame Pointer to buffer to store the frame
 * @return 0 on success, -EAGAIN if queue is empty
 */
int mac_queue_tx_pop(struct mac_queue_frame *frame);

/**
 * @brief Push a received frame to the RX queue
 *
 * Called by block handlers when a MAC payload is received from the network.
 * This function never blocks and drops the frame if the queue is full.
 *
 * @param frame Pointer to received frame (copied)
 * @return 0 on success, -ENOMEM if queue is full
 */
int mac_queue_rx_push(const struct mac_queue_frame *frame);

/**
 * @brief Pop a frame from the RX queue
 *
 * Retrieves the next received frame for application processing.
 * Can block waiting for frames if desired.
 *
 * @param frame Pointer to buffer to store the frame
 * @param timeout Maximum time to wait for a frame
 * @return 0 on success, -EAGAIN if no frame available
 */
int mac_queue_rx_pop(struct mac_queue_frame *frame, k_timeout_t timeout);

/**
 * @brief Get current TX queue depth
 *
 * @return Number of frames currently in TX queue
 */
size_t mac_queue_tx_depth(void);

/**
 * @brief Get current RX queue depth
 *
 * @return Number of frames currently in RX queue
 */
size_t mac_queue_rx_depth(void);

/**
 * @brief Clear all frames from TX queue
 *
 * Removes all pending transmission frames. Useful for applications
 * that want to ensure only the most recent data is transmitted.
 *
 * @return Number of frames that were cleared
 */
size_t mac_queue_tx_clear(void);

/**
 * @brief Clear all frames from RX queue
 *
 * Removes all pending received frames.
 *
 * @return Number of frames that were cleared
 */
size_t mac_queue_rx_clear(void);

/**
 * @brief Get queue statistics
 *
 * @param stats Pointer to structure to store statistics
 */
void mac_queue_get_stats(struct mac_queue_stats *stats);

/**
 * @brief Reset queue statistics
 *
 * Clears all statistics counters except current queue depths.
 */
void mac_queue_reset_stats(void);

/*
 * Helper functions for common frame operations
 */

/**
 * @brief Prepare a broadcast frame
 *
 * Constructs a standard IEEE 802.15.4 data frame with broadcast destination
 * using the locally configured PAN ID and node address.
 *
 * @param frame Pointer to frame structure to populate
 * @param payload Pointer to payload data
 * @param len Length of payload data
 * @return 0 on success, negative error code on failure
 */
int mac_queue_prepare_broadcast(struct mac_queue_frame *frame,
                               const uint8_t *payload, size_t len);

/**
 * @brief Check if frame is broadcast
 *
 * @param frame Pointer to frame to check
 * @return true if frame is broadcast, false otherwise
 */
bool mac_queue_is_broadcast(const struct mac_queue_frame *frame);

/**
 * @brief Extract payload from a frame
 *
 * Extracts the payload portion of an IEEE 802.15.4 frame,
 * skipping the MAC header.
 *
 * @param frame Pointer to frame to extract from
 * @param buf Buffer to store extracted payload
 * @param len Pointer to buffer size (input) and actual length (output)
 * @return 0 on success, negative error code on failure
 */
int mac_queue_extract_payload(const struct mac_queue_frame *frame,
                             uint8_t *buf, size_t *len);

/**
 * @brief Extract source address from a MAC queue frame
 *
 * Parses the IEEE 802.15.4 header and extracts the source address.
 *
 * @param frame Frame to extract source address from
 * @param src_addr Pointer to store the extracted source address
 * @return 0 on success, negative error code on failure
 */
int mac_queue_extract_src_addr(const struct mac_queue_frame *frame, uint16_t *src_addr);

/**
 * @brief Prepare a received frame for RX queue
 *
 * Constructs a standard IEEE 802.15.4 data frame using the provided
 * source address (the original sender). Use this when reconstructing
 * received frames for the RX queue.
 *
 * @param frame Pointer to frame structure to populate
 * @param src_addr Source address of the original sender
 * @param payload Pointer to payload data
 * @param len Length of payload data
 * @return 0 on success, negative error code on failure
 */
int mac_queue_prepare_rx_frame(struct mac_queue_frame *frame,
                               uint16_t src_addr,
                               const uint8_t *payload, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_APP_LIB_COMMUNICATION_MAC_QUEUE_H_ */
