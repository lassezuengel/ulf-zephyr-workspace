#ifndef ANNOUNCEMENT_BLOCK_H
#define ANNOUNCEMENT_BLOCK_H

#include <zephyr/kernel.h>
#include <stdint.h>

/**
 * @brief Runtime configuration for the announcement block
 */
struct announcement_block_config {
    uint16_t max_depth;
    uint16_t transmission_delay_us;
    uint16_t guard_period_us;
    uint8_t channel;
    uint8_t announce_probability_pct; /**< 0-100: probability of initiating flood this round */
};

/**
 * @brief Payload flooded during announcement (fits in 50-byte glossy payload)
 *
 * Contains the announcing node's identity and state. Received by all
 * nodes in the network and used to populate their node tables.
 */
struct __attribute__((__packed__)) announcement_payload {
    uint16_t node_addr;     /**< Announcing node's address */
    float pos_x;            /**< Position X (meters) */
    float pos_y;            /**< Position Y (meters) */
    float pos_z;            /**< Position Z (meters) */
    uint8_t mode;           /**< Node mode (node_mode_t) */
    uint8_t position_mode;  /**< Position mode (node_position_mode_t) */
    uint8_t flags;          /**< Node flags (root, anchor, etc.) */
};
/* Size: 2 + 4 + 4 + 4 + 1 + 1 + 1 = 17 bytes */

void announcement_block_handler(uint64_t event_time, void *user_data);

#endif /* ANNOUNCEMENT_BLOCK_H */
