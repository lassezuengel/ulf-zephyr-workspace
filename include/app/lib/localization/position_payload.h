/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Position payload structures and functions for embedding position
 * information in ranging frames via MAC queue.
 */

#ifndef POSITION_PAYLOAD_H
#define POSITION_PAYLOAD_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <zephyr/sys/util.h>

#define POSITION_PAYLOAD_VERSION      0x01
#define POSITION_PAYLOAD_FLAG_ANCHOR  BIT(0)  /* Node has static position (is anchor) */

/**
 * @brief Position payload header (2 bytes)
 *
 * All nodes send this header. Anchors additionally include position data.
 */
struct position_payload_header {
    uint8_t version;    /* Payload version (POSITION_PAYLOAD_VERSION) */
    uint8_t flags;      /* POSITION_PAYLOAD_FLAG_* bits */
} __packed;

/**
 * @brief Anchor position payload (14 bytes total)
 *
 * Sent by anchor nodes (position mode = STATIC).
 * Contains header + 3D position coordinates.
 */
struct position_anchor_payload {
    struct position_payload_header header;
    float pos_x;        /* X coordinate in meters */
    float pos_y;        /* Y coordinate in meters */
    float pos_z;        /* Z coordinate in meters */
} __packed;

/* Payload sizes */
#define POSITION_PAYLOAD_HEADER_SIZE   sizeof(struct position_payload_header)
#define POSITION_PAYLOAD_ANCHOR_SIZE   sizeof(struct position_anchor_payload)
#define POSITION_PAYLOAD_MAX_SIZE      POSITION_PAYLOAD_ANCHOR_SIZE

/**
 * @brief Build position payload for transmission
 *
 * Builds a position payload based on this node's position mode:
 * - STATIC (anchor): Includes header + position coordinates
 * - LEAST_SQUARES/BELIEF_PROPAGATION (mobile): Header only
 *
 * @param buf Output buffer (must be at least POSITION_PAYLOAD_MAX_SIZE)
 * @param buf_size Size of output buffer
 * @return Number of bytes written, or negative errno on failure
 */
int position_payload_build(uint8_t *buf, size_t buf_size);

/**
 * @brief Process received position payload
 *
 * Parses a received position payload and updates the neighbor table
 * if the sender is an anchor node.
 *
 * @param sender_id Node ID of the sender
 * @param buf Payload buffer
 * @param len Payload length
 * @param rtc Current RTC timestamp
 * @return 0 on success, negative errno on failure
 */
int position_payload_process(uint16_t sender_id, const uint8_t *buf, size_t len, uint64_t rtc);

/**
 * @brief Check if payload indicates an anchor node
 *
 * @param buf Payload buffer
 * @param len Payload length
 * @return true if sender is an anchor, false otherwise
 */
bool position_payload_is_anchor(const uint8_t *buf, size_t len);

#endif /* POSITION_PAYLOAD_H */
