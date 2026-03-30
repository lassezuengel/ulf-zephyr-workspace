/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Node table abstraction for tracking other nodes in the network.
 */

#ifndef NODE_TABLE_H
#define NODE_TABLE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <zephyr/sys/util.h>

#define NODE_TABLE_FLAG_ROOT      BIT(0)  /* Node is network root */
#define NODE_TABLE_FLAG_NEIGHBOR  BIT(1)  /* Direct neighbor (1-hop) */
#define NODE_TABLE_FLAG_ANCHOR    BIT(2)  /* Static anchor node */

/**
 * @brief Node entry structure
 *
 * Stores information about a node in the network.
 * Size: 32 bytes per entry (with padding)
 */
struct node_entry {
    uint16_t node_id;           /* Node address (0xFFFF = invalid) */
    uint64_t last_seen_rtc;     /* RTC timestamp of last update (32768 Hz) */
    int32_t last_distance_mm;   /* Last measured distance in mm (0 = unknown) */
    uint8_t hop_count;          /* Hop count from this node (0 = direct neighbor) */
    uint8_t flags;              /* NODE_TABLE_FLAG_* bits */
    int8_t rssi;                /* RSSI of last reception (dBm, 0 = unknown) */
    uint8_t _reserved;          /* Padding for alignment */
    /* Position data (valid if FLAG_ANCHOR set) */
    float pos_x;                /* X coordinate in meters */
    float pos_y;                /* Y coordinate in meters */
    float pos_z;                /* Z coordinate in meters */
};

/**
 * @brief Initialize the node table
 *
 * Clears all entries and prepares the table for use.
 * Safe to call multiple times.
 *
 * @return 0 on success
 */
int node_table_init(void);

/**
 * @brief Update or create a node entry with distance measurement
 *
 * If the node doesn't exist, creates a new entry.
 * If the table is full, returns -ENOMEM.
 *
 * @param node_id Node address
 * @param distance_mm Distance in millimeters
 * @param rtc Current RTC timestamp
 * @return 0 on success, negative errno on failure
 */
int node_table_update(uint16_t node_id, int32_t distance_mm, uint64_t rtc);

/**
 * @brief Update or create a node entry with hop count
 *
 * @param node_id Node address
 * @param hop_count Number of hops to reach this node
 * @param rtc Current RTC timestamp
 * @return 0 on success, negative errno on failure
 */
int node_table_update_hop_count(uint16_t node_id, uint8_t hop_count, uint64_t rtc);

/**
 * @brief Update RSSI for a node
 *
 * @param node_id Node address
 * @param rssi RSSI value in dBm
 * @param rtc Current RTC timestamp
 * @return 0 on success, negative errno on failure
 */
int node_table_update_rssi(uint16_t node_id, int8_t rssi, uint64_t rtc);

/**
 * @brief Set flags on a node entry
 *
 * @param node_id Node address (must exist)
 * @param flags Flags to set (ORed with existing)
 * @return 0 on success, -ENOENT if node not found
 */
int node_table_set_flags(uint16_t node_id, uint8_t flags);

/**
 * @brief Clear flags on a node entry
 *
 * @param node_id Node address (must exist)
 * @param flags Flags to clear
 * @return 0 on success, -ENOENT if node not found
 */
int node_table_clear_flags(uint16_t node_id, uint8_t flags);

/**
 * @brief Get a node entry by ID
 *
 * @param node_id Node address
 * @return Pointer to entry (valid until next table modification), or NULL
 */
struct node_entry *node_table_get(uint16_t node_id);

/**
 * @brief Check if a node exists in the table
 *
 * @param node_id Node address
 * @return true if node exists
 */
bool node_table_exists(uint16_t node_id);

/**
 * @brief Get current number of nodes in the table
 *
 * @return Number of valid entries
 */
size_t node_table_get_count(void);

/**
 * @brief Copy all node entries to output buffer
 *
 * @param out Output buffer
 * @param max Maximum entries to copy
 * @return Number of entries copied
 */
size_t node_table_get_all(struct node_entry *out, size_t max);

/**
 * @brief Copy neighbor entries (hop_count == 0 or FLAG_NEIGHBOR set)
 *
 * @param out Output buffer
 * @param max Maximum entries to copy
 * @return Number of entries copied
 */
size_t node_table_get_neighbors(struct node_entry *out, size_t max);

/**
 * @brief Copy anchor entries (FLAG_ANCHOR set with valid position)
 *
 * Returns only nodes that have FLAG_ANCHOR set and have reported their position.
 * These are nodes with static positions suitable for localization.
 *
 * @param out Output buffer
 * @param max Maximum entries to copy
 * @return Number of entries copied
 */
size_t node_table_get_anchors(struct node_entry *out, size_t max);

/**
 * @brief Update node position and set anchor flag
 *
 * Updates the position coordinates for a node and sets FLAG_ANCHOR.
 * Creates the entry if it doesn't exist.
 *
 * @param node_id Node address
 * @param x X coordinate in meters
 * @param y Y coordinate in meters
 * @param z Z coordinate in meters
 * @param rtc Current RTC timestamp
 * @return 0 on success, negative errno on failure
 */
int node_table_update_position(uint16_t node_id, float x, float y, float z, uint64_t rtc);

/**
 * @brief Get node entry by index (for iteration)
 *
 * @param index Index (0 to node_table_get_count()-1)
 * @return Pointer to entry, or NULL if index out of range
 */
struct node_entry *node_table_get_by_index(size_t index);

/**
 * @brief Remove expired entries from the table
 *
 * Removes entries where (current_rtc - last_seen_rtc) > timeout_ticks
 *
 * @param current_rtc Current RTC timestamp
 * @param timeout_ticks Expiration threshold in RTC ticks
 */
void node_table_expire(uint64_t current_rtc, uint64_t timeout_ticks);

/**
 * @brief Remove a specific node from the table
 *
 * @param node_id Node address to remove
 */
void node_table_remove(uint16_t node_id);

/**
 * @brief Clear all entries from the table
 */
void node_table_clear(void);

/**
 * @brief Callback type for node table changes
 *
 * Called when a node is added, updated, or removed.
 * entry is NULL when node is removed.
 *
 * WARNING: Called while holding internal mutex. Do not block or
 * call back into node_table functions.
 */
typedef void (*node_table_changed_cb_t)(uint16_t node_id, const struct node_entry *entry);

/**
 * @brief Register callback for node table changes
 *
 * Only one callback can be registered. Passing NULL unregisters.
 *
 * @param cb Callback function
 */
void node_table_register_callback(node_table_changed_cb_t cb);

/**
 * @brief Get sorted array of node IDs from the table
 *
 * Copies node IDs to output buffer, sorted by node_id for deterministic ordering.
 * This ensures all nodes produce the same order for hash-based scheduling.
 *
 * @param out Output buffer for node IDs
 * @param max Maximum entries to copy
 * @return Number of entries copied
 */
size_t node_table_get_node_ids(uint16_t *out, size_t max);

#endif /* NODE_TABLE_H */
