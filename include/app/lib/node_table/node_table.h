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

#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
#include <app/lib/ranging/time_series_store.h>
#endif

#define NODE_TABLE_FLAG_ROOT      BIT(0)  /* Node is network root */
#define NODE_TABLE_FLAG_NEIGHBOR  BIT(1)  /* Direct neighbor (1-hop) */
#define NODE_TABLE_FLAG_ANCHOR    BIT(2)  /* Static anchor node */

/**
 * @brief DS-TWR timestamps for a node pair (6 core timestamps)
 *
 * Portable representation of the raw timestamps from a single DS-TWR
 * exchange, suitable for antenna delay calibration. Uses int64_t to
 * match the firmware's overflow-corrected timestamp representation.
 */
struct node_twr_timestamps {
    uint16_t initiator_id;
    uint16_t responder_id;
    int64_t  tx_init;     /* Initiator TX poll */
    int64_t  rx_init;     /* Responder RX poll */
    int64_t  tx_resp;     /* Responder TX response */
    int64_t  rx_resp;     /* Initiator RX response */
    int64_t  tx_final;    /* Initiator TX final */
    int64_t  rx_final;    /* Responder RX final */
};

/**
 * @brief Node entry structure
 *
 * Stores information about a node in the network.
 * Size varies depending on CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED.
 */
struct node_entry {
    uint16_t node_id;           /* Node address (0xFFFF = invalid) */
    uint64_t last_seen_rtc;     /* RTC timestamp of last update (32768 Hz) */
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    ts_series_float_t *distance_series;           /* Raw time series -- ALL samples (meters) */
    ts_series_float_t *filtered_distance_series;  /* Outlier-gated -- clean samples only (meters) */
#else
    int32_t last_distance_mm;   /* Last measured distance in mm (0 = unknown) */
#endif
#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
    /* Latest single-channel MM snapshot (for BLE streaming) */
    int32_t  last_twr_distance_um;    /* TWR distance in micrometers */
    int16_t  last_phase_mrad;         /* Fine phase in milliradians */
    int16_t  last_coarse_phase_mrad;  /* Coarse phase in milliradians */
    uint8_t  last_mm_channel;         /* UWB channel number */
    int16_t  last_cfo_ppb;            /* Clock frequency offset in ppb */
#endif
    uint8_t hop_count;          /* Hop count from this node (0 = direct neighbor) */
    uint8_t flags;              /* NODE_TABLE_FLAG_* bits */
    int8_t rssi;                /* RSSI of last reception (dBm, 0 = unknown) */
    uint8_t _reserved;          /* Padding for alignment */
    struct node_twr_timestamps last_twr;  /* Last DS-TWR timestamps for calibration */
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
 * @brief Update stored DS-TWR timestamps for a neighbor
 *
 * Stores the raw TWR timestamps for later retrieval (e.g. for antenna
 * delay calibration). Does NOT trigger the change callback -- call this
 * before node_table_update() so the callback fires once with both
 * distance and TWR data available.
 *
 * @param node_id Node address (must already exist)
 * @param twr Pointer to timestamp data to store
 * @return 0 on success, -ENOENT if node not found
 */
int node_table_update_twr(uint16_t node_id, const struct node_twr_timestamps *twr);

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
 * WARNING: Returns a pointer directly into the node table array. The pointer
 * is only valid until the next table modification (update, remove, expire).
 * Do NOT use across preemption points or from threads that race with
 * node_table_update(). Prefer node_table_get_copy() for safe access.
 *
 * @param node_id Node address
 * @return Pointer to entry (valid until next table modification), or NULL
 */
struct node_entry *node_table_get(uint16_t node_id);

/**
 * @brief Copy a node entry by ID (thread-safe)
 *
 * Copies the entry data under the node table mutex, so the caller gets a
 * consistent snapshot that remains valid regardless of concurrent modifications.
 *
 * @param node_id Node address
 * @param out Output buffer to copy entry into
 * @return 0 on success, -ENOENT if node not found
 */
int node_table_get_copy(uint16_t node_id, struct node_entry *out);

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
 * Called via node_table_notify_changed() OUTSIDE the node table mutex.
 * The callback may safely call any node_table API function.
 *
 * @param node_id    Node that changed
 * @param entry      Snapshot of entry data at notification time
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
 * @brief Fire change callbacks for all dirty entries
 *
 * All node_table mutating functions (update, update_rssi, set_flags,
 * update_position, etc.) mark entries as dirty internally. This function
 * snapshots the dirty set under the mutex, releases the mutex, then
 * fires the registered callback for each dirty entry.
 *
 * The callback is invoked OUTSIDE the node table mutex, so it is safe
 * to call node_table_get_filtered_distance_m() and similar APIs from
 * within the callback.
 *
 * Typical usage: call node_table_update() / update_rssi() / etc. in a
 * loop for each neighbor in a ranging round, then call
 * node_table_notify_changed() once at the end.
 */
void node_table_notify_changed(void);

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

/* ========================================================================== */
/* Distance filter strategy                                                   */
/* ========================================================================== */

/**
 * @brief Distance filter strategy for node_table_get_filtered_distance_m()
 */
enum distance_filter_strategy {
    DISTANCE_FILTER_NONE = 0,            /**< No filtering, return raw value */
    DISTANCE_FILTER_MOVING_AVERAGE = 1,  /**< Simple moving average */
    DISTANCE_FILTER_SAVITZKY_GOLAY = 2,  /**< Savitzky-Golay polynomial fit */
};

/* ========================================================================== */
/* Distance time series API                                                   */
/* ========================================================================== */

/**
 * @brief Get most recent raw distance for a node in meters
 *
 * @param node_id Node address
 * @param distance_m Output: distance in meters
 * @return 0 on success, -ENOENT if node not found, -ENODATA if no samples
 */
int node_table_get_distance_m(uint16_t node_id, float *distance_m);

/**
 * @brief Get most recent raw distance for a node in millimeters
 *
 * Convenience wrapper that converts from internal float meters.
 *
 * @param node_id Node address
 * @param distance_mm Output: distance in millimeters
 * @return 0 on success, -ENOENT if node not found, -ENODATA if no samples
 */
int node_table_get_distance_mm(uint16_t node_id, int32_t *distance_mm);

/**
 * @brief Get filtered distance for a node in meters
 *
 * Applies Savitzky-Golay filter if enabled and enough samples available.
 * Falls back to raw value if filtering not possible.
 *
 * @param node_id Node address
 * @param distance_m Output: filtered distance in meters
 * @return 0 on success, -ENOENT if node not found, -ENODATA if no samples
 */
int node_table_get_filtered_distance_m(uint16_t node_id, float *distance_m);

/**
 * @brief Get filtered distance for a node in millimeters
 *
 * Convenience wrapper that converts from internal float meters.
 *
 * @param node_id Node address
 * @param distance_mm Output: filtered distance in millimeters
 * @return 0 on success, -ENOENT if node not found, -ENODATA if no samples
 */
int node_table_get_filtered_distance_mm(uint16_t node_id, int32_t *distance_mm);

/**
 * @brief Enable or disable distance filtering at runtime
 *
 * @param enabled true to enable Savitzky-Golay filtering
 */
void node_table_set_distance_filter_enabled(bool enabled);

/**
 * @brief Check if distance filtering is enabled
 *
 * @return true if filtering is enabled
 */
bool node_table_get_distance_filter_enabled(void);

/**
 * @brief Set the distance filter strategy
 *
 * @param strategy Filter strategy to use
 */
void node_table_set_distance_filter_strategy(enum distance_filter_strategy strategy);

/**
 * @brief Get the current distance filter strategy
 *
 * @return Current filter strategy
 */
enum distance_filter_strategy node_table_get_distance_filter_strategy(void);

/**
 * @brief Set the filter window size (used by both moving average and savgol)
 *
 * @param window_size Window size (will be coerced to odd for savgol)
 */
void node_table_set_distance_filter_window_size(uint8_t window_size);

/**
 * @brief Get the current filter window size
 */
uint8_t node_table_get_distance_filter_window_size(void);

/**
 * @brief Set the Savitzky-Golay polynomial order
 *
 * @param poly_order Polynomial order (must be < window_size)
 */
void node_table_set_distance_filter_poly_order(uint8_t poly_order);

/**
 * @brief Get the current Savitzky-Golay polynomial order
 */
uint8_t node_table_get_distance_filter_poly_order(void);

/**
 * @brief Set the distance series capacity for all nodes
 *
 * Resizes existing series (clearing stored samples) and sets the
 * capacity used for newly created series.
 *
 * @param capacity Number of samples per node (1-64)
 */
void node_table_set_distance_series_capacity(uint8_t capacity);

/**
 * @brief Get the current distance series capacity
 */
uint8_t node_table_get_distance_series_capacity(void);

/* ========================================================================== */
/* Outlier pre-filter API                                                     */
/* ========================================================================== */

/**
 * @brief Enable or disable outlier pre-filtering at runtime
 *
 * When enabled, new distance samples are checked against the existing
 * time series. Samples deviating more than threshold * MAD from the
 * mean are rejected before being stored.
 *
 * @param enabled true to enable outlier rejection
 */
void node_table_set_outlier_filter_enabled(bool enabled);

/**
 * @brief Check if outlier pre-filtering is enabled
 */
bool node_table_get_outlier_filter_enabled(void);

/**
 * @brief Set the outlier filter threshold multiplier
 *
 * @param threshold_x10 MAD multiplier scaled by 10 (e.g., 30 = 3.0x MAD).
 *                      Clamped to [10, 100].
 */
void node_table_set_outlier_filter_threshold(uint8_t threshold_x10);

/**
 * @brief Get the current outlier filter threshold multiplier (x10)
 */
uint8_t node_table_get_outlier_filter_threshold(void);

/* ========================================================================== */
/* MM ranging data API                                                        */
/* ========================================================================== */

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)

/**
 * @brief Single-channel MM ranging update data
 */
struct node_mm_update {
    int32_t twr_distance_um;     /**< TWR distance in micrometers */
    int16_t phase_mrad;          /**< Fine phase in milliradians */
    int16_t coarse_phase_mrad;   /**< Coarse phase in milliradians */
    uint8_t channel;             /**< UWB channel number */
    int16_t cfo_ppb;             /**< Clock frequency offset in ppb */
};

/**
 * @brief Store single-channel MM ranging measurement for a node
 *
 * Stores scalar snapshot for BLE streaming. Marks entry dirty.
 *
 * @param node_id Node address (must already exist)
 * @param mm Pointer to update data
 * @return 0 on success, -ENOENT if node not found
 */
int node_table_update_mm(uint16_t node_id, const struct node_mm_update *mm);

/*
 * Legacy stubs -- kept so twr_mm.c (preserved for future use) still links.
 * These are no-ops; the time series are no longer stored in the node table.
 */
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
ts_series_t *node_table_get_mm_twr_series(uint16_t node_id);
ts_series_t *node_table_get_mm_phase_series(uint16_t node_id, int channel_index);
ts_series_t *node_table_get_mm_d_diff_series(uint16_t node_id);
#endif

void node_table_set_mm_filter_window_size(uint8_t size);
uint8_t node_table_get_mm_filter_window_size(void);
void node_table_set_mm_filter_poly_order(uint8_t order);
uint8_t node_table_get_mm_filter_poly_order(void);
void node_table_set_mm_series_capacity(uint8_t capacity);
uint8_t node_table_get_mm_series_capacity(void);
void node_table_set_mm_phase_filter_window_size(uint8_t size);
uint8_t node_table_get_mm_phase_filter_window_size(void);
void node_table_set_mm_phase_filter_poly_order(uint8_t order);
uint8_t node_table_get_mm_phase_filter_poly_order(void);
void node_table_set_mm_twr_filter_window_size(uint8_t size);
uint8_t node_table_get_mm_twr_filter_window_size(void);
void node_table_set_mm_twr_filter_poly_order(uint8_t order);
uint8_t node_table_get_mm_twr_filter_poly_order(void);

#endif /* CONFIG_NODE_TABLE_MM_ENABLED */

/* ========================================================================== */
/* Eviction timer API                                                         */
/* ========================================================================== */

/**
 * @brief Set the eviction timeout for stale node entries
 *
 * When set to a positive value, a periodic work item removes entries
 * older than timeout_ms. Setting to 0 disables eviction.
 *
 * @param timeout_ms Eviction timeout in milliseconds (0 = disabled)
 */
void node_table_set_eviction_timeout_ms(uint32_t timeout_ms);

/**
 * @brief Get the current eviction timeout
 *
 * @return Eviction timeout in milliseconds (0 = disabled)
 */
uint32_t node_table_get_eviction_timeout_ms(void);

#endif /* NODE_TABLE_H */
