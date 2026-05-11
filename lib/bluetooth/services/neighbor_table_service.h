/**
 * @file neighbor_table_service.h
 * @brief GATT service for exposing neighbor table information
 */

#ifndef NEIGHBOR_TABLE_SERVICE_H
#define NEIGHBOR_TABLE_SERVICE_H

#include <zephyr/types.h>

/**
 * @brief Initialize and register the neighbor table GATT service
 *
 * This service provides characteristics for reading:
 * - Neighbor count (number of known neighbors)
 * - Neighbor list (array of neighbor entries with distance, age, etc.)
 *
 * The neighbor list characteristic supports notifications.
 *
 * @return 0 on success, negative errno on failure
 */
int neighbor_table_service_init(void);

/**
 * @brief Send notification with current neighbor table data
 *
 * Sends a notification to connected clients with subscribed CCCD.
 * Call this when the neighbor table has been updated.
 *
 * @return 0 on success, negative errno on failure
 */
int neighbor_table_service_notify(void);

/**
 * @brief Send position notification if enabled
 *
 * Sends the device's current position via BLE notification.
 * Call this when the localization result has been updated.
 *
 * @param x X coordinate in meters
 * @param y Y coordinate in meters
 * @param z Z coordinate in meters
 * @return 0 on success, -ENOENT if notifications not enabled
 */
int neighbor_table_service_notify_position(float x, float y, float z);

#if defined(CONFIG_SYNCHROFLY_BLOCK_MULOC)
struct muloc_round_result;

/**
 * @brief Send MULoc round data as BLE notification
 *
 * @param round    Pointer to the round result data
 * @param anchor_id This node's anchor ID (0xFF = tag/listener)
 * @param anchor_count Number of anchors in the system
 * @return 0 on success, negative errno on failure
 */
int neighbor_table_notify_muloc_round(const struct muloc_round_result *round,
                                       uint8_t anchor_id, uint8_t anchor_count);
#endif

#if defined(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
struct cir_read_block_result;

/**
 * @brief Send CIR data as chunked BLE notifications
 *
 * Copies the CIR result into an internal buffer and starts asynchronous
 * chunked transfer via BLE notifications. Safe to call from block handler
 * context -- the actual notifications are sent via a work queue.
 *
 * @param result Pointer to the CIR read result
 * @return 0 on success, -ENOENT if notifications not enabled,
 *         -EBUSY if a transfer is already in progress
 */
int neighbor_table_service_notify_cir(const struct cir_read_block_result *result);
#endif

#endif /* NEIGHBOR_TABLE_SERVICE_H */
