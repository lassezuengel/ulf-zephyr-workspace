/**
 * @file superframe_config_service.h
 * @brief GATT service for superframe configuration
 *
 * Provides characteristics for:
 * - Block count (number of blocks in superframe)
 * - Slot index (selects which block to read/write)
 * - Block config (configuration for selected block)
 * - Apply (triggers superframe rebuild and restart)
 * - Status (reports apply status with notifications)
 * - Full config (read entire superframe in one operation)
 */

#ifndef SYNCHROFLY_BT_SUPERFRAME_CONFIG_SERVICE_H
#define SYNCHROFLY_BT_SUPERFRAME_CONFIG_SERVICE_H

/**
 * @brief Superframe service status codes
 */
enum superframe_service_status {
    SUPERFRAME_STATUS_IDLE = 0,              /**< No pending changes */
    SUPERFRAME_STATUS_PENDING = 1,           /**< Changes pending, not applied */
    SUPERFRAME_STATUS_APPLIED = 2,           /**< Changes applied successfully */
    SUPERFRAME_STATUS_ERROR_INVALID = 3,     /**< Invalid configuration */
    SUPERFRAME_STATUS_ERROR_APPLY_FAILED = 4 /**< Apply operation failed */
};

/**
 * @brief Initialize the superframe configuration GATT service
 *
 * The service is automatically registered via BT_GATT_SERVICE_DEFINE.
 * This function performs any additional initialization if needed.
 *
 * @return 0 on success, negative error code on failure
 */
int superframe_config_service_init(void);

#endif /* SYNCHROFLY_BT_SUPERFRAME_CONFIG_SERVICE_H */
