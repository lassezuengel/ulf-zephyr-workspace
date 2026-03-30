/**
 * @file block_type_registry.h
 * @brief Registry mapping block types to handlers and config converters
 *
 * This module provides type-safe conversion between BLE-serializable
 * block configurations and runtime block configurations with callbacks.
 */

#ifndef SYNCHROFLY_BLOCK_TYPE_REGISTRY_H
#define SYNCHROFLY_BLOCK_TYPE_REGISTRY_H

#include <stddef.h>
#include <app/lib/blocks/block_types.h>
#include <app/lib/blocks/blocks.h>

/**
 * @brief Generic block result callback type
 *
 * This is a unified callback signature that apps register to receive
 * block results. The actual result data depends on block type.
 */
typedef void (*block_result_cb_t)(block_type_t type, void *result, void *user_data);

/**
 * @brief Function to build runtime config from BLE config
 *
 * @param ble_config BLE-serializable block configuration
 * @param result_cb Callback to invoke with block results (may be NULL)
 * @param cb_user_data User data for the callback
 * @param out_runtime_config Output runtime configuration (pre-allocated)
 * @return 0 on success, negative error code on failure
 */
typedef int (*build_runtime_config_fn)(const struct block_config_ble *ble_config,
                                       block_result_cb_t result_cb,
                                       void *cb_user_data,
                                       void *out_runtime_config);

/**
 * @brief Function to populate BLE config from runtime config
 *
 * @param runtime_config Runtime block configuration
 * @param out_ble_config Output BLE-serializable configuration
 * @return 0 on success, negative error code on failure
 */
typedef int (*build_ble_config_fn)(const void *runtime_config,
                                   struct block_config_ble *out_ble_config);

/**
 * @brief Block type information
 */
struct block_type_info {
    block_type_t type;                      /**< Block type enum value */
    const char *name;                       /**< Human-readable name */
    block_handler_t handler;                /**< Block handler function */
    size_t runtime_config_size;             /**< Size of runtime config struct */
    build_runtime_config_fn build_runtime;  /**< BLE -> runtime converter */
    build_ble_config_fn build_ble;          /**< Runtime -> BLE converter */
};

/**
 * @brief Get block type info from registry
 *
 * @param type Block type to look up
 * @return Pointer to block type info, or NULL if not found
 */
const struct block_type_info *block_type_get_info(block_type_t type);

/**
 * @brief Get block type name
 *
 * @param type Block type
 * @return Human-readable name, or "unknown" if not found
 */
const char *block_type_get_name(block_type_t type);

/**
 * @brief Validate a BLE block configuration
 *
 * @param config BLE block configuration to validate
 * @return 0 if valid, negative error code describing issue
 */
int block_type_validate_config(const struct block_config_ble *config);

#endif /* SYNCHROFLY_BLOCK_TYPE_REGISTRY_H */
