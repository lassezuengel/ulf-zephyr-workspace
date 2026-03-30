/**
 * @file superframe_settings.h
 * @brief Superframe configuration settings API
 *
 * This module provides runtime configuration of superframe structure
 * with NVS persistence. Applications set defaults via superframe_builder.c,
 * which can be overridden via BLE or loaded from NVS.
 */

#ifndef SYNCHROFLY_MANAGEMENT_SUPERFRAME_SETTINGS_H
#define SYNCHROFLY_MANAGEMENT_SUPERFRAME_SETTINGS_H

#include <app/lib/blocks/block_types.h>
#include <app/lib/scheduling/upper/block_scheduler.h>
#include <app/lib/scheduling/upper/block_type_registry.h>

/**
 * @brief Initialize superframe settings subsystem
 *
 * Loads configuration from NVS if available, otherwise uses defaults
 * set via superframe_settings_set_defaults().
 *
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_init(void);

/**
 * @brief Set default configuration
 *
 * Called by superframe_builder.c to set the app's default superframe.
 * This is used when NVS has no stored configuration.
 *
 * @param config Default configuration
 */
void superframe_settings_set_defaults(const struct superframe_config_ble *config);

/**
 * @brief Get current superframe configuration
 *
 * @param config Output buffer for configuration
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_get(struct superframe_config_ble *config);

/**
 * @brief Set superframe configuration
 *
 * Updates the configuration and persists to NVS.
 * Does NOT automatically rebuild/restart the superframe.
 * Call superframe_settings_apply() to activate changes.
 *
 * @param config New configuration
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_set(const struct superframe_config_ble *config);

/**
 * @brief Get block count
 *
 * @return Number of blocks in current configuration
 */
uint8_t superframe_settings_get_block_count(void);

/**
 * @brief Set block count
 *
 * @param count New block count (1 to SUPERFRAME_MAX_BLOCKS)
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_set_block_count(uint8_t count);

/**
 * @brief Get configuration for a specific block
 *
 * @param index Block index (0 to block_count-1)
 * @param config Output buffer for block configuration
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_get_block(uint8_t index, struct block_config_ble *config);

/**
 * @brief Set configuration for a specific block
 *
 * @param index Block index (0 to block_count-1)
 * @param config New block configuration
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_set_block(uint8_t index, const struct block_config_ble *config);

/**
 * @brief Validate current configuration
 *
 * @return 0 if valid, negative error code describing issue
 */
int superframe_settings_validate(void);

/**
 * @brief Register callback for block results
 *
 * Applications call this to receive results from specific block types.
 * The callback is invoked when a block of the registered type completes.
 *
 * @param type Block type to register for
 * @param cb Callback function
 * @param user_data User data passed to callback
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_register_callback(block_type_t type,
                                          block_result_cb_t cb,
                                          void *user_data);

/**
 * @brief Build executable superframe from current settings
 *
 * Creates a runtime superframe with callbacks wired up from registered handlers.
 * The returned superframe must be freed with superframe_free() when done.
 *
 * @param sframe Output pointer to allocated superframe
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_build(struct superframe **sframe);

/**
 * @brief Apply settings and restart superframe
 *
 * Stops the current superframe (if running), rebuilds from current settings,
 * and starts execution.
 *
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_apply(void);

/**
 * @brief Save current configuration to NVS
 *
 * @return 0 on success, negative error code on failure
 */
int superframe_settings_save(void);

/**
 * @brief Print current superframe configuration
 */
void superframe_settings_print(void);

#endif /* SYNCHROFLY_MANAGEMENT_SUPERFRAME_SETTINGS_H */
