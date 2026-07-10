/**
 * @file network_settings_service.h
 * @brief GATT service for configuring network parameters
 *
 * Provides characteristics for:
 * - Scheduling scheme (Basic/Hashed/Contention)
 * - Ranging round parameters (slots per phase, phases)
 * - Superframe slots
 * - Glossy parameters (guard, max depth, TX delay)
 * - Scheduler slot duration
 */

#ifndef SYNCHROFLY_BT_NETWORK_SETTINGS_SERVICE_H
#define SYNCHROFLY_BT_NETWORK_SETTINGS_SERVICE_H

/**
 * @brief Initialize the network settings GATT service
 *
 * The service is automatically registered via BT_GATT_SERVICE_DEFINE.
 * This function performs any additional initialization if needed.
 *
 * @return 0 on success, negative error code on failure
 */
int network_settings_service_init(void);

#endif /* SYNCHROFLY_BT_NETWORK_SETTINGS_SERVICE_H */
