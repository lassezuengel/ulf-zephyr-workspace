/**
 * @file bt_services.h
 * @brief GATT Service Registry API
 *
 * Allows applications and libraries to register custom GATT services
 * that will be exposed via the BLE peripheral.
 */

#ifndef SYNCHROFLY_BT_SERVICES_H
#define SYNCHROFLY_BT_SERVICES_H

#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register a custom GATT service
 *
 * Services should be registered during initialization, before
 * Bluetooth advertising starts.
 *
 * @param svc Pointer to GATT service definition (must remain valid)
 * @return 0 on success, negative errno on failure
 */
int synchrofly_bt_register_service(const struct bt_gatt_service *svc);

/**
 * @brief Initialize the service registry
 *
 * Called automatically during Bluetooth peripheral initialization.
 * Applications should not call this directly.
 *
 * @return 0 on success, negative errno on failure
 */
int synchrofly_bt_services_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SYNCHROFLY_BT_SERVICES_H */
