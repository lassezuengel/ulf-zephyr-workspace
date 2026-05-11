/**
 * @file bt_peripheral.h
 * @brief SynchroFly Bluetooth LE Peripheral API
 *
 * Provides BLE peripheral functionality including:
 * - Automatic advertising with unique device names
 * - Connection management with auto-reconnect
 * - Status LED control
 * - MCUmgr/DFU support when enabled
 */

#ifndef SYNCHROFLY_BT_PERIPHERAL_H
#define SYNCHROFLY_BT_PERIPHERAL_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get current Bluetooth connection state
 *
 * @return true if a client is connected, false otherwise
 */
bool synchrofly_bt_is_connected(void);

/**
 * @brief Get the device's BLE name
 *
 * Returns the full device name including the node ID suffix,
 * e.g., "SynchroFly-D768" (where D768 is the IoT-LAB compatible UID)
 *
 * @return Pointer to device name string (valid for lifetime of program)
 */
const char *synchrofly_bt_get_device_name(void);

/**
 * @brief Get current Bluetooth connection handle
 *
 * @return Pointer to current connection, or NULL if not connected
 */
struct bt_conn *synchrofly_bt_get_conn(void);

#ifdef __cplusplus
}
#endif

#endif /* SYNCHROFLY_BT_PERIPHERAL_H */
