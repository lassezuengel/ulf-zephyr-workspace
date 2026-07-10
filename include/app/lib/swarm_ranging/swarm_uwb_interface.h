/**
 * @file swarm_uwb_interface.h
 * @brief UWB driver abstraction layer for swarm ranging
 *
 * Provides a simplified interface to zephyr-uwb driver that matches
 * the original SEU-NetSI adhocdeck API.
 */

#ifndef SWARM_UWB_INTERFACE_H
#define SWARM_UWB_INTERFACE_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#include <app/lib/swarm_ranging/swarm_ranging_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * RX event structure
 * Contains received data and timestamp
 */
typedef struct {
    uint8_t *data;
    uint16_t length;
    uint64_t rx_timestamp;  // 40-bit UWB timestamp
} swarm_uwb_rx_event_t;

/**
 * RX callback function type
 * Called when a UWB_RANGING_MESSAGE is received
 */
typedef void (*swarm_uwb_rx_callback_t)(swarm_uwb_rx_event_t *event);

/**
 * TX callback function type
 * Called when a ranging message is successfully transmitted
 */
typedef void (*swarm_uwb_tx_callback_t)(uint64_t tx_timestamp, uint16_t seq_num);

/**
 * Initialize UWB interface for swarm ranging
 *
 * @param dev UWB device pointer
 * @return 0 on success, negative error code on failure
 */
int swarm_uwb_init(const struct device *dev);

/**
 * Get local UWB address
 *
 * @return Local UWB address (16-bit)
 */
uint16_t swarm_uwb_get_address(void);

/**
 * Send UWB packet (blocking)
 *
 * This function wraps the packet with UWB_Packet_Header_t and sends it.
 * Equivalent to uwbSendPacketBlock() in original implementation.
 *
 * @param data Pointer to data to send
 * @param length Length of data in bytes
 * @return 0 on success, negative error code on failure
 */
int swarm_uwb_send_packet_blocking(uint8_t *data, uint16_t length);

/**
 * Register RX callback for ranging messages
 *
 * @param callback Callback function to be called on RX
 */
void swarm_uwb_register_rx_callback(swarm_uwb_rx_callback_t callback);

/**
 * Register TX callback for ranging messages
 *
 * @param callback Callback function to be called on TX complete
 */
void swarm_uwb_register_tx_callback(swarm_uwb_tx_callback_t callback);

/**
 * Read last RX timestamp
 *
 * @return 40-bit UWB timestamp
 */
uint64_t swarm_uwb_read_rx_timestamp(void);

/**
 * Read last TX timestamp
 *
 * @return 40-bit UWB timestamp
 */
uint64_t swarm_uwb_read_tx_timestamp(void);

/**
 * Start continuous RX mode
 *
 * @return 0 on success, negative error code on failure
 */
int swarm_uwb_start_rx(void);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_UWB_INTERFACE_H */
