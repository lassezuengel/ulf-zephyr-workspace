/*
 * Copyright (c) 2025 SynchroFly Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * CRTP Bridge BLE Service
 *
 * Provides a bidirectional BLE interface for CRTP communication with the
 * Crazyflie STM32 via the UWB deck's SPI HCI protocol:
 *
 * - TX characteristic (Write): BLE client sends CRTP packets to Crazyflie
 * - RX characteristic (Notify): BLE client receives CRTP responses from Crazyflie
 *
 * This enables mobile apps to access Crazyflie log/param services through BLE
 * without requiring a Crazyradio dongle.
 */

#ifndef CRTP_SERVICE_H
#define CRTP_SERVICE_H

/*
 * CRTP Service UUID: "crtp" in ASCII hex (63727470)
 * Full UUID: 63727470-0000-0000-0000-000000000000
 */
#define CRTP_SERVICE_UUID_VAL \
    BT_UUID_128_ENCODE(0x63727470, 0x0000, 0x0000, 0x0000, 0x000000000000ULL)

/*
 * CRTP TX Characteristic UUID (Write without response)
 * Used by clients to send CRTP packets to the Crazyflie.
 * Format: [header (1 byte)] [payload (0-30 bytes)]
 * Header: port(4 bits) | channel(2 bits) | reserved(2 bits)
 */
#define CRTP_TX_CHAR_UUID_VAL \
    BT_UUID_128_ENCODE(0x63727470, 0x0000, 0x0000, 0x0000, 0x000000000001ULL)

/*
 * CRTP RX Characteristic UUID (Notify)
 * Used to receive CRTP responses from the Crazyflie.
 * Format: [header (1 byte)] [payload (0-30 bytes)]
 * Header: port(4 bits) | channel(2 bits) | reserved(2 bits)
 *
 * Enable notifications on this characteristic to receive CRTP packets
 * that the Crazyflie sends in response to requests written to TX.
 */
#define CRTP_RX_CHAR_UUID_VAL \
    BT_UUID_128_ENCODE(0x63727470, 0x0000, 0x0000, 0x0000, 0x000000000002ULL)

#endif /* CRTP_SERVICE_H */
