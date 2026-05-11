/*
 * Copyright (c) 2025 SynchroFly Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * System Control BLE Service
 *
 * Provides a write-only characteristic to trigger a system reset
 * via BLE. Writing any value initiates a soft reboot after a short
 * delay to allow the BLE stack to send the write response.
 */

#ifndef SYSTEM_CONTROL_SERVICE_H
#define SYSTEM_CONTROL_SERVICE_H

/* Service UUID: "syscontrl" in ASCII hex (73797363-6f6e-7472-6c00) */
#define SYSTEM_CONTROL_SERVICE_UUID_VAL \
    BT_UUID_128_ENCODE(0x73797363, 0x6f6e, 0x7472, 0x6c00, 0x000000000000ULL)

/* Reset trigger characteristic UUID */
#define SYSTEM_CONTROL_RESET_CHAR_UUID_VAL \
    BT_UUID_128_ENCODE(0x73797363, 0x6f6e, 0x7472, 0x6c00, 0x000000000001ULL)

/* Sync reset characteristic UUID */
#define SYSTEM_CONTROL_SYNC_RESET_CHAR_UUID_VAL \
    BT_UUID_128_ENCODE(0x73797363, 0x6f6e, 0x7472, 0x6c00, 0x000000000002ULL)

#endif /* SYSTEM_CONTROL_SERVICE_H */
