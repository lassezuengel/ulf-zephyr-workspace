/*
 * Copyright (c) 2025 SynchroFly Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device Identification BLE Service
 *
 * Provides a simple write-only characteristic to trigger visual
 * device identification via LED blinking.
 */

#ifndef IDENTIFICATION_SERVICE_H
#define IDENTIFICATION_SERVICE_H

/* Service UUID: "identify" in ASCII hex (6964656e-7469-6679) */
#define IDENTIFICATION_SERVICE_UUID_VAL \
    BT_UUID_128_ENCODE(0x6964656e, 0x7469, 0x6679, 0x0000, 0x000000000000ULL)

/* Trigger characteristic UUID */
#define IDENTIFICATION_TRIGGER_CHAR_UUID_VAL \
    BT_UUID_128_ENCODE(0x6964656e, 0x7469, 0x6679, 0x0000, 0x000000000001ULL)

#endif /* IDENTIFICATION_SERVICE_H */
