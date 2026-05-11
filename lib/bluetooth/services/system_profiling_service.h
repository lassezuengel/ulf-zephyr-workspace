/*
 * Copyright (c) 2025 SynchroFly Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * System Profiling BLE Service
 *
 * Exposes heap and per-thread stack usage statistics over BLE GATT
 * for runtime debugging without RTT/JTAG access.
 */

#ifndef SYSTEM_PROFILING_SERVICE_H
#define SYSTEM_PROFILING_SERVICE_H

/* Service UUID: "sysprofile" -> 73797370-726f-6669-6c65 */
#define SYSTEM_PROFILING_SERVICE_UUID_VAL \
    BT_UUID_128_ENCODE(0x73797370, 0x726f, 0x6669, 0x6c65, 0x000000000000ULL)

/* Heap Stats characteristic UUID: ...0001 */
#define SYSTEM_PROFILING_HEAP_STATS_UUID_VAL \
    BT_UUID_128_ENCODE(0x73797370, 0x726f, 0x6669, 0x6c65, 0x000000000001ULL)

/* Thread Count characteristic UUID: ...0002 */
#define SYSTEM_PROFILING_THREAD_COUNT_UUID_VAL \
    BT_UUID_128_ENCODE(0x73797370, 0x726f, 0x6669, 0x6c65, 0x000000000002ULL)

/* Thread Stats characteristic UUID: ...0003 */
#define SYSTEM_PROFILING_THREAD_STATS_UUID_VAL \
    BT_UUID_128_ENCODE(0x73797370, 0x726f, 0x6669, 0x6c65, 0x000000000003ULL)

/* Block Heap Stats characteristic UUID: ...0004 */
#define SYSTEM_PROFILING_BLOCK_HEAP_STATS_UUID_VAL \
    BT_UUID_128_ENCODE(0x73797370, 0x726f, 0x6669, 0x6c65, 0x000000000004ULL)

#endif /* SYSTEM_PROFILING_SERVICE_H */
