/*
 * Copyright (c) 2025 SynchroFly Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * System Profiling BLE Service
 *
 * Exposes heap and per-thread stack usage statistics over BLE GATT.
 * Requires CONFIG_THREAD_STACK_INFO, CONFIG_SYS_HEAP_RUNTIME_STATS,
 * CONFIG_INIT_STACKS, and CONFIG_THREAD_MONITOR.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <zephyr/sys/sys_heap.h>
#include <string.h>

#include "system_profiling_service.h"
#include <app/lib/system/block_heap.h>

LOG_MODULE_REGISTER(sys_profiling_svc, CONFIG_LOG_DEFAULT_LEVEL);

/* The kernel system heap created by CONFIG_HEAP_MEM_POOL_SIZE */
extern struct k_heap _system_heap;

/* Max threads we can report (24 bytes each, MTU=247 -> ~10 entries) */
#define MAX_THREAD_ENTRIES 10

#define THREAD_NAME_LEN 16

struct __packed thread_stats_entry {
    char     name[THREAD_NAME_LEN];
    uint32_t stack_size;
    uint32_t stack_used;
};

/* Scratch buffer filled on each read */
static struct thread_stats_entry thread_buf[MAX_THREAD_ENTRIES];
static uint8_t thread_count;

/* ---- Heap stats characteristic ----------------------------------------- */

static ssize_t read_heap_stats(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    struct sys_memory_stats stats;
    int err = sys_heap_runtime_stats_get(&_system_heap.heap, &stats);

    if (err) {
        LOG_ERR("Failed to get heap stats: %d", err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct __packed {
        uint32_t free_bytes;
        uint32_t allocated_bytes;
        uint32_t max_allocated_bytes;
    } response = {
        .free_bytes = (uint32_t)stats.free_bytes,
        .allocated_bytes = (uint32_t)stats.allocated_bytes,
        .max_allocated_bytes = (uint32_t)stats.max_allocated_bytes,
    };

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &response, sizeof(response));
}

/* ---- Block heap stats characteristic ----------------------------------- */

static ssize_t read_block_heap_stats(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset)
{
    struct k_heap *bh = block_heap_get();
    struct sys_memory_stats stats;
    int err = sys_heap_runtime_stats_get(&bh->heap, &stats);

    if (err) {
        LOG_ERR("Failed to get block heap stats: %d", err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct __packed {
        uint32_t free_bytes;
        uint32_t allocated_bytes;
        uint32_t max_allocated_bytes;
    } response = {
        .free_bytes = (uint32_t)stats.free_bytes,
        .allocated_bytes = (uint32_t)stats.allocated_bytes,
        .max_allocated_bytes = (uint32_t)stats.max_allocated_bytes,
    };

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &response, sizeof(response));
}

/* ---- Thread enumeration ------------------------------------------------ */

static void thread_info_cb(const struct k_thread *thread, void *user_data)
{
    if (thread_count >= MAX_THREAD_ENTRIES) {
        return;
    }

    struct thread_stats_entry *entry = &thread_buf[thread_count];
    memset(entry, 0, sizeof(*entry));

    const char *name = k_thread_name_get((k_tid_t)thread);
    if (name) {
        strncpy(entry->name, name, THREAD_NAME_LEN - 1);
    } else {
        snprintf(entry->name, THREAD_NAME_LEN, "0x%08x",
                 (unsigned int)(uintptr_t)thread);
    }

    size_t unused = 0;
    int err = k_thread_stack_space_get((struct k_thread *)thread, &unused);

    if (err == 0) {
        entry->stack_size = (uint32_t)thread->stack_info.size;
        entry->stack_used = entry->stack_size - (uint32_t)unused;
    } else {
        entry->stack_size = (uint32_t)thread->stack_info.size;
        entry->stack_used = 0;
    }

    thread_count++;
}

/* ---- Thread count characteristic --------------------------------------- */

static ssize_t read_thread_count(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    thread_count = 0;
    k_thread_foreach(thread_info_cb, NULL);

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &thread_count, sizeof(thread_count));
}

/* ---- Thread stats characteristic --------------------------------------- */

static ssize_t read_thread_stats(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    thread_count = 0;
    k_thread_foreach(thread_info_cb, NULL);

    size_t data_len = thread_count * sizeof(struct thread_stats_entry);

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             thread_buf, data_len);
}

/* ---- Service definition ------------------------------------------------ */

static struct bt_uuid_128 svc_uuid = BT_UUID_INIT_128(
    SYSTEM_PROFILING_SERVICE_UUID_VAL);

static struct bt_uuid_128 heap_stats_uuid = BT_UUID_INIT_128(
    SYSTEM_PROFILING_HEAP_STATS_UUID_VAL);

static struct bt_uuid_128 thread_count_uuid = BT_UUID_INIT_128(
    SYSTEM_PROFILING_THREAD_COUNT_UUID_VAL);

static struct bt_uuid_128 thread_stats_uuid = BT_UUID_INIT_128(
    SYSTEM_PROFILING_THREAD_STATS_UUID_VAL);

static struct bt_uuid_128 block_heap_stats_uuid = BT_UUID_INIT_128(
    SYSTEM_PROFILING_BLOCK_HEAP_STATS_UUID_VAL);

BT_GATT_SERVICE_DEFINE(system_profiling_svc,
    BT_GATT_PRIMARY_SERVICE(&svc_uuid),

    /* Heap Stats (read-only, 12 bytes) */
    BT_GATT_CHARACTERISTIC(&heap_stats_uuid.uuid,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_heap_stats, NULL, NULL),
    BT_GATT_CUD("Heap Stats", BT_GATT_PERM_READ),

    /* Thread Count (read-only, 1 byte) */
    BT_GATT_CHARACTERISTIC(&thread_count_uuid.uuid,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_thread_count, NULL, NULL),
    BT_GATT_CUD("Thread Count", BT_GATT_PERM_READ),

    /* Thread Stats (read-only, variable length) */
    BT_GATT_CHARACTERISTIC(&thread_stats_uuid.uuid,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_thread_stats, NULL, NULL),
    BT_GATT_CUD("Thread Stats", BT_GATT_PERM_READ),

    /* Block Heap Stats (read-only, 12 bytes) */
    BT_GATT_CHARACTERISTIC(&block_heap_stats_uuid.uuid,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_block_heap_stats, NULL, NULL),
    BT_GATT_CUD("Block Heap Stats", BT_GATT_PERM_READ),
);
