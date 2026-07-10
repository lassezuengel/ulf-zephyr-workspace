/*
 * Copyright (c) 2025 SynchroFly Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * System Control BLE Service
 *
 * Provides write-only characteristics for system-level control:
 * - Soft reboot (with delay for BLE response)
 * - Sync reset (clear time sync state + restart superframe)
 *
 * Both commands share a single deferred work item to save RAM on
 * constrained targets (nRF52832 has 64KB).
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

#include <app/lib/timesync/time_synchronization.h>
#include <app/lib/scheduling/upper/block_scheduler.h>
#if defined(CONFIG_SYNCHROFLY_SWARM_RANGING)
#include <app/lib/swarm_ranging/swarm_ranging_core.h>
#endif

#include "system_control_service.h"

LOG_MODULE_REGISTER(system_control_service, CONFIG_LOG_DEFAULT_LEVEL);

/* Service UUID */
static struct bt_uuid_128 system_control_service_uuid = BT_UUID_INIT_128(
    SYSTEM_CONTROL_SERVICE_UUID_VAL);

/* Characteristic UUIDs */
static struct bt_uuid_128 reset_trigger_uuid = BT_UUID_INIT_128(
    SYSTEM_CONTROL_RESET_CHAR_UUID_VAL);

static struct bt_uuid_128 sync_reset_trigger_uuid = BT_UUID_INIT_128(
    SYSTEM_CONTROL_SYNC_RESET_CHAR_UUID_VAL);

/* Delay before actions to let BLE stack send the write response */
#define CONTROL_DELAY_MS 200

/* Command codes for the shared work item */
enum sysctrl_cmd {
    SYSCTRL_CMD_NONE = 0,
    SYSCTRL_CMD_REBOOT,
    SYSCTRL_CMD_SYNC_RESET,
};

static volatile enum sysctrl_cmd pending_cmd;

static void control_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(control_work, control_work_handler);

static void control_work_handler(struct k_work *work)
{
    enum sysctrl_cmd cmd = pending_cmd;
    pending_cmd = SYSCTRL_CMD_NONE;

    switch (cmd) {
    case SYSCTRL_CMD_REBOOT:
        LOG_WRN("Rebooting now...");
        sys_reboot(SYS_REBOOT_COLD);
        break;
    case SYSCTRL_CMD_SYNC_RESET:
        LOG_INF("Performing sync reset...");
        time_sync_scheduler_pause();
        time_sync_reset_sync_state();
#if defined(CONFIG_SYNCHROFLY_SWARM_RANGING)
        swarm_ranging_reset();
#endif
        block_scheduler_rebuild_superframe();
        LOG_INF("Sync reset complete, scheduler resumed");
        break;
    default:
        break;
    }
}

static ssize_t write_reset_trigger(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len,
                                   uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    LOG_WRN("System reset requested via BLE");
    pending_cmd = SYSCTRL_CMD_REBOOT;
    k_work_schedule(&control_work, K_MSEC(CONTROL_DELAY_MS));

    return len;
}

static ssize_t write_sync_reset_trigger(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        const void *buf, uint16_t len,
                                        uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    LOG_INF("Sync reset requested via BLE");
    pending_cmd = SYSCTRL_CMD_SYNC_RESET;
    k_work_schedule(&control_work, K_MSEC(CONTROL_DELAY_MS));

    return len;
}

/* Service Definition */
BT_GATT_SERVICE_DEFINE(system_control_svc,
    BT_GATT_PRIMARY_SERVICE(&system_control_service_uuid),

    /* Reset Trigger Characteristic - write only */
    BT_GATT_CHARACTERISTIC(&reset_trigger_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, write_reset_trigger, NULL),

    /* Sync Reset Characteristic - write only */
    BT_GATT_CHARACTERISTIC(&sync_reset_trigger_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, write_sync_reset_trigger, NULL),
);
