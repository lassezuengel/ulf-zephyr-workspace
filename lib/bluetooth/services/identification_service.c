/*
 * Copyright (c) 2025 SynchroFly Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device Identification BLE Service
 *
 * Provides a simple write-only characteristic to trigger visual
 * device identification via LED blinking.
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "identification_service.h"

LOG_MODULE_REGISTER(identification_service, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Weak implementation of synchrofly_status_identify for when
 * the status LED module is not enabled. The real implementation
 * in status_led.c will override this when available.
 */
__weak void synchrofly_status_identify(uint32_t duration_ms)
{
    LOG_WRN("Status LED not available, identification not supported");
}

/* Service UUID */
static struct bt_uuid_128 identification_service_uuid = BT_UUID_INIT_128(
    IDENTIFICATION_SERVICE_UUID_VAL);

/* Trigger characteristic UUID */
static struct bt_uuid_128 trigger_uuid = BT_UUID_INIT_128(
    IDENTIFICATION_TRIGGER_CHAR_UUID_VAL);

/* Identification duration in milliseconds */
#define IDENTIFICATION_DURATION_MS 2000

/**
 * Write handler for identification trigger characteristic.
 * Any write triggers a 2-second LED blink pattern.
 */
static ssize_t write_identify_trigger(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len,
                                      uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    /* Trigger identification regardless of write value */
    synchrofly_status_identify(IDENTIFICATION_DURATION_MS);

    LOG_INF("Device identification triggered via BLE");

    return len;
}

/* Service Definition */
BT_GATT_SERVICE_DEFINE(identification_svc,
    BT_GATT_PRIMARY_SERVICE(&identification_service_uuid),

    /* Trigger Characteristic - write only */
    BT_GATT_CHARACTERISTIC(&trigger_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, write_identify_trigger, NULL),
);
