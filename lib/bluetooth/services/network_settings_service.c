/**
 * @file network_settings_service.c
 * @brief GATT service for configuring network parameters
 *
 * Provides characteristics for:
 * - Scheduling scheme (Basic/Hashed/Contention)
 * - Ranging round parameters (slots per phase, phases)
 * - Superframe slots
 * - Glossy parameters (guard, max depth, TX delay)
 * - Scheduler slot duration
 * - Distance filter settings (strategy, window size, poly order)
 * - Distance series capacity
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <app/lib/management/network_settings.h>
#include <app/lib/blocks/mtm.h>
#include <app/lib/node_table/node_table.h>
#include "network_settings_service.h"

LOG_MODULE_REGISTER(network_settings_svc, CONFIG_LOG_DEFAULT_LEVEL);

/* Custom UUID for network settings service */
/* Base UUID: 6e657473-6574-7469-6e67-000000000000 (ASCII "netsetting") */
#define BT_UUID_NETWORK_SETTINGS_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000000ULL)

#define BT_UUID_NETWORK_SETTINGS_SERVICE \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_SETTINGS_SERVICE_VAL)

/* Scheduling scheme characteristic UUID: ...0001 */
#define BT_UUID_NETWORK_SCHEDULING_SCHEME_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000001ULL)

#define BT_UUID_NETWORK_SCHEDULING_SCHEME \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_SCHEDULING_SCHEME_VAL)

/* Slots per phase characteristic UUID: ...0002 */
#define BT_UUID_NETWORK_SLOTS_PER_PHASE_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000002ULL)

#define BT_UUID_NETWORK_SLOTS_PER_PHASE \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_SLOTS_PER_PHASE_VAL)

/* Ranging phases characteristic UUID: ...0003 */
#define BT_UUID_NETWORK_RANGING_PHASES_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000003ULL)

#define BT_UUID_NETWORK_RANGING_PHASES \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_RANGING_PHASES_VAL)

/* Superframe slots characteristic UUID: ...0004 */
#define BT_UUID_NETWORK_SUPERFRAME_SLOTS_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000004ULL)

#define BT_UUID_NETWORK_SUPERFRAME_SLOTS \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_SUPERFRAME_SLOTS_VAL)

/* Glossy guard (us) characteristic UUID: ...0005 */
#define BT_UUID_NETWORK_GLOSSY_GUARD_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000005ULL)

#define BT_UUID_NETWORK_GLOSSY_GUARD \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_GLOSSY_GUARD_VAL)

/* Glossy max depth characteristic UUID: ...0006 */
#define BT_UUID_NETWORK_GLOSSY_MAX_DEPTH_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000006ULL)

#define BT_UUID_NETWORK_GLOSSY_MAX_DEPTH \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_GLOSSY_MAX_DEPTH_VAL)

/* Glossy TX delay (us) characteristic UUID: ...0007 */
#define BT_UUID_NETWORK_GLOSSY_TX_DELAY_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000007ULL)

#define BT_UUID_NETWORK_GLOSSY_TX_DELAY \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_GLOSSY_TX_DELAY_VAL)

/* Slot duration (ms) characteristic UUID: ...0008 */
#define BT_UUID_NETWORK_SLOT_DURATION_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000008ULL)

#define BT_UUID_NETWORK_SLOT_DURATION \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_SLOT_DURATION_VAL)

/* Slot padding (us) characteristic UUID: ...0009 */
#define BT_UUID_NETWORK_SLOT_PADDING_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000009ULL)

#define BT_UUID_NETWORK_SLOT_PADDING \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_SLOT_PADDING_VAL)

/* Distance filter enabled characteristic UUID: ...000A */
#define BT_UUID_NETWORK_DISTANCE_FILTER_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x00000000000AULL)

#define BT_UUID_NETWORK_DISTANCE_FILTER \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_DISTANCE_FILTER_VAL)

/* Filter strategy characteristic UUID: ...000B */
#define BT_UUID_NETWORK_FILTER_STRATEGY_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x00000000000BULL)

#define BT_UUID_NETWORK_FILTER_STRATEGY \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_FILTER_STRATEGY_VAL)

/* Filter window size characteristic UUID: ...000C */
#define BT_UUID_NETWORK_FILTER_WINDOW_SIZE_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x00000000000CULL)

#define BT_UUID_NETWORK_FILTER_WINDOW_SIZE \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_FILTER_WINDOW_SIZE_VAL)

/* Filter poly order characteristic UUID: ...000D */
#define BT_UUID_NETWORK_FILTER_POLY_ORDER_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x00000000000DULL)

#define BT_UUID_NETWORK_FILTER_POLY_ORDER \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_FILTER_POLY_ORDER_VAL)

/* Series capacity characteristic UUID: ...000E */
#define BT_UUID_NETWORK_SERIES_CAPACITY_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x00000000000EULL)

#define BT_UUID_NETWORK_SERIES_CAPACITY \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_SERIES_CAPACITY_VAL)

/* Outlier filter enabled characteristic UUID: ...0012 */
#define BT_UUID_NETWORK_OUTLIER_FILTER_ENABLED_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000012ULL)

#define BT_UUID_NETWORK_OUTLIER_FILTER_ENABLED \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_OUTLIER_FILTER_ENABLED_VAL)

/* Outlier filter threshold characteristic UUID: ...0013 */
#define BT_UUID_NETWORK_OUTLIER_FILTER_THRESHOLD_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000013ULL)

#define BT_UUID_NETWORK_OUTLIER_FILTER_THRESHOLD \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_OUTLIER_FILTER_THRESHOLD_VAL)

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)

/* MM filter window size characteristic UUID: ...000F */
#define BT_UUID_NETWORK_MM_FILTER_WINDOW_SIZE_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x00000000000FULL)

#define BT_UUID_NETWORK_MM_FILTER_WINDOW_SIZE \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_MM_FILTER_WINDOW_SIZE_VAL)

/* MM filter poly order characteristic UUID: ...0010 */
#define BT_UUID_NETWORK_MM_FILTER_POLY_ORDER_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000010ULL)

#define BT_UUID_NETWORK_MM_FILTER_POLY_ORDER \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_MM_FILTER_POLY_ORDER_VAL)

/* MM series capacity characteristic UUID: ...0011 */
#define BT_UUID_NETWORK_MM_SERIES_CAPACITY_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000011ULL)

#define BT_UUID_NETWORK_MM_SERIES_CAPACITY \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_MM_SERIES_CAPACITY_VAL)

/* MM phase filter window size characteristic UUID: ...0014 */
#define BT_UUID_NETWORK_MM_PHASE_FILTER_WINDOW_SIZE_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000014ULL)

#define BT_UUID_NETWORK_MM_PHASE_FILTER_WINDOW_SIZE \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_MM_PHASE_FILTER_WINDOW_SIZE_VAL)

/* MM phase filter poly order characteristic UUID: ...0015 */
#define BT_UUID_NETWORK_MM_PHASE_FILTER_POLY_ORDER_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000015ULL)

#define BT_UUID_NETWORK_MM_PHASE_FILTER_POLY_ORDER \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_MM_PHASE_FILTER_POLY_ORDER_VAL)

/* MM TWR filter window size characteristic UUID: ...0016 */
#define BT_UUID_NETWORK_MM_TWR_FILTER_WINDOW_SIZE_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000016ULL)

#define BT_UUID_NETWORK_MM_TWR_FILTER_WINDOW_SIZE \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_MM_TWR_FILTER_WINDOW_SIZE_VAL)

/* MM TWR filter poly order characteristic UUID: ...0017 */
#define BT_UUID_NETWORK_MM_TWR_FILTER_POLY_ORDER_VAL \
    BT_UUID_128_ENCODE(0x6e657473, 0x6574, 0x7469, 0x6e67, 0x000000000017ULL)

#define BT_UUID_NETWORK_MM_TWR_FILTER_POLY_ORDER \
    BT_UUID_DECLARE_128(BT_UUID_NETWORK_MM_TWR_FILTER_POLY_ORDER_VAL)

#endif /* CONFIG_NODE_TABLE_MM_ENABLED */

/* ========================================================================== */
/* Scheduling Scheme (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_scheduling_scheme(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset)
{
    schedule_type_t scheme = network_get_schedule_type();
    uint8_t val = (uint8_t)scheme;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_scheduling_scheme(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len, uint16_t offset,
                                       uint8_t flags)
{
    uint8_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val > SCHEDULE_CONTENTION) {
        LOG_ERR("Invalid scheduling scheme: %u", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    network_set_schedule_type((schedule_type_t)val);
    LOG_INF("Scheduling scheme updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* Slots Per Phase (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_slots_per_phase(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = network_get_ranging_round_slots_per_phase();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_slots_per_phase(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset,
                                     uint8_t flags)
{
    uint8_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val == 0) {
        LOG_ERR("Invalid slots per phase: %u (must be > 0)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    network_set_ranging_round_slots_per_phase(val);
    LOG_INF("Slots per phase updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* Ranging Phases (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_ranging_phases(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = network_get_ranging_round_phases();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_ranging_phases(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset,
                                    uint8_t flags)
{
    uint8_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val == 0) {
        LOG_ERR("Invalid ranging phases: %u (must be > 0)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    network_set_ranging_round_phases(val);
    LOG_INF("Ranging phases updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* Superframe Slots (uint16_t, 2 bytes) */
/* ========================================================================== */

static ssize_t read_superframe_slots(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset)
{
    uint16_t val = network_get_superframe_slots();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_superframe_slots(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags)
{
    uint16_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val == 0) {
        LOG_ERR("Invalid superframe slots: %u (must be > 0)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    network_set_superframe_slots(val);
    LOG_INF("Superframe slots updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* Glossy Guard (uint16_t, 2 bytes, microseconds) */
/* ========================================================================== */

static ssize_t read_glossy_guard(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    uint16_t val = network_get_glossy_guard_us();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_glossy_guard(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len, uint16_t offset,
                                  uint8_t flags)
{
    uint16_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    network_set_glossy_guard_us(val);
    LOG_INF("Glossy guard updated via BLE: %u us", val);

    return len;
}

/* ========================================================================== */
/* Glossy Max Depth (uint16_t, 2 bytes, hops) */
/* ========================================================================== */

static ssize_t read_glossy_max_depth(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset)
{
    uint16_t val = network_get_glossy_max_depth();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_glossy_max_depth(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags)
{
    uint16_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val == 0) {
        LOG_ERR("Invalid glossy max depth: %u (must be > 0)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    network_set_glossy_max_depth(val);
    LOG_INF("Glossy max depth updated via BLE: %u hops", val);

    return len;
}

/* ========================================================================== */
/* Glossy TX Delay (uint16_t, 2 bytes, microseconds) */
/* ========================================================================== */

static ssize_t read_glossy_tx_delay(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset)
{
    uint16_t val = network_get_glossy_transmission_delay_us();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_glossy_tx_delay(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset,
                                     uint8_t flags)
{
    uint16_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    network_set_glossy_transmission_delay_us(val);
    LOG_INF("Glossy TX delay updated via BLE: %u us", val);

    return len;
}

/* ========================================================================== */
/* Slot Duration (uint32_t, 4 bytes, milliseconds) */
/* ========================================================================== */

static ssize_t read_slot_duration(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
    uint32_t val = network_get_scheduler_slot_duration_ms();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_slot_duration(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len, uint16_t offset,
                                   uint8_t flags)
{
    uint32_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val == 0) {
        LOG_ERR("Invalid slot duration: %u (must be > 0)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    network_set_scheduler_slot_duration_ms(val);
    LOG_INF("Slot duration updated via BLE: %u ms", val);

    return len;
}

/* ========================================================================== */
/* Slot Padding (uint16_t, 2 bytes, microseconds) */
/* ========================================================================== */

static ssize_t read_slot_padding(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    uint16_t val = network_get_slot_padding_us();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_slot_padding(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len, uint16_t offset,
                                  uint8_t flags)
{
    uint16_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    network_set_slot_padding_us(val);
    LOG_INF("Slot padding updated via BLE: %u us", val);

    return len;
}

/* ========================================================================== */
/* Distance Filter Enabled (uint8_t, 1 byte, boolean) */
/* ========================================================================== */

static ssize_t read_distance_filter(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_distance_filter_enabled() ? 1 : 0;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_distance_filter(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset,
                                     uint8_t flags)
{
    uint8_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    node_table_set_distance_filter_enabled(val != 0);
    LOG_INF("Distance filter %s via BLE", val ? "enabled" : "disabled");

    return len;
}

#if defined(CONFIG_SYNCHROFLY_BT_FILTER_CONFIG_CHARS)

/* ========================================================================== */
/* Filter Strategy (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_filter_strategy(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = (uint8_t)node_table_get_distance_filter_strategy();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_filter_strategy(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset,
                                     uint8_t flags)
{
    uint8_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val > DISTANCE_FILTER_SAVITZKY_GOLAY) {
        LOG_ERR("Invalid filter strategy: %u", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_distance_filter_strategy((enum distance_filter_strategy)val);
    LOG_INF("Filter strategy updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* Filter Window Size (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_filter_window_size(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = (uint8_t)node_table_get_distance_filter_window_size();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_filter_window_size(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        const void *buf, uint16_t len, uint16_t offset,
                                        uint8_t flags)
{
    uint8_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val < 3 || val > 33) {
        LOG_ERR("Invalid filter window size: %u (must be 3-33)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_distance_filter_window_size(val);
    LOG_INF("Filter window size updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* Filter Poly Order (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_filter_poly_order(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_distance_filter_poly_order();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_filter_poly_order(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len, uint16_t offset,
                                       uint8_t flags)
{
    uint8_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val < 1 || val > 5) {
        LOG_ERR("Invalid filter poly order: %u (must be 1-5)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_distance_filter_poly_order(val);
    LOG_INF("Filter poly order updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* Series Capacity (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_series_capacity(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = (uint8_t)node_table_get_distance_series_capacity();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_series_capacity(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset,
                                     uint8_t flags)
{
    uint8_t val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val < 1 || val > 64) {
        LOG_ERR("Invalid series capacity: %u (must be 1-64)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_distance_series_capacity(val);
    LOG_INF("Series capacity updated via BLE: %u", val);

    return len;
}

#endif /* CONFIG_SYNCHROFLY_BT_FILTER_CONFIG_CHARS */

/* ========================================================================== */
/* Outlier Filter Enabled (uint8_t, 1 byte, boolean) */
/* ========================================================================== */

static ssize_t read_outlier_filter_enabled(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_outlier_filter_enabled() ? 1 : 0;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_outlier_filter_enabled(struct bt_conn *conn,
                                            const struct bt_gatt_attr *attr,
                                            const void *buf, uint16_t len, uint16_t offset,
                                            uint8_t flags)
{
    uint8_t val;

    if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    if (len != sizeof(val)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    memcpy(&val, buf, sizeof(val));
    node_table_set_outlier_filter_enabled(val != 0);
    LOG_INF("Outlier filter %s via BLE", val ? "enabled" : "disabled");

    return len;
}

/* ========================================================================== */
/* Outlier Filter Threshold (uint8_t, 1 byte, x10 MAD multiplier) */
/* ========================================================================== */

static ssize_t read_outlier_filter_threshold(struct bt_conn *conn,
                                             const struct bt_gatt_attr *attr,
                                             void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_outlier_filter_threshold();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_outlier_filter_threshold(struct bt_conn *conn,
                                              const struct bt_gatt_attr *attr,
                                              const void *buf, uint16_t len, uint16_t offset,
                                              uint8_t flags)
{
    uint8_t val;

    if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    if (len != sizeof(val)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    memcpy(&val, buf, sizeof(val));

    if (val < 10 || val > 100) {
        LOG_ERR("Invalid outlier threshold: %u (must be 10-100)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_outlier_filter_threshold(val);
    LOG_INF("Outlier threshold updated via BLE: %u (%.1fx MAD)",
            val, (double)((float)val / 10.0f));

    return len;
}

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)

/* ========================================================================== */
/* MM Filter Window Size (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_mm_filter_window_size(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_mm_filter_window_size();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_mm_filter_window_size(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           const void *buf, uint16_t len, uint16_t offset,
                                           uint8_t flags)
{
    uint8_t val;

    if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    if (len != sizeof(val)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    memcpy(&val, buf, sizeof(val));

    if (val < 1 || val > 33) {
        LOG_ERR("Invalid MM filter window size: %u (must be 1-33)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_mm_filter_window_size(val);
    LOG_INF("MM filter window size updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* MM Filter Poly Order (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_mm_filter_poly_order(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_mm_filter_poly_order();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_mm_filter_poly_order(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len, uint16_t offset,
                                          uint8_t flags)
{
    uint8_t val;

    if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    if (len != sizeof(val)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    memcpy(&val, buf, sizeof(val));

    if (val > 6) {
        LOG_ERR("Invalid MM filter poly order: %u (must be 0-6)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_mm_filter_poly_order(val);
    LOG_INF("MM filter poly order updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* MM Series Capacity (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_mm_series_capacity(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_mm_series_capacity();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_mm_series_capacity(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        const void *buf, uint16_t len, uint16_t offset,
                                        uint8_t flags)
{
    uint8_t val;

    if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    if (len != sizeof(val)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    memcpy(&val, buf, sizeof(val));

    if (val < 5 || val > 64) {
        LOG_ERR("Invalid MM series capacity: %u (must be 5-64)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_mm_series_capacity(val);
    LOG_INF("MM series capacity updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* MM Phase Filter Window Size (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_mm_phase_filter_window_size(struct bt_conn *conn,
                                                const struct bt_gatt_attr *attr,
                                                void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_mm_phase_filter_window_size();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_mm_phase_filter_window_size(struct bt_conn *conn,
                                                 const struct bt_gatt_attr *attr,
                                                 const void *buf, uint16_t len, uint16_t offset,
                                                 uint8_t flags)
{
    uint8_t val;

    if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    if (len != sizeof(val)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    memcpy(&val, buf, sizeof(val));

    if (val < 1 || val > 13) {
        LOG_ERR("Invalid MM phase filter window size: %u (must be 1-13)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_mm_phase_filter_window_size(val);
    LOG_INF("MM phase filter window size updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* MM Phase Filter Poly Order (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_mm_phase_filter_poly_order(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_mm_phase_filter_poly_order();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_mm_phase_filter_poly_order(struct bt_conn *conn,
                                                const struct bt_gatt_attr *attr,
                                                const void *buf, uint16_t len, uint16_t offset,
                                                uint8_t flags)
{
    uint8_t val;

    if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    if (len != sizeof(val)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    memcpy(&val, buf, sizeof(val));

    if (val > 6) {
        LOG_ERR("Invalid MM phase filter poly order: %u (must be 0-6)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_mm_phase_filter_poly_order(val);
    LOG_INF("MM phase filter poly order updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* MM TWR Filter Window Size (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_mm_twr_filter_window_size(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_mm_twr_filter_window_size();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_mm_twr_filter_window_size(struct bt_conn *conn,
                                                const struct bt_gatt_attr *attr,
                                                const void *buf, uint16_t len, uint16_t offset,
                                                uint8_t flags)
{
    uint8_t val;

    if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    if (len != sizeof(val)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    memcpy(&val, buf, sizeof(val));

    if (val < 1 || val > 33) {
        LOG_ERR("Invalid MM TWR filter window size: %u (must be 1-33)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_mm_twr_filter_window_size(val);
    LOG_INF("MM TWR filter window size updated via BLE: %u", val);

    return len;
}

/* ========================================================================== */
/* MM TWR Filter Poly Order (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_mm_twr_filter_poly_order(struct bt_conn *conn,
                                              const struct bt_gatt_attr *attr,
                                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = node_table_get_mm_twr_filter_poly_order();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_mm_twr_filter_poly_order(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               const void *buf, uint16_t len, uint16_t offset,
                                               uint8_t flags)
{
    uint8_t val;

    if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    if (len != sizeof(val)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    memcpy(&val, buf, sizeof(val));

    if (val > 6) {
        LOG_ERR("Invalid MM TWR filter poly order: %u (must be 0-6)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    node_table_set_mm_twr_filter_poly_order(val);
    LOG_INF("MM TWR filter poly order updated via BLE: %u", val);

    return len;
}

#endif /* CONFIG_NODE_TABLE_MM_ENABLED */

/* ========================================================================== */
/* GATT Service Definition */
/* ========================================================================== */

BT_GATT_SERVICE_DEFINE(network_settings_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_NETWORK_SETTINGS_SERVICE),

    /* Scheduling scheme characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_SCHEDULING_SCHEME,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_scheduling_scheme, write_scheduling_scheme, NULL),
    BT_GATT_CUD("Schedule", BT_GATT_PERM_READ),

    /* Slots per phase characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_SLOTS_PER_PHASE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_slots_per_phase, write_slots_per_phase, NULL),
    BT_GATT_CUD("SlotsPhase", BT_GATT_PERM_READ),

    /* Ranging phases characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_RANGING_PHASES,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_ranging_phases, write_ranging_phases, NULL),
    BT_GATT_CUD("Phases", BT_GATT_PERM_READ),

    /* Superframe slots characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_SUPERFRAME_SLOTS,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_superframe_slots, write_superframe_slots, NULL),
    BT_GATT_CUD("Superframe", BT_GATT_PERM_READ),

    /* Glossy guard characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_GLOSSY_GUARD,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_glossy_guard, write_glossy_guard, NULL),
    BT_GATT_CUD("GlossyGuard", BT_GATT_PERM_READ),

    /* Glossy max depth characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_GLOSSY_MAX_DEPTH,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_glossy_max_depth, write_glossy_max_depth, NULL),
    BT_GATT_CUD("GlossyDepth", BT_GATT_PERM_READ),

    /* Glossy TX delay characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_GLOSSY_TX_DELAY,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_glossy_tx_delay, write_glossy_tx_delay, NULL),
    BT_GATT_CUD("GlossyDelay", BT_GATT_PERM_READ),

    /* Slot duration characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_SLOT_DURATION,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_slot_duration, write_slot_duration, NULL),
    BT_GATT_CUD("SlotDuration", BT_GATT_PERM_READ),

    /* Slot padding characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_SLOT_PADDING,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_slot_padding, write_slot_padding, NULL),
    BT_GATT_CUD("SlotPadding", BT_GATT_PERM_READ),

    /* Distance filter enabled characteristic (read/write) - legacy */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_DISTANCE_FILTER,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_distance_filter, write_distance_filter, NULL),
    BT_GATT_CUD("DistFilter", BT_GATT_PERM_READ),

#if defined(CONFIG_SYNCHROFLY_BT_FILTER_CONFIG_CHARS)
    /* Filter strategy characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_FILTER_STRATEGY,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_filter_strategy, write_filter_strategy, NULL),

    /* Filter window size characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_FILTER_WINDOW_SIZE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_filter_window_size, write_filter_window_size, NULL),

    /* Filter poly order characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_FILTER_POLY_ORDER,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_filter_poly_order, write_filter_poly_order, NULL),

    /* Series capacity characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_SERIES_CAPACITY,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_series_capacity, write_series_capacity, NULL),
#endif /* CONFIG_SYNCHROFLY_BT_FILTER_CONFIG_CHARS */

    /* Outlier filter enabled characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_OUTLIER_FILTER_ENABLED,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_outlier_filter_enabled, write_outlier_filter_enabled, NULL),

    /* Outlier filter threshold characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_OUTLIER_FILTER_THRESHOLD,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_outlier_filter_threshold, write_outlier_filter_threshold, NULL),

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
    /* MM filter window size characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_MM_FILTER_WINDOW_SIZE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_mm_filter_window_size, write_mm_filter_window_size, NULL),

    /* MM filter poly order characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_MM_FILTER_POLY_ORDER,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_mm_filter_poly_order, write_mm_filter_poly_order, NULL),

    /* MM series capacity characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_MM_SERIES_CAPACITY,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_mm_series_capacity, write_mm_series_capacity, NULL),

    /* MM phase filter window size characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_MM_PHASE_FILTER_WINDOW_SIZE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_mm_phase_filter_window_size, write_mm_phase_filter_window_size, NULL),

    /* MM phase filter poly order characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_MM_PHASE_FILTER_POLY_ORDER,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_mm_phase_filter_poly_order, write_mm_phase_filter_poly_order, NULL),

    /* MM TWR filter window size characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_MM_TWR_FILTER_WINDOW_SIZE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_mm_twr_filter_window_size, write_mm_twr_filter_window_size, NULL),

    /* MM TWR filter poly order characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NETWORK_MM_TWR_FILTER_POLY_ORDER,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_mm_twr_filter_poly_order, write_mm_twr_filter_poly_order, NULL),
#endif /* CONFIG_NODE_TABLE_MM_ENABLED */
);

int network_settings_service_init(void)
{
    LOG_INF("Network settings service initialized");
    /* Service is automatically registered via BT_GATT_SERVICE_DEFINE */
    return 0;
}
