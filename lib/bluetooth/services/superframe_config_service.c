/**
 * @file superframe_config_service.c
 * @brief GATT service for superframe configuration
 *
 * Provides characteristics for:
 * - Block count (R/W): Number of blocks in superframe (1-16)
 * - Slot index (W): Selects which block to read/write via BlockConfig
 * - Block config (R/W): Configuration for the selected block
 * - Apply (W): Write 1 to rebuild and restart superframe
 * - Status (R/N): Reports apply status with notifications
 * - Full config (R): Read entire superframe configuration at once
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <app/lib/management/superframe_settings.h>
#include <app/lib/blocks/block_types.h>
#include <app/lib/scheduling/upper/block_scheduler.h>
#include "superframe_config_service.h"

LOG_MODULE_REGISTER(superframe_config_svc, CONFIG_LOG_DEFAULT_LEVEL);

/* Custom UUID for superframe config service */
/* Base UUID: 73757065-7266-7261-6d65-000000000000 (ASCII "superframe") */
#define BT_UUID_SUPERFRAME_CONFIG_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x73757065, 0x7266, 0x7261, 0x6d65, 0x000000000000ULL)

#define BT_UUID_SUPERFRAME_CONFIG_SERVICE \
    BT_UUID_DECLARE_128(BT_UUID_SUPERFRAME_CONFIG_SERVICE_VAL)

/* Block count characteristic UUID: ...0001 */
#define BT_UUID_SUPERFRAME_BLOCK_COUNT_VAL \
    BT_UUID_128_ENCODE(0x73757065, 0x7266, 0x7261, 0x6d65, 0x000000000001ULL)

#define BT_UUID_SUPERFRAME_BLOCK_COUNT \
    BT_UUID_DECLARE_128(BT_UUID_SUPERFRAME_BLOCK_COUNT_VAL)

/* Slot index characteristic UUID: ...0002 */
#define BT_UUID_SUPERFRAME_SLOT_INDEX_VAL \
    BT_UUID_128_ENCODE(0x73757065, 0x7266, 0x7261, 0x6d65, 0x000000000002ULL)

#define BT_UUID_SUPERFRAME_SLOT_INDEX \
    BT_UUID_DECLARE_128(BT_UUID_SUPERFRAME_SLOT_INDEX_VAL)

/* Block config characteristic UUID: ...0003 */
#define BT_UUID_SUPERFRAME_BLOCK_CONFIG_VAL \
    BT_UUID_128_ENCODE(0x73757065, 0x7266, 0x7261, 0x6d65, 0x000000000003ULL)

#define BT_UUID_SUPERFRAME_BLOCK_CONFIG \
    BT_UUID_DECLARE_128(BT_UUID_SUPERFRAME_BLOCK_CONFIG_VAL)

/* Apply characteristic UUID: ...0004 */
#define BT_UUID_SUPERFRAME_APPLY_VAL \
    BT_UUID_128_ENCODE(0x73757065, 0x7266, 0x7261, 0x6d65, 0x000000000004ULL)

#define BT_UUID_SUPERFRAME_APPLY \
    BT_UUID_DECLARE_128(BT_UUID_SUPERFRAME_APPLY_VAL)

/* Status characteristic UUID: ...0005 */
#define BT_UUID_SUPERFRAME_STATUS_VAL \
    BT_UUID_128_ENCODE(0x73757065, 0x7266, 0x7261, 0x6d65, 0x000000000005ULL)

#define BT_UUID_SUPERFRAME_STATUS \
    BT_UUID_DECLARE_128(BT_UUID_SUPERFRAME_STATUS_VAL)

/* Full config characteristic UUID: ...0006 */
#define BT_UUID_SUPERFRAME_FULL_CONFIG_VAL \
    BT_UUID_128_ENCODE(0x73757065, 0x7266, 0x7261, 0x6d65, 0x000000000006ULL)

#define BT_UUID_SUPERFRAME_FULL_CONFIG \
    BT_UUID_DECLARE_128(BT_UUID_SUPERFRAME_FULL_CONFIG_VAL)

/* Block timing stats characteristic UUID: ...0007 */
#define BT_UUID_SUPERFRAME_BLOCK_TIMING_VAL \
    BT_UUID_128_ENCODE(0x73757065, 0x7266, 0x7261, 0x6d65, 0x000000000007ULL)

#define BT_UUID_SUPERFRAME_BLOCK_TIMING \
    BT_UUID_DECLARE_128(BT_UUID_SUPERFRAME_BLOCK_TIMING_VAL)

/* Reset block timing stats characteristic UUID: ...0008 */
#define BT_UUID_SUPERFRAME_RESET_BLOCK_TIMING_VAL \
    BT_UUID_128_ENCODE(0x73757065, 0x7266, 0x7261, 0x6d65, 0x000000000008ULL)

#define BT_UUID_SUPERFRAME_RESET_BLOCK_TIMING \
    BT_UUID_DECLARE_128(BT_UUID_SUPERFRAME_RESET_BLOCK_TIMING_VAL)

/* Service state */
static uint8_t selected_slot_index;
static uint8_t service_status = SUPERFRAME_STATUS_IDLE;
static bool status_notify_enabled;

/* ========================================================================== */
/* Block Count (uint8_t, 1 byte) */
/* ========================================================================== */

static ssize_t read_block_count(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    uint8_t count = superframe_settings_get_block_count();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &count, sizeof(count));
}

static ssize_t write_block_count(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset,
                                 uint8_t flags)
{
    uint8_t count;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(count)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&count, buf, sizeof(count));

    if (count == 0 || count > SUPERFRAME_MAX_BLOCKS) {
        LOG_ERR("Invalid block count: %u (must be 1-%d)", count, SUPERFRAME_MAX_BLOCKS);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    int ret = superframe_settings_set_block_count(count);
    if (ret < 0) {
        LOG_ERR("Failed to set block count: %d", ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    service_status = SUPERFRAME_STATUS_PENDING;
    LOG_INF("Block count updated via BLE: %u", count);

    return len;
}

/* ========================================================================== */
/* Slot Index (uint8_t, 1 byte, write-only) */
/* ========================================================================== */

static ssize_t write_slot_index(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset,
                                uint8_t flags)
{
    uint8_t index;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(index)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&index, buf, sizeof(index));

    if (index >= SUPERFRAME_MAX_BLOCKS) {
        LOG_ERR("Invalid slot index: %u (must be < %d)", index, SUPERFRAME_MAX_BLOCKS);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    selected_slot_index = index;
    LOG_DBG("Slot index set to %u", index);

    return len;
}

/* ========================================================================== */
/* Block Config (BLOCK_CONFIG_BLE_SIZE bytes) */
/* ========================================================================== */

static ssize_t read_block_config(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    struct block_config_ble config;
    int ret;

    ret = superframe_settings_get_block(selected_slot_index, &config);
    if (ret < 0) {
        LOG_ERR("Failed to read block %u: %d", selected_slot_index, ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &config, sizeof(config));
}

static ssize_t write_block_config(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len, uint16_t offset,
                                  uint8_t flags)
{
    struct block_config_ble config;
    int ret;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(config)) {
        LOG_ERR("Invalid block config length: %u (expected %zu)", len, sizeof(config));
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&config, buf, sizeof(config));

    /* Validate block type */
    if (config.type >= BLOCK_TYPE_MAX) {
        LOG_ERR("Invalid block type: %u", config.type);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    ret = superframe_settings_set_block(selected_slot_index, &config);
    if (ret < 0) {
        LOG_ERR("Failed to set block %u: %d", selected_slot_index, ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    service_status = SUPERFRAME_STATUS_PENDING;
    LOG_INF("Block %u config updated via BLE (type=%u)", selected_slot_index, config.type);

    return len;
}

/* ========================================================================== */
/* Apply (uint8_t, 1 byte, write-only) */
/* ========================================================================== */

static ssize_t write_apply(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           const void *buf, uint16_t len, uint16_t offset,
                           uint8_t flags)
{
    uint8_t val;
    int ret;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&val, buf, sizeof(val));

    if (val != 1) {
        LOG_DBG("Apply characteristic written with value %u (ignored, write 1 to apply)", val);
        return len;
    }

    /* Validate configuration first */
    ret = superframe_settings_validate();
    if (ret < 0) {
        LOG_ERR("Superframe configuration validation failed: %d", ret);
        service_status = SUPERFRAME_STATUS_ERROR_INVALID;
        /* TODO: Notify status change if enabled */
        return len;  /* Accept write but set error status */
    }

    /* Save to NVS */
    ret = superframe_settings_save();
    if (ret < 0) {
        LOG_ERR("Failed to save superframe settings: %d", ret);
        /* Continue with apply anyway - runtime will work, just won't persist */
    }

    /* Apply the configuration */
    ret = superframe_settings_apply();
    if (ret < 0) {
        LOG_ERR("Failed to apply superframe settings: %d", ret);
        service_status = SUPERFRAME_STATUS_ERROR_APPLY_FAILED;
    } else {
        LOG_INF("Superframe configuration applied successfully");
        service_status = SUPERFRAME_STATUS_APPLIED;
    }

    /* TODO: Notify status change if enabled */

    return len;
}

/* ========================================================================== */
/* Status (uint8_t, 1 byte, read with notifications) */
/* ========================================================================== */

static ssize_t read_status(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &service_status, sizeof(service_status));
}

static void status_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Status notifications %s", status_notify_enabled ? "enabled" : "disabled");
}

/* ========================================================================== */
/* Full Config (read entire superframe) */
/* ========================================================================== */

static ssize_t read_full_config(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    struct superframe_config_ble config;
    int ret;
    size_t config_size;

    ret = superframe_settings_get(&config);
    if (ret < 0) {
        LOG_ERR("Failed to read superframe config: %d", ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    /* Calculate actual size based on block count */
    config_size = 1 + (config.block_count * sizeof(struct block_config_ble));

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &config, config_size);
}

/* ========================================================================== */
/* Block Timing Stats (read-only, max execution time per block) */
/* ========================================================================== */

static ssize_t read_block_timing(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    uint8_t block_count = superframe_settings_get_block_count();
    uint16_t timing_data[SUPERFRAME_MAX_BLOCKS];

    int ret = block_scheduler_get_timing_stats(timing_data, block_count);
    if (ret < 0) {
        LOG_ERR("Failed to get timing stats: %d", ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    /* Format: [block_count (1 byte)] + [max_time_ms per block (2 bytes each, LE)] */
    uint8_t response[1 + SUPERFRAME_MAX_BLOCKS * 2];
    response[0] = block_count;
    memcpy(&response[1], timing_data, block_count * sizeof(uint16_t));

    size_t response_size = 1 + (block_count * sizeof(uint16_t));
    return bt_gatt_attr_read(conn, attr, buf, len, offset, response, response_size);
}

/* ========================================================================== */
/* Reset Block Timing Stats (write-only, resets max execution times) */
/* ========================================================================== */

static ssize_t write_reset_block_timing(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        const void *buf, uint16_t len,
                                        uint16_t offset, uint8_t flags)
{
    block_scheduler_reset_timing_stats();
    LOG_INF("Block timing stats reset via BLE");
    return len;
}

/* ========================================================================== */
/* GATT Service Definition */
/* ========================================================================== */

BT_GATT_SERVICE_DEFINE(superframe_config_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_SUPERFRAME_CONFIG_SERVICE),

    /* Block count characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_SUPERFRAME_BLOCK_COUNT,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_block_count, write_block_count, NULL),
    BT_GATT_CUD("BlockCount", BT_GATT_PERM_READ),

    /* Slot index characteristic (write-only) */
    BT_GATT_CHARACTERISTIC(BT_UUID_SUPERFRAME_SLOT_INDEX,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, write_slot_index, NULL),
    BT_GATT_CUD("SlotIndex", BT_GATT_PERM_READ),

    /* Block config characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_SUPERFRAME_BLOCK_CONFIG,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_block_config, write_block_config, NULL),
    BT_GATT_CUD("BlockConfig", BT_GATT_PERM_READ),

    /* Apply characteristic (write-only) */
    BT_GATT_CHARACTERISTIC(BT_UUID_SUPERFRAME_APPLY,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, write_apply, NULL),
    BT_GATT_CUD("Apply", BT_GATT_PERM_READ),

    /* Status characteristic (read with notifications) */
    BT_GATT_CHARACTERISTIC(BT_UUID_SUPERFRAME_STATUS,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_status, NULL, NULL),
    BT_GATT_CCC(status_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("Status", BT_GATT_PERM_READ),

    /* Full config characteristic (read-only) */
    BT_GATT_CHARACTERISTIC(BT_UUID_SUPERFRAME_FULL_CONFIG,
                          BT_GATT_CHRC_READ,
                          BT_GATT_PERM_READ,
                          read_full_config, NULL, NULL),
    BT_GATT_CUD("FullConfig", BT_GATT_PERM_READ),

    /* Block timing stats characteristic (read-only) */
    BT_GATT_CHARACTERISTIC(BT_UUID_SUPERFRAME_BLOCK_TIMING,
                          BT_GATT_CHRC_READ,
                          BT_GATT_PERM_READ,
                          read_block_timing, NULL, NULL),
    BT_GATT_CUD("BlockTiming", BT_GATT_PERM_READ),

    /* Reset block timing stats characteristic (write-only) */
    BT_GATT_CHARACTERISTIC(BT_UUID_SUPERFRAME_RESET_BLOCK_TIMING,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, write_reset_block_timing, NULL),
    BT_GATT_CUD("ResetBlockTiming", BT_GATT_PERM_READ),
);

int superframe_config_service_init(void)
{
    LOG_INF("Superframe config service initialized");
    /* Service is automatically registered via BT_GATT_SERVICE_DEFINE */
    return 0;
}
