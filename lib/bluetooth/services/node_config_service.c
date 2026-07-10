/**
 * @file node_config_service.c
 * @brief GATT service for configuring node parameters
 *
 * Provides characteristics for:
 * - Node position (x, y, z in meters)
 * - Node ID (16-bit address)
 * - Node mode (operating mode)
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <app/lib/system/node.h>
#include <app/lib/timesync/time_synchronization.h>
#include <app/lib/scheduling/upper/block_scheduler.h>
#include "../bt_services.h"
#include "node_config_service.h"

LOG_MODULE_REGISTER(node_config_svc, CONFIG_LOG_DEFAULT_LEVEL);

/* Custom UUID for node configuration service */
/* Base UUID: 6e6f6465-636f-6e66-6967-000000000000 (ASCII "nodeconfig") */
#define BT_UUID_NODE_CONFIG_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x6e6f6465, 0x636f, 0x6e66, 0x6967, 0x000000000000ULL)

#define BT_UUID_NODE_CONFIG_SERVICE \
    BT_UUID_DECLARE_128(BT_UUID_NODE_CONFIG_SERVICE_VAL)

/* Position characteristic UUID: ...0001 */
#define BT_UUID_NODE_POSITION_VAL \
    BT_UUID_128_ENCODE(0x6e6f6465, 0x636f, 0x6e66, 0x6967, 0x000000000001ULL)

#define BT_UUID_NODE_POSITION \
    BT_UUID_DECLARE_128(BT_UUID_NODE_POSITION_VAL)

/* Node ID characteristic UUID: ...0002 */
#define BT_UUID_NODE_ID_VAL \
    BT_UUID_128_ENCODE(0x6e6f6465, 0x636f, 0x6e66, 0x6967, 0x000000000002ULL)

#define BT_UUID_NODE_ID \
    BT_UUID_DECLARE_128(BT_UUID_NODE_ID_VAL)

/* Node mode characteristic UUID: ...0003 */
#define BT_UUID_NODE_MODE_VAL \
    BT_UUID_128_ENCODE(0x6e6f6465, 0x636f, 0x6e66, 0x6967, 0x000000000003ULL)

#define BT_UUID_NODE_MODE \
    BT_UUID_DECLARE_128(BT_UUID_NODE_MODE_VAL)

/* Network root (is_root) characteristic UUID: ...0004 */
#define BT_UUID_NODE_IS_ROOT_VAL \
    BT_UUID_128_ENCODE(0x6e6f6465, 0x636f, 0x6e66, 0x6967, 0x000000000004ULL)

#define BT_UUID_NODE_IS_ROOT \
    BT_UUID_DECLARE_128(BT_UUID_NODE_IS_ROOT_VAL)

/* Position mode characteristic UUID: ...0005 */
#define BT_UUID_NODE_POSITION_MODE_VAL \
    BT_UUID_128_ENCODE(0x6e6f6465, 0x636f, 0x6e66, 0x6967, 0x000000000005ULL)

#define BT_UUID_NODE_POSITION_MODE \
    BT_UUID_DECLARE_128(BT_UUID_NODE_POSITION_MODE_VAL)

/* Free-running mode characteristic UUID: ...0006 */
#define BT_UUID_NODE_FREE_RUNNING_VAL \
    BT_UUID_128_ENCODE(0x6e6f6465, 0x636f, 0x6e66, 0x6967, 0x000000000006ULL)

#define BT_UUID_NODE_FREE_RUNNING \
    BT_UUID_DECLARE_128(BT_UUID_NODE_FREE_RUNNING_VAL)

/* Sync mode characteristic UUID: ...0007 */
#define BT_UUID_NODE_SYNC_MODE_VAL \
    BT_UUID_128_ENCODE(0x6e6f6465, 0x636f, 0x6e66, 0x6967, 0x000000000007ULL)

#define BT_UUID_NODE_SYNC_MODE \
    BT_UUID_DECLARE_128(BT_UUID_NODE_SYNC_MODE_VAL)

/* Position characteristic: 3 doubles (24 bytes) */
static ssize_t read_position(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    struct node_config_position pos;
    int err = get_node_config_position(&pos);

    if (err) {
        LOG_WRN("Position not set, returning zeros");
        memset(&pos, 0, sizeof(pos));
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pos, sizeof(pos));
}

static ssize_t write_position(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset,
                              uint8_t flags)
{
    struct node_config_position pos;

    if (offset != 0) {
        LOG_ERR("Position write with offset not supported");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(pos)) {
        LOG_ERR("Invalid position length: %u (expected %u)", len, sizeof(pos));
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&pos, buf, sizeof(pos));

    int err = set_node_config_position(&pos);
    if (err) {
        LOG_ERR("Failed to set position (err %d)", err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    LOG_INF("Position updated via BLE: (%.3f, %.3f, %.3f)", pos.x, pos.y, pos.z);
    return len;
}

/* Node ID characteristic: uint16_t (2 bytes) */
static ssize_t read_node_id(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    deca_short_addr_t node_id = get_node_addr();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &node_id, sizeof(node_id));
}

/* Note: Node ID is read-only (derived from hardware ID) */

/* Node mode characteristic: uint8_t (1 byte) */
static ssize_t read_node_mode(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    node_mode_t mode = get_node_mode();
    uint8_t mode_val = (uint8_t)mode;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &mode_val, sizeof(mode_val));
}

static ssize_t write_node_mode(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len, uint16_t offset,
                               uint8_t flags)
{
    uint8_t mode_val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(mode_val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&mode_val, buf, sizeof(mode_val));

    /* Validate mode value */
    if (mode_val > NODE_MODE_RANGING_DISABLED) {
        LOG_ERR("Invalid node mode: %u", mode_val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    set_node_mode((node_mode_t)mode_val);
    LOG_INF("Node mode updated via BLE: %u", mode_val);

    return len;
}

/* Network root (is_root) characteristic: uint8_t (1 byte, 0=false, 1=true) */
static ssize_t read_is_root(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    bool is_root = get_node_is_root();
    uint8_t is_root_val = is_root ? 1 : 0;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &is_root_val, sizeof(is_root_val));
}

static ssize_t write_is_root(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset,
                             uint8_t flags)
{
    uint8_t is_root_val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(is_root_val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&is_root_val, buf, sizeof(is_root_val));

    /* Accept 0 (false) or 1 (true) */
    if (is_root_val > 1) {
        LOG_ERR("Invalid is_root value: %u (must be 0 or 1)", is_root_val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    bool is_root = (is_root_val != 0);

    int err = set_node_is_root(is_root);
    if (err) {
        LOG_ERR("Failed to set is_root (err %d)", err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    /* Apply the change immediately to the running time synchronization */
    time_sync_set_root_mode(is_root);

    LOG_INF("Network root mode updated via BLE: %s", is_root ? "enabled" : "disabled");

    return len;
}

/* Position mode characteristic: uint8_t (1 byte) */
static ssize_t read_position_mode(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
    node_position_mode_t mode = get_node_position_mode();
    uint8_t mode_val = (uint8_t)mode;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &mode_val, sizeof(mode_val));
}

static ssize_t write_position_mode(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len, uint16_t offset,
                                   uint8_t flags)
{
    uint8_t mode_val;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(mode_val)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&mode_val, buf, sizeof(mode_val));

    /* Validate position mode value */
    if (mode_val > NODE_POSITION_MODE_BELIEF_PROPAGATION) {
        LOG_ERR("Invalid position mode: %u", mode_val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    int err = set_node_position_mode((node_position_mode_t)mode_val);
    if (err) {
        LOG_ERR("Failed to set position mode (err %d)", err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    LOG_INF("Position mode updated via BLE: %u", mode_val);

    return len;
}

/* Free-running mode characteristic: uint8_t (1 byte, 0=false, 1=true) */
static ssize_t read_free_running(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = time_sync_is_free_running() ? 1 : 0;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_free_running(struct bt_conn *conn,
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

    if (val > 1) {
        LOG_ERR("Invalid free_running value: %u (must be 0 or 1)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    bool enable = (val != 0);

    /* Persist to NVS */
    int err = set_node_free_running(enable);
    if (err) {
        LOG_ERR("Failed to persist free_running (err %d)", err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    /* Apply to running time sync */
    time_sync_set_free_running(enable);

    /* Rebuild superframe so the scheduler applies the new mode */
    block_scheduler_rebuild_superframe();

    LOG_INF("Free-running mode updated via BLE: %s", enable ? "enabled" : "disabled");

    return len;
}

/* Sync mode read/write handlers */
static ssize_t read_sync_mode(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t val = get_node_sync_mode();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_sync_mode(struct bt_conn *conn,
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

    if (val > 1) {
        LOG_ERR("Invalid sync_mode value: %u (must be 0 or 1)", val);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    /* Persist to NVS */
    int err = set_node_sync_mode(val);
    if (err) {
        LOG_ERR("Failed to persist sync_mode (err %d)", err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    /* Apply to running time sync */
    time_sync_set_mode(val == 1 ? TIME_SYNC_MODE_OFFSET : TIME_SYNC_MODE_SKEW);

    /* Rebuild superframe so the scheduler applies the new mode */
    block_scheduler_rebuild_superframe();

    LOG_INF("Sync mode updated via BLE: %s", val == 1 ? "OFFSET" : "SKEW");

    return len;
}

/* Node configuration service definition */
BT_GATT_SERVICE_DEFINE(node_config_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_NODE_CONFIG_SERVICE),

    /* Position characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NODE_POSITION,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_position, write_position, NULL),
    BT_GATT_CUD("Position", BT_GATT_PERM_READ),

    /* Node ID characteristic (read-only) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NODE_ID,
                          BT_GATT_CHRC_READ,
                          BT_GATT_PERM_READ,
                          read_node_id, NULL, NULL),
    BT_GATT_CUD("Node ID", BT_GATT_PERM_READ),

    /* Node mode characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NODE_MODE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_node_mode, write_node_mode, NULL),
    BT_GATT_CUD("Mode", BT_GATT_PERM_READ),

    /* Network root (is_root) characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NODE_IS_ROOT,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_is_root, write_is_root, NULL),
    BT_GATT_CUD("Root", BT_GATT_PERM_READ),

    /* Position mode characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NODE_POSITION_MODE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_position_mode, write_position_mode, NULL),
    BT_GATT_CUD("PosMode", BT_GATT_PERM_READ),

    /* Free-running mode characteristic (read/write) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NODE_FREE_RUNNING,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_free_running, write_free_running, NULL),
    BT_GATT_CUD("FreeRun", BT_GATT_PERM_READ),

    /* Sync mode characteristic (read/write) - 0=SKEW, 1=OFFSET */
    BT_GATT_CHARACTERISTIC(BT_UUID_NODE_SYNC_MODE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_sync_mode, write_sync_mode, NULL),
    BT_GATT_CUD("SyncMode", BT_GATT_PERM_READ),
);

int node_config_service_init(void)
{
    LOG_INF("Node configuration service initialized");
    /* Service is automatically registered via BT_GATT_SERVICE_DEFINE */
    return 0;
}
