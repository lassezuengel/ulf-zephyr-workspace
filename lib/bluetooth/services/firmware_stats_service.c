/*
 * Firmware Statistics BLE GATT Service
 *
 * Exposes on-demand statistics subscription/read/notify over BLE.
 * Uses the select-then-read pattern (like A/B testing service).
 * Change-driven notifications via k_work callback from the stats core.
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "firmware_stats_service.h"
#include <app/lib/stats/firmware_stats.h>

LOG_MODULE_REGISTER(firmware_stats_svc, CONFIG_LOG_DEFAULT_LEVEL);

/* ---- State -------------------------------------------------------------- */

static uint32_t selected_stat_id;
static bool value_notifications_enabled;

/* Pointer to the Value characteristic attribute (for bt_gatt_notify).
 * Resolved in the init function from the service attrs array. */
static const struct bt_gatt_attr *value_attr;

/* ---- UUIDs -------------------------------------------------------------- */

static struct bt_uuid_128 svc_uuid = BT_UUID_INIT_128(
	FIRMWARE_STATS_SERVICE_UUID_VAL);

static struct bt_uuid_128 subscribe_uuid = BT_UUID_INIT_128(
	FIRMWARE_STATS_SUBSCRIBE_UUID_VAL);

static struct bt_uuid_128 unsubscribe_uuid = BT_UUID_INIT_128(
	FIRMWARE_STATS_UNSUBSCRIBE_UUID_VAL);

static struct bt_uuid_128 select_uuid = BT_UUID_INIT_128(
	FIRMWARE_STATS_SELECT_UUID_VAL);

static struct bt_uuid_128 value_uuid = BT_UUID_INIT_128(
	FIRMWARE_STATS_VALUE_UUID_VAL);

static struct bt_uuid_128 active_count_uuid = BT_UUID_INIT_128(
	FIRMWARE_STATS_ACTIVE_COUNT_UUID_VAL);

/* ---- Subscribe characteristic (write-only) ------------------------------ */

static ssize_t write_subscribe(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len,
			       uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint32_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint32_t stat_id = sys_get_le32(buf);
	int rc = firmware_stats_subscribe(stat_id);

	if (rc == -ENOMEM) {
		LOG_WRN("Stats subscribe 0x%08x: no free slots", stat_id);
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	}
	if (rc == -EALREADY) {
		/* Already subscribed is fine -- idempotent */
		LOG_DBG("Stats 0x%08x already active", stat_id);
	}

	return len;
}

/* ---- Unsubscribe characteristic (write-only) ---------------------------- */

static ssize_t write_unsubscribe(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len,
				 uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint32_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint32_t stat_id = sys_get_le32(buf);
	firmware_stats_unsubscribe(stat_id);

	return len;
}

/* ---- Select characteristic (read/write) --------------------------------- */

static ssize_t read_select(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	uint32_t val = sys_cpu_to_le32(selected_stat_id);
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &val, sizeof(val));
}

static ssize_t write_select(struct bt_conn *conn,
			    const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len,
			    uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint32_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	selected_stat_id = sys_get_le32(buf);
	LOG_DBG("Stats select: 0x%08x", selected_stat_id);

	return len;
}

/* ---- Value characteristic (read/notify) --------------------------------- */

static ssize_t read_value(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf, uint16_t len, uint16_t offset)
{
	uint32_t raw_val = 0;
	firmware_stats_read(selected_stat_id, &raw_val);
	uint32_t val = sys_cpu_to_le32(raw_val);
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &val, sizeof(val));
}

static void value_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	value_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Stats value notifications %s",
		value_notifications_enabled ? "enabled" : "disabled");
}

/* ---- Active count characteristic (read-only) ---------------------------- */

static ssize_t read_active_count(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	uint8_t count = firmware_stats_active_count();
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &count, sizeof(count));
}

/* ---- Notification callback (invoked via k_work from stats core) --------- */

static void stats_notify_callback(void)
{
	if (!value_notifications_enabled || !value_attr) {
		return;
	}

	uint32_t raw_val;
	if (firmware_stats_check_and_clear_dirty(selected_stat_id, &raw_val)) {
		uint32_t val = sys_cpu_to_le32(raw_val);
		int err = bt_gatt_notify(NULL, value_attr, &val, sizeof(val));
		if (err && err != -ENOTCONN) {
			LOG_WRN("Stats notify failed: %d", err);
		}
	}
}

/* ---- GATT service definition -------------------------------------------- */

/*
 * Attribute index layout:
 *   [0]  Primary Service declaration
 *   [1]  Subscribe char declaration
 *   [2]  Subscribe char value
 *   [3]  Unsubscribe char declaration
 *   [4]  Unsubscribe char value
 *   [5]  Select char declaration
 *   [6]  Select char value
 *   [7]  Value char declaration
 *   [8]  Value char value          <-- value_attr
 *   [9]  Value CCC descriptor
 *   [10] Active Count char declaration
 *   [11] Active Count char value
 */

BT_GATT_SERVICE_DEFINE(firmware_stats_svc,
	BT_GATT_PRIMARY_SERVICE(&svc_uuid),

	/* Subscribe (write-only) */
	BT_GATT_CHARACTERISTIC(&subscribe_uuid.uuid,
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_subscribe, NULL),

	/* Unsubscribe (write-only) */
	BT_GATT_CHARACTERISTIC(&unsubscribe_uuid.uuid,
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_unsubscribe, NULL),

	/* Select (read/write) */
	BT_GATT_CHARACTERISTIC(&select_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_select, write_select, NULL),

	/* Value (read/notify) */
	BT_GATT_CHARACTERISTIC(&value_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_value, NULL, NULL),
	BT_GATT_CCC(value_ccc_changed,
		     BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Active Count (read-only) */
	BT_GATT_CHARACTERISTIC(&active_count_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_active_count, NULL, NULL),
);

/* ---- Init --------------------------------------------------------------- */

static int firmware_stats_service_init(void)
{
	/* Resolve the Value characteristic attr pointer.
	 * Index 8 = value char value attribute (see layout above). */
	value_attr = &firmware_stats_svc.attrs[8];

	/* Register our notification callback with the stats core */
	firmware_stats_set_notify_callback(stats_notify_callback);

	LOG_INF("Firmware stats BLE service initialized");
	return 0;
}

SYS_INIT(firmware_stats_service_init, APPLICATION, 91);
