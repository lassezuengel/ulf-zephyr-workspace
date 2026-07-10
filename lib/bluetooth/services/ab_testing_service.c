/*
 * A/B Testing BLE Service
 *
 * Exposes a generic key-value store over BLE for runtime comparison of
 * firmware algorithm variants without reflashing.
 *
 * Protocol:
 *   1. Write the toggle index to the Index characteristic.
 *   2. Read or write the Value characteristic to get/set that toggle.
 *
 * All values default to 0 (variant A / legacy behaviour).
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "ab_testing_service.h"
#include <app/lib/bluetooth/ab_testing.h>

LOG_MODULE_REGISTER(ab_testing_service, CONFIG_LOG_DEFAULT_LEVEL);

/* ---- storage ------------------------------------------------------------ */

static uint8_t ab_values[AB_TEST_MAX_TOGGLES];   /* zero-initialised */
static uint8_t selected_index;

/* ---- public getter (called from ranging code) --------------------------- */

uint8_t ab_testing_get_value(uint8_t index)
{
	if (index >= AB_TEST_MAX_TOGGLES) {
		return 0;
	}
	return ab_values[index];
}

/* ---- UUIDs -------------------------------------------------------------- */

static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
	AB_TESTING_SERVICE_UUID_VAL);

static struct bt_uuid_128 index_uuid = BT_UUID_INIT_128(
	AB_TESTING_INDEX_CHAR_UUID_VAL);

static struct bt_uuid_128 value_uuid = BT_UUID_INIT_128(
	AB_TESTING_VALUE_CHAR_UUID_VAL);

/* ---- Index characteristic ----------------------------------------------- */

static ssize_t read_index(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &selected_index, sizeof(selected_index));
}

static ssize_t write_index(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   const void *buf, uint16_t len,
			   uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t idx = *(const uint8_t *)buf;
	if (idx >= AB_TEST_MAX_TOGGLES) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	selected_index = idx;
	LOG_DBG("A/B test index set to %u", selected_index);

	return len;
}

/* ---- Value characteristic ----------------------------------------------- */

static ssize_t read_value(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf, uint16_t len, uint16_t offset)
{
	uint8_t val = ab_values[selected_index];
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &val, sizeof(val));
}

static ssize_t write_value(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   const void *buf, uint16_t len,
			   uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t val = *(const uint8_t *)buf;
	ab_values[selected_index] = val;

	LOG_INF("A/B toggle[%u] = %u", selected_index, val);

	return len;
}

/* ---- GATT service definition -------------------------------------------- */

BT_GATT_SERVICE_DEFINE(ab_testing_svc,
	BT_GATT_PRIMARY_SERVICE(&service_uuid),

	/* Index characteristic (read/write uint8) */
	BT_GATT_CHARACTERISTIC(&index_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_index, write_index, NULL),

	/* Value characteristic (read/write uint8) */
	BT_GATT_CHARACTERISTIC(&value_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_value, write_value, NULL),
);
