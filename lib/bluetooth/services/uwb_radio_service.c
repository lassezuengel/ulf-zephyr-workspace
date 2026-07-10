/*
 * Copyright (c) 2024 SynchroFly
 * SPDX-License-Identifier: Apache-2.0
 *
 * UWB Radio Settings BLE Service
 *
 * Provides BLE characteristics for configuring UWB antenna delays.
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include <app/lib/system/hw.h>
#include <app/lib/blocks/mtm.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>

#include "uwb_radio_service.h"
#include "poly_correction.h"
#include "xrloc_correction.h"

LOG_MODULE_REGISTER(uwb_radio_service, CONFIG_BT_GATT_LOG_LEVEL);

/* Service UUID: "uwbradiose" in ASCII hex */
#define UWB_RADIO_SERVICE_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000000)

/* Characteristic UUIDs */
#define UWB_TX_ANT_DELAY_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000001)

#define UWB_RX_ANT_DELAY_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000002)

#define UWB_BIAS_CORRECTION_MODE_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000003)

#define UWB_REJECT_FRAMES_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000004)

#define UWB_FP_INDEX_THRESHOLD_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000005)

#define UWB_SMART_POWER_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000006)

#define UWB_TX_POWER_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000007)

#define UWB_POLY_GLOBAL_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000008)

#define UWB_POLY_NODE_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x000000000009)

#define UWB_POLY_NODE_CLEAR_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x00000000000a)

#define UWB_XRLOC_NODE_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x00000000000b)

#define UWB_XRLOC_NODE_CLEAR_CHAR_UUID \
	BT_UUID_128_ENCODE(0x75776272, 0x6164, 0x696f, 0x7365, 0x00000000000c)

static struct bt_uuid_128 uwb_radio_service_uuid = BT_UUID_INIT_128(UWB_RADIO_SERVICE_UUID);
static struct bt_uuid_128 tx_ant_delay_uuid = BT_UUID_INIT_128(UWB_TX_ANT_DELAY_CHAR_UUID);
static struct bt_uuid_128 rx_ant_delay_uuid = BT_UUID_INIT_128(UWB_RX_ANT_DELAY_CHAR_UUID);
static struct bt_uuid_128 bias_correction_mode_uuid = BT_UUID_INIT_128(UWB_BIAS_CORRECTION_MODE_CHAR_UUID);
static struct bt_uuid_128 reject_frames_uuid = BT_UUID_INIT_128(UWB_REJECT_FRAMES_CHAR_UUID);
static struct bt_uuid_128 fp_index_threshold_uuid = BT_UUID_INIT_128(UWB_FP_INDEX_THRESHOLD_CHAR_UUID);
static struct bt_uuid_128 smart_power_uuid = BT_UUID_INIT_128(UWB_SMART_POWER_CHAR_UUID);
static struct bt_uuid_128 tx_power_uuid = BT_UUID_INIT_128(UWB_TX_POWER_CHAR_UUID);
static struct bt_uuid_128 poly_global_uuid = BT_UUID_INIT_128(UWB_POLY_GLOBAL_CHAR_UUID);
static struct bt_uuid_128 poly_node_uuid = BT_UUID_INIT_128(UWB_POLY_NODE_CHAR_UUID);
static struct bt_uuid_128 poly_node_clear_uuid = BT_UUID_INIT_128(UWB_POLY_NODE_CLEAR_CHAR_UUID);
static struct bt_uuid_128 xrloc_node_uuid = BT_UUID_INIT_128(UWB_XRLOC_NODE_CHAR_UUID);
static struct bt_uuid_128 xrloc_node_clear_uuid = BT_UUID_INIT_128(UWB_XRLOC_NODE_CLEAR_CHAR_UUID);

/* Read TX antenna delay */
static ssize_t read_tx_antenna_delay(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     void *buf, uint16_t len, uint16_t offset)
{
	const uwb_driver_t *drv = uwb_driver_get(ieee802154_dev);
	if (!drv || !drv->get_antenna_delay) {
		LOG_ERR("UWB driver or get_antenna_delay not available");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	uwb_antenna_delay_t delay;
	drv->get_antenna_delay(ieee802154_dev, &delay);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &delay.tx_ant_dly, sizeof(delay.tx_ant_dly));
}

/* Write TX antenna delay */
static ssize_t write_tx_antenna_delay(struct bt_conn *conn,
				      const struct bt_gatt_attr *attr,
				      const void *buf, uint16_t len,
				      uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(uint16_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uwb_driver_t *drv = uwb_driver_get(ieee802154_dev);
	if (!drv || !drv->get_antenna_delay || !drv->set_antenna_delay) {
		LOG_ERR("UWB driver or antenna delay functions not available");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Read current delays, update TX, write back */
	uwb_antenna_delay_t delay;
	drv->get_antenna_delay(ieee802154_dev, &delay);
	memcpy(&delay.tx_ant_dly, buf, sizeof(uint16_t));
	drv->set_antenna_delay(ieee802154_dev, &delay);

	LOG_INF("TX antenna delay set to %u via BLE", delay.tx_ant_dly);

	return len;
}

/* Read RX antenna delay */
static ssize_t read_rx_antenna_delay(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     void *buf, uint16_t len, uint16_t offset)
{
	const uwb_driver_t *drv = uwb_driver_get(ieee802154_dev);
	if (!drv || !drv->get_antenna_delay) {
		LOG_ERR("UWB driver or get_antenna_delay not available");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	uwb_antenna_delay_t delay;
	drv->get_antenna_delay(ieee802154_dev, &delay);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &delay.rx_ant_dly, sizeof(delay.rx_ant_dly));
}

/* Write RX antenna delay */
static ssize_t write_rx_antenna_delay(struct bt_conn *conn,
				      const struct bt_gatt_attr *attr,
				      const void *buf, uint16_t len,
				      uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(uint16_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uwb_driver_t *drv = uwb_driver_get(ieee802154_dev);
	if (!drv || !drv->get_antenna_delay || !drv->set_antenna_delay) {
		LOG_ERR("UWB driver or antenna delay functions not available");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Read current delays, update RX, write back */
	uwb_antenna_delay_t delay;
	drv->get_antenna_delay(ieee802154_dev, &delay);
	memcpy(&delay.rx_ant_dly, buf, sizeof(uint16_t));
	drv->set_antenna_delay(ieee802154_dev, &delay);

	LOG_INF("RX antenna delay set to %u via BLE", delay.rx_ant_dly);

	return len;
}

/* Read bias correction mode (single uint8_t) */
static ssize_t read_bias_correction_mode(struct bt_conn *conn,
					 const struct bt_gatt_attr *attr,
					 void *buf, uint16_t len, uint16_t offset)
{
	uint8_t mode = (uint8_t)mtm_get_bias_correction_mode();
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &mode, sizeof(mode));
}

/* Write bias correction mode (single uint8_t: 0=none, 1=distance, 2=timestamp) */
static ssize_t write_bias_correction_mode(struct bt_conn *conn,
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

	uint8_t mode = *(const uint8_t *)buf;
	if (mode > BIAS_CORRECTION_XRLOC_PER_NODE) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	mtm_set_bias_correction_mode((bias_correction_mode_t)mode);
	LOG_INF("Bias correction mode set to %u via BLE", mode);

	return len;
}

/* Read reject frames (single uint8_t boolean: 0=disabled, 1=enabled) */
static ssize_t read_reject_frames(struct bt_conn *conn,
				  const struct bt_gatt_attr *attr,
				  void *buf, uint16_t len, uint16_t offset)
{
	uint8_t val = mtm_get_reject_frames() ? 1 : 0;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

/* Write reject frames (single uint8_t boolean: 0=disabled, 1=enabled) */
static ssize_t write_reject_frames(struct bt_conn *conn,
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
	mtm_set_correct_reject_frames(val != 0);
	LOG_INF("Reject frames set to %u via BLE", val);

	return len;
}

/* Read FP index threshold (uint16_t little-endian) */
static ssize_t read_fp_index_threshold(struct bt_conn *conn,
				       const struct bt_gatt_attr *attr,
				       void *buf, uint16_t len, uint16_t offset)
{
	uint16_t val = mtm_get_fp_index_threshold();
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

/* Write FP index threshold (uint16_t little-endian) */
static ssize_t write_fp_index_threshold(struct bt_conn *conn,
					const struct bt_gatt_attr *attr,
					const void *buf, uint16_t len,
					uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(uint16_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint16_t val;
	memcpy(&val, buf, sizeof(val));
	mtm_set_fp_index_threshold(val);
	LOG_INF("FP index threshold set to %u via BLE", val);

	return len;
}

/* Read smart power (uint8_t: 0=manual/OFF, 1=smart/ON) */
static ssize_t read_smart_power(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	const uwb_driver_t *drv = uwb_driver_get(ieee802154_dev);
	if (!drv || !drv->get_smart_power) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	uint8_t val = drv->get_smart_power(ieee802154_dev) ? 1 : 0;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

/* Write smart power (uint8_t: 0=manual/OFF, 1=smart/ON) */
static ssize_t write_smart_power(struct bt_conn *conn,
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

	const uwb_driver_t *drv = uwb_driver_get(ieee802154_dev);
	if (!drv || !drv->set_smart_power) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	uint8_t val = *(const uint8_t *)buf;
	drv->set_smart_power(ieee802154_dev, val != 0);
	LOG_INF("Smart TX power set to %s via BLE", val ? "ON" : "OFF");

	return len;
}

/* Read TX power (uint32_t little-endian: raw TX_POWER register) */
static ssize_t read_tx_power(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	const uwb_driver_t *drv = uwb_driver_get(ieee802154_dev);
	if (!drv || !drv->get_tx_power) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	uint32_t power = drv->get_tx_power(ieee802154_dev);
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &power, sizeof(power));
}

/* Write TX power (uint32_t little-endian: raw TX_POWER register) */
static ssize_t write_tx_power(struct bt_conn *conn,
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

	const uwb_driver_t *drv = uwb_driver_get(ieee802154_dev);
	if (!drv || !drv->set_tx_power) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	uint32_t power;
	memcpy(&power, buf, sizeof(power));
	drv->set_tx_power(ieee802154_dev, power);
	LOG_INF("TX power set to 0x%08x via BLE", power);

	return len;
}

/* ------------------------------------------------------------------ */
/* Polynomial correction characteristics                               */
/* ------------------------------------------------------------------ */

/*
 * Read global correction polynomial.
 * Format: [degree:u8][coeffs:f32 LE...][d_min:f32 LE][d_max:f32 LE]
 */
static ssize_t read_poly_global(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	const struct correction_poly *poly = poly_correction_get_global();
	uint8_t resp[1 + POLY_CORRECTION_MAX_COEFFS * sizeof(float) + 2 * sizeof(float)];
	size_t pos = 0;

	resp[pos++] = poly->degree;
	size_t coeff_bytes = ((size_t)poly->degree + 1) * sizeof(float);
	memcpy(resp + pos, poly->coeffs, coeff_bytes);
	pos += coeff_bytes;
	memcpy(resp + pos, &poly->d_min, sizeof(float));
	pos += sizeof(float);
	memcpy(resp + pos, &poly->d_max, sizeof(float));
	pos += sizeof(float);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, resp, pos);
}

/*
 * Write global correction polynomial.
 * Format: [degree:u8][coeffs:f32 LE...][d_min:f32 LE][d_max:f32 LE]
 */
static ssize_t write_poly_global(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len,
				 uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len < 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *data = buf;
	uint8_t degree = data[0];
	if (degree > POLY_CORRECTION_MAX_DEGREE) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	size_t coeff_bytes = ((size_t)degree + 1) * sizeof(float);
	size_t expected = 1 + coeff_bytes + 2 * sizeof(float);
	if (len != expected) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	struct correction_poly poly;
	memset(&poly, 0, sizeof(poly));
	poly.degree = degree;
	memcpy(poly.coeffs, data + 1, coeff_bytes);
	memcpy(&poly.d_min, data + 1 + coeff_bytes, sizeof(float));
	memcpy(&poly.d_max, data + 1 + coeff_bytes + sizeof(float), sizeof(float));

	poly_correction_set_global(&poly);
	LOG_INF("Global correction poly via BLE: degree=%u, range=[%.2f, %.2f]",
		degree, (double)poly.d_min, (double)poly.d_max);

	return len;
}

/*
 * Read per-(node, channel) correction polynomial for the last-written entry.
 * Format: [node_id:u16 LE][channel:u8][degree:u8][coeffs:f32 LE...][d_min:f32][d_max:f32]
 * Returns [0x00 0x00 0x00 0x00] (node_id=0, ch=0, degree=0) if none set.
 */
static uint16_t last_poly_node_id;
static uint8_t last_poly_channel;

static ssize_t read_poly_node(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	uint8_t resp[4 + POLY_CORRECTION_MAX_COEFFS * sizeof(float) + 2 * sizeof(float)];
	memcpy(resp, &last_poly_node_id, sizeof(uint16_t));
	resp[2] = last_poly_channel;

	const struct correction_poly *poly =
		poly_correction_get_node(last_poly_node_id, last_poly_channel);
	if (!poly) {
		resp[3] = 0;
		return bt_gatt_attr_read(conn, attr, buf, len, offset, resp, 4);
	}

	size_t pos = 3;
	resp[pos++] = poly->degree;
	size_t coeff_bytes = ((size_t)poly->degree + 1) * sizeof(float);
	memcpy(resp + pos, poly->coeffs, coeff_bytes);
	pos += coeff_bytes;
	memcpy(resp + pos, &poly->d_min, sizeof(float));
	pos += sizeof(float);
	memcpy(resp + pos, &poly->d_max, sizeof(float));
	pos += sizeof(float);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, resp, pos);
}

/*
 * Write per-(node, channel) correction polynomial.
 * Format: [node_id:u16 LE][channel:u8][degree:u8][coeffs:f32 LE...][d_min:f32][d_max:f32]
 */
static ssize_t write_poly_node(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len,
			       uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len < 4) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *data = buf;
	uint16_t node_id;
	memcpy(&node_id, data, sizeof(uint16_t));
	uint8_t channel = data[2];
	uint8_t degree = data[3];

	if (degree > POLY_CORRECTION_MAX_DEGREE) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	size_t coeff_bytes = ((size_t)degree + 1) * sizeof(float);
	size_t expected = 4 + coeff_bytes + 2 * sizeof(float);
	if (len != expected) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	struct correction_poly poly;
	memset(&poly, 0, sizeof(poly));
	poly.degree = degree;
	memcpy(poly.coeffs, data + 4, coeff_bytes);
	memcpy(&poly.d_min, data + 4 + coeff_bytes, sizeof(float));
	memcpy(&poly.d_max, data + 4 + coeff_bytes + sizeof(float), sizeof(float));

	int ret = poly_correction_set_node(node_id, channel, &poly);
	if (ret < 0) {
		LOG_ERR("Failed to set per-node poly for 0x%04x ch%u: %d",
			node_id, channel, ret);
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	}

	last_poly_node_id = node_id;
	last_poly_channel = channel;
	LOG_INF("Per-node poly via BLE: node=0x%04x ch%u, degree=%u, range=[%.2f, %.2f]",
		node_id, channel, degree, (double)poly.d_min, (double)poly.d_max);

	return len;
}

/* Write-only: clear all per-node correction entries */
static ssize_t write_poly_node_clear(struct bt_conn *conn,
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

	poly_correction_clear_nodes();
	LOG_INF("Per-node correction table cleared via BLE");

	return len;
}

/*
 * Write per-(node, channel) XRLoc correction parameters.
 * Format: [node_id:u16 LE][channel:u8][alpha:f32 LE][beta:f32 LE][gamma:f32 LE]
 *         [lambda_m:f32 LE][d_min:f32 LE][d_max:f32 LE]
 * Total: 2 + 1 + 6*4 = 27 bytes
 */
static ssize_t write_xrloc_node(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len,
				uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	const size_t expected = sizeof(uint16_t) + 1 + 6 * sizeof(float);
	if (len != expected) {
		LOG_ERR("XRLoc write: expected %zu bytes, got %u", expected, len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *data = buf;
	uint16_t node_id;
	memcpy(&node_id, data, sizeof(uint16_t));
	uint8_t channel = data[2];

	struct xrloc_correction x;
	memcpy(&x.alpha,    data + 3,  sizeof(float));
	memcpy(&x.beta,     data + 7,  sizeof(float));
	memcpy(&x.gamma,    data + 11, sizeof(float));
	memcpy(&x.lambda_m, data + 15, sizeof(float));
	memcpy(&x.d_min,    data + 19, sizeof(float));
	memcpy(&x.d_max,    data + 23, sizeof(float));

	int ret = xrloc_correction_set_node(node_id, channel, &x);
	if (ret < 0) {
		LOG_ERR("Failed to set XRLoc for 0x%04x ch%u: %d",
			node_id, channel, ret);
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	}

	LOG_INF("XRLoc via BLE: node=0x%04x, alpha=%.2f, beta=%.4f, "
		"gamma=%.3f, lambda=%.4fm, range=[%.2f, %.2f]",
		node_id, (double)x.alpha, (double)x.beta,
		(double)x.gamma, (double)x.lambda_m,
		(double)x.d_min, (double)x.d_max);

	return len;
}

/* Write-only: clear all per-node XRLoc entries */
static ssize_t write_xrloc_node_clear(struct bt_conn *conn,
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

	xrloc_correction_clear_nodes();
	LOG_INF("XRLoc correction table cleared via BLE");

	return len;
}

/* Service Definition */
BT_GATT_SERVICE_DEFINE(uwb_radio_svc,
	BT_GATT_PRIMARY_SERVICE(&uwb_radio_service_uuid),

	/* TX Antenna Delay Characteristic */
	BT_GATT_CHARACTERISTIC(&tx_ant_delay_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_tx_antenna_delay, write_tx_antenna_delay, NULL),

	/* RX Antenna Delay Characteristic */
	BT_GATT_CHARACTERISTIC(&rx_ant_delay_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_rx_antenna_delay, write_rx_antenna_delay, NULL),

	/* Bias Correction Mode Characteristic */
	BT_GATT_CHARACTERISTIC(&bias_correction_mode_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_bias_correction_mode, write_bias_correction_mode, NULL),

	/* Reject Frames Characteristic */
	BT_GATT_CHARACTERISTIC(&reject_frames_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_reject_frames, write_reject_frames, NULL),

	/* FP Index Threshold Characteristic */
	BT_GATT_CHARACTERISTIC(&fp_index_threshold_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_fp_index_threshold, write_fp_index_threshold, NULL),

	/* Smart TX Power Characteristic */
	BT_GATT_CHARACTERISTIC(&smart_power_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_smart_power, write_smart_power, NULL),

	/* TX Power Characteristic */
	BT_GATT_CHARACTERISTIC(&tx_power_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_tx_power, write_tx_power, NULL),

	/* Polynomial Correction: Global */
	BT_GATT_CHARACTERISTIC(&poly_global_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_poly_global, write_poly_global, NULL),

	/* Polynomial Correction: Per-Node */
	BT_GATT_CHARACTERISTIC(&poly_node_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_poly_node, write_poly_node, NULL),

	/* Polynomial Correction: Clear Per-Node Table */
	BT_GATT_CHARACTERISTIC(&poly_node_clear_uuid.uuid,
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_poly_node_clear, NULL),

	/* XRLoc Correction: Per-Node */
	BT_GATT_CHARACTERISTIC(&xrloc_node_uuid.uuid,
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_xrloc_node, NULL),

	/* XRLoc Correction: Clear Per-Node Table */
	BT_GATT_CHARACTERISTIC(&xrloc_node_clear_uuid.uuid,
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_xrloc_node_clear, NULL),
);

int uwb_radio_service_init(void)
{
	LOG_INF("UWB Radio Settings service initialized");
	return 0;
}
