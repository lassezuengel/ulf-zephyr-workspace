/**
 * @file bt_peripheral.c
 * @brief SynchroFly Bluetooth LE Peripheral Implementation
 *
 * Provides BLE connectivity for configuration and diagnostics.
 * Uses Zephyr init system to start advertising automatically when Bluetooth is enabled.
 *
 * When CONFIG_MCUMGR_TRANSPORT_BT is enabled, also advertises SMP service UUID
 * for Device Firmware Update (DFU) over Bluetooth LE.
 *
 * Bluetooth Status LED (if enabled):
 * - OFF: Bluetooth disabled/not initialized
 * - Slow blink (1Hz): Advertising, waiting for connection
 * - ON: Connected
 */

#include "bt_peripheral.h"
#include "bt_services.h"
#include "services/node_config_service.h"
#include "services/neighbor_table_service.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include <app/lib/system/node.h>
#include <app/lib/timesync/time_synchronization.h>

#ifdef CONFIG_MCUMGR_TRANSPORT_BT
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#endif

#if defined(CONFIG_MCUMGR_GRP_IMG_STATUS_HOOKS) || defined(CONFIG_MCUMGR_GRP_IMG_UPLOAD_CHECK_HOOK)
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#endif

#ifdef CONFIG_LOG_BACKEND_BLE
#include <zephyr/logging/log_backend_ble.h>
#endif

LOG_MODULE_REGISTER(synchrofly_bt, CONFIG_LOG_DEFAULT_LEVEL);

/* Bluetooth status LED from device tree (optional) */
#ifdef CONFIG_SYNCHROFLY_BT_STATUS_LED
static const struct gpio_dt_spec bt_status_led =
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(bt_status_led), gpios, {0});
#endif

/* Device name buffer: "<PREFIX>-XXXX" where XXXX is the node ID (IoT-LAB UID) in hex */
#define DEVICE_NAME_MAX_LEN 32
static char device_name[DEVICE_NAME_MAX_LEN];

/* Hardware device types for BLE advertisement identification */
#define SYNCHROFLY_DEVICE_TYPE_UNKNOWN        0
#define SYNCHROFLY_DEVICE_TYPE_DWM1001_ANCHOR 1
#define SYNCHROFLY_DEVICE_TYPE_UWB_DECK       2

/* Manufacturer data structure for SynchroFly advertisement
 * Total size: 10 bytes (2 + 1 + 1 + 1 + 4 + 1)
 * Fits within BLE scan response packet limits (31 bytes total)
 */
struct synchrofly_mfg_data {
	uint16_t company_id;     /* 0xFFFF for internal/test use */
	int8_t pos_x_dm;         /* Position X in decimeters (range: ±12.7m) */
	int8_t pos_y_dm;         /* Position Y in decimeters (range: ±12.7m) */
	int8_t pos_z_dm;         /* Position Z in decimeters (range: ±12.7m) */
	int32_t rtc_timestamp;   /* Synchronized RTC time in ms, or -1 if no time base */
	uint8_t device_type;     /* Hardware type: 0=unknown, 1=dwm1001_anchor, 2=uwb_deck */
} __packed;

static struct synchrofly_mfg_data mfg_data = {
	.company_id = 0xFFFF,
	.pos_x_dm = 0,
	.pos_y_dm = 0,
	.pos_z_dm = 0,
	.rtc_timestamp = -1,  /* No time base initially */
#ifdef CONFIG_CRAZYFLIE
	.device_type = SYNCHROFLY_DEVICE_TYPE_UWB_DECK,
#else
	.device_type = SYNCHROFLY_DEVICE_TYPE_DWM1001_ANCHOR,
#endif
};

/* Advertisement data
 * When DFU is enabled, include SMP service UUID for MCUmgr discovery
 * Note: To fit within 31-byte advertising packet limit, we only advertise one UUID.
 * Additional UUIDs (like NUS for logging) and manufacturer data are in scan response.
 */
#ifdef CONFIG_MCUMGR_TRANSPORT_BT
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, SMP_BT_SVC_UUID_VAL),
};
#elif defined(CONFIG_LOG_BACKEND_BLE)
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, LOGGER_BACKEND_BLE_ADV_UUID_DATA),
};
#else
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};
#endif

/* Scan response data - includes device name and manufacturer data
 * Manufacturer data is in scan response to fit within advertisement packet size limits
 * Note: When both DFU and BLE logging are enabled, we drop the NUS UUID from scan response
 *       to stay within 31-byte limit (name ~18 + mfg data 10 + UUID 18 = 46 bytes > 31)
 *       BLE logging can still work through GATT service discovery
 */
static struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name, 0),  /* Length set at runtime */
	BT_DATA(BT_DATA_MANUFACTURER_DATA, (uint8_t *)&mfg_data, sizeof(mfg_data)),
};

/* Connection state */
static struct bt_conn *current_conn = NULL;
static bool is_connected = false;

/* LED blink work for advertising state */
#ifdef CONFIG_SYNCHROFLY_BT_STATUS_LED
static struct k_work_delayable led_blink_work;
#endif
static struct k_work_delayable adv_restart_work;
static struct k_work_delayable mfg_data_update_work;

#ifdef CONFIG_SYNCHROFLY_BT_STATUS_LED
static void led_blink_fn(struct k_work *work)
{
	if (!bt_status_led.port) {
		return;
	}

	/* Only blink when advertising (not connected) */
	if (!is_connected) {
		gpio_pin_toggle_dt(&bt_status_led);
		/* Schedule next toggle in 500ms for 1Hz blink */
		k_work_schedule(&led_blink_work, K_MSEC(500));
	}
}
#endif

static void adv_restart_fn(struct k_work *work)
{
	int err;

	/* Restart advertising */
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
	                      sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to restart (err %d)", err);
		/* Retry after 1 second if it failed */
		k_work_schedule(&adv_restart_work, K_SECONDS(1));
	} else {
		LOG_INF("Advertising restarted successfully");
	}
}

static void mfg_data_update_fn(struct k_work *work)
{
	int err;

	/* Get current node configured position */
	struct node_config_position pos;
	if (get_node_config_position(&pos) == 0) {
		/* Convert from meters (float) to decimeters (int8_t)
		 * Clamp to ±12.7 meters to fit in int8_t range
		 */
		float x_dm = (float)(pos.x * 10.0);
		float y_dm = (float)(pos.y * 10.0);
		float z_dm = (float)(pos.z * 10.0);

		mfg_data.pos_x_dm = (int8_t)(x_dm > 127.0f ? 127 : (x_dm < -128.0f ? -128 : x_dm));
		mfg_data.pos_y_dm = (int8_t)(y_dm > 127.0f ? 127 : (y_dm < -128.0f ? -128 : y_dm));
		mfg_data.pos_z_dm = (int8_t)(z_dm > 127.0f ? 127 : (z_dm < -128.0f ? -128 : z_dm));
	} else {
		/* No position configured - keep at 0,0,0 */
		mfg_data.pos_x_dm = 0;
		mfg_data.pos_y_dm = 0;
		mfg_data.pos_z_dm = 0;
	}

	/* Get synchronized RTC timestamp in milliseconds, or -1 if no time base */
	uint64_t ref_time_ticks;
	int ret = get_current_reference_time(NULL, &ref_time_ticks);
	if (ret == 0) {
		/* We have a time base - convert ticks to milliseconds */
		mfg_data.rtc_timestamp = (int32_t)TICKS_TO_MSEC(ref_time_ticks);
	} else {
		/* No time base available */
		mfg_data.rtc_timestamp = -1;
	}

	/* Update advertisement data without restarting advertising
	 * Only update when actively advertising (not when connected)
	 */
	if (!is_connected) {
		err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (err) {
			LOG_WRN("Failed to update advertisement data (err %d)", err);
		}
	}

	/* Schedule next update in 1 second */
	k_work_schedule(&mfg_data_update_work, K_SECONDS(1));
}

/* Connection callbacks */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Connection failed: %s (err 0x%02x)", addr, err);
		/* Continue advertising/blinking */
	} else {
		LOG_INF("Connected: %s", addr);

		/* Store connection reference */
		current_conn = bt_conn_ref(conn);
		is_connected = true;

		/* Stop manufacturer data updates while connected to prevent
		 * interference with GATT operations
		 */
		k_work_cancel_delayable(&mfg_data_update_work);

#ifdef CONFIG_SYNCHROFLY_BT_STATUS_LED
		/* Stop blinking and turn LED ON solid */
		k_work_cancel_delayable(&led_blink_work);
		if (bt_status_led.port) {
			gpio_pin_set_dt(&bt_status_led, 1);
		}
#endif
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	/* Release connection reference */
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	is_connected = false;

	/* Resume manufacturer data updates after disconnect */
	k_work_schedule(&mfg_data_update_work, K_SECONDS(1));

#ifdef CONFIG_SYNCHROFLY_BT_STATUS_LED
	/* Resume blinking for advertising */
	k_work_schedule(&led_blink_work, K_NO_WAIT);
#endif

	/* Schedule advertising restart after 200ms delay
	 * This gives the BLE stack time to clean up connection resources
	 * Don't restart from ISR context - use work queue
	 */
	k_work_schedule(&adv_restart_work, K_MSEC(200));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* MTU update callback for GATT */
static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_INF("MTU updated: TX %d, RX %d bytes", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};

/* Public API implementations */
bool synchrofly_bt_is_connected(void)
{
	return is_connected;
}

const char *synchrofly_bt_get_device_name(void)
{
	return device_name;
}

struct bt_conn *synchrofly_bt_get_conn(void)
{
	return current_conn;
}

#ifdef CONFIG_LOG_BACKEND_BLE
/**
 * @brief Hook callback for BLE logging backend status changes
 * @details Called when a client subscribes/unsubscribes to the NUS TX characteristic
 * @param backend_status True if backend is enabled (subscribed), false if disabled
 * @param ctx User context (unused)
 */
static void ble_log_backend_hook(bool backend_status, void *ctx)
{
	ARG_UNUSED(ctx);
	LOG_INF("BLE log backend %s", backend_status ? "enabled" : "disabled");
}
#endif

#if defined(CONFIG_MCUMGR_GRP_IMG_STATUS_HOOKS) || defined(CONFIG_MCUMGR_GRP_IMG_UPLOAD_CHECK_HOOK)
/* Track whether UWB scheduler is paused for DFU to avoid redundant pause calls */
static bool dfu_uwb_paused;

/**
 * @brief MCUmgr DFU status callback
 * @details Pauses/resumes UWB scheduler during firmware upload to improve BLE reliability.
 *          Hooks into DFU_CHUNK (fires on every chunk) to pause UWB as early as possible,
 *          and DFU_STOPPED to resume when upload completes or is aborted.
 */
static enum mgmt_cb_return dfu_status_callback(uint32_t event,
					       enum mgmt_cb_return prev_status,
					       int32_t *rc, uint16_t *group,
					       bool *abort_more, void *data,
					       size_t data_size)
{
	ARG_UNUSED(prev_status);
	ARG_UNUSED(rc);
	ARG_UNUSED(group);
	ARG_UNUSED(abort_more);
	ARG_UNUSED(data);
	ARG_UNUSED(data_size);

	if (event == MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK ||
	    event == MGMT_EVT_OP_IMG_MGMT_DFU_STARTED) {
		/* Pause UWB on first chunk - DFU_CHUNK fires before DFU_STARTED */
		if (!dfu_uwb_paused) {
			LOG_INF("DFU chunk received - pausing UWB scheduler");
			time_sync_scheduler_pause();
			dfu_uwb_paused = true;
		}
	} else if (event == MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED) {
		if (dfu_uwb_paused) {
			LOG_INF("DFU stopped - resuming UWB scheduler");
			time_sync_scheduler_resume();
			dfu_uwb_paused = false;
		}
	}

	return MGMT_CB_OK;
}

static struct mgmt_callback dfu_callback = {
	.callback = dfu_status_callback,
	.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK |
		    MGMT_EVT_OP_IMG_MGMT_DFU_STARTED |
		    MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED,
};
#endif

/* Bluetooth initialization function
 * Called automatically via SYS_INIT when Bluetooth is enabled in config
 */
static int synchrofly_bt_init(void)
{
	int err;
	bt_addr_le_t addr;
	size_t name_len;

	LOG_INF("Initializing Bluetooth subsystem");

#ifdef CONFIG_SYNCHROFLY_BT_STATUS_LED
	/* Initialize LED if available */
	if (bt_status_led.port) {
		if (!gpio_is_ready_dt(&bt_status_led)) {
			LOG_ERR("Bluetooth status LED GPIO not ready");
		} else {
			err = gpio_pin_configure_dt(&bt_status_led, GPIO_OUTPUT_INACTIVE);
			if (err) {
				LOG_ERR("Failed to configure Bluetooth status LED (err %d)", err);
			} else {
				LOG_INF("Bluetooth status LED configured on %s pin %d",
				        bt_status_led.port->name, bt_status_led.pin);
			}
		}
	} else {
		LOG_WRN("No Bluetooth status LED defined in device tree");
	}

	/* Initialize LED blink work */
	k_work_init_delayable(&led_blink_work, led_blink_fn);
#endif

	/* Initialize advertising restart work */
	k_work_init_delayable(&adv_restart_work, adv_restart_fn);

	/* Initialize manufacturer data update work */
	k_work_init_delayable(&mfg_data_update_work, mfg_data_update_fn);

	/* Enable Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	LOG_INF("Bluetooth initialized");

	/* Get device address (needed for other purposes) */
	bt_id_get(&addr, NULL);

	/* Format: "<PREFIX>-XXXX" where XXXX is the node ID (matching IoT-LAB UID) */
	deca_short_addr_t node_id = get_node_addr();
	snprintf(device_name, sizeof(device_name), "%s-%04X",
	         CONFIG_SYNCHROFLY_BT_DEVICE_NAME_PREFIX, node_id);

	/* Update scan response data length */
	name_len = strlen(device_name);
	sd[0].data_len = name_len;

	LOG_INF("Device name set to: %s", device_name);

#ifdef CONFIG_LOG_BACKEND_BLE
	/* Register hook for BLE logging backend status */
	logger_backend_ble_set_hook(ble_log_backend_hook, NULL);
	LOG_INF("BLE logging backend hook registered");
#endif

#if defined(CONFIG_MCUMGR_GRP_IMG_STATUS_HOOKS) || defined(CONFIG_MCUMGR_GRP_IMG_UPLOAD_CHECK_HOOK)
	/* Register MCUmgr DFU status callback to pause UWB during firmware upload */
	mgmt_callback_register(&dfu_callback);
	LOG_INF("MCUmgr DFU status callback registered");
#endif

	/* Initialize service registry (for custom GATT services) */
	err = synchrofly_bt_services_init();
	if (err) {
		LOG_ERR("Service registry init failed (err %d)", err);
		return err;
	}

	/* Initialize custom service callbacks (notification support, etc.) */
	err = node_config_service_init();
	if (err) {
		LOG_ERR("Node config service init failed (err %d)", err);
		return err;
	}

	err = neighbor_table_service_init();
	if (err) {
		LOG_ERR("Neighbor table service init failed (err %d)", err);
		return err;
	}

	/* Register GATT callbacks */
	bt_gatt_cb_register(&gatt_callbacks);

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
	                      sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}

	LOG_INF("Advertising started as '%s'", device_name);

#ifdef CONFIG_SYNCHROFLY_BT_STATUS_LED
	/* Start LED blinking to indicate advertising */
	if (bt_status_led.port) {
		k_work_schedule(&led_blink_work, K_NO_WAIT);
	}
#endif

	/* Start manufacturer data updates (every 1 second) */
	k_work_schedule(&mfg_data_update_work, K_SECONDS(1));

	return 0;
}

/* Register init function to run during APPLICATION init phase
 * Priority CONFIG_SYNCHROFLY_BT_INIT_PRIORITY (default 91) ensures it runs AFTER
 * MCUmgr handlers are initialized (priority 90).
 * This is critical for DFU builds where SMP service must be registered before
 * we call bt_enable() and start advertising.
 */
SYS_INIT(synchrofly_bt_init, APPLICATION, CONFIG_SYNCHROFLY_BT_INIT_PRIORITY);
