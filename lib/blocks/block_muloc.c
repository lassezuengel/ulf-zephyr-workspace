/**
 * @file block_muloc.c
 * @brief MULoc anchor overhearing superframe block handler
 *
 * Follows the pattern of block_mm.c: check node mode, get glossy sync,
 * convert RTC to DWT timestamp, set channel, call driver, invoke callback.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/lib/system/node.h>
#include <app/lib/system/hw.h>
#include <app/lib/timesync/time_synchronization.h>

#include <app/lib/blocks/muloc.h>
#include <app/lib/blocks/blocks.h>

#include <stdio.h>

LOG_MODULE_REGISTER(muloc_block);

void muloc_block_handler(uint64_t rtc_event_time, void *user_data)
{
	struct muloc_block_config *config = (struct muloc_block_config *)user_data;
	const uwb_driver_t *uwb_driver = uwb_driver_get(ieee802154_dev);
	const char *exit_reason = NULL;
	int exit_code = 0;
	bool success = false;

	if (get_node_mode() == NODE_MODE_RANGING_DISABLED) {
		exit_reason = "ranging_disabled";
		goto cleanup;
	}

	uint64_t last_successful_glossy_sync_rtc;
	struct deca_glossy_result sync_result;

	if (get_last_glossy_result(&sync_result, &last_successful_glossy_sync_rtc) <= 0) {
		exit_reason = "no_glossy_sync";
		goto cleanup;
	}

	uint64_t reference_round_start_ts;
	ref_rtc_to_deca(rtc_event_time, &reference_round_start_ts);

	uint64_t local_round_start_ts;

	if (get_deca_local_timestamp(reference_round_start_ts + US_TO_DWT_TS(700),
				     &local_round_start_ts) < 0) {
		exit_reason = "ts_convert";
		goto cleanup;
	}

	uint8_t ch = config->start_channel ? config->start_channel : 1;

	if (uwb_driver && uwb_driver->set_channel) {
		uwb_driver->set_channel(ieee802154_dev, ch);
	}

	/* Allocate round results on stack */
	struct muloc_round_result results[MULOC_MAX_ROUNDS];

	memset(results, 0, sizeof(results));

	struct muloc_ranging_config ranging_conf = {
		.anchor_count = config->anchor_count,
		.anchor_id = config->anchor_id,
		.num_rounds = config->num_rounds,
		.delay_time_us = config->delay_time_us,
		.turn_delay_us = config->delay_time_us, /* Same as inter-anchor delay */
		.rx_timeout_us = 2500,
		.round_start_ts = local_round_start_ts,
		.start_channel = config->start_channel ? config->start_channel : 1,
		.hop_channel = config->hop_channel ? config->hop_channel : 3,
	};

	int ret = deca_muloc_ranging(ieee802154_dev, &ranging_conf,
				     results, MULOC_MAX_ROUNDS);

	if (ret < 0) {
		exit_reason = "ranging";
		exit_code = ret;
		if (config->muloc_cb) {
			config->muloc_cb(MULOC_STATUS_ERROR, NULL, config->cb_user_data);
		}
		goto cleanup;
	}

	success = true;
	{
		struct muloc_block_result result = {
			.rounds = results,
			.round_count = (uint8_t)ret,
			.anchor_id = config->anchor_id,
			.rtc = rtc_event_time,
			.start_channel = ch,
		};

		if (config->muloc_cb) {
			config->muloc_cb(MULOC_STATUS_SUCCESS, &result,
					 config->cb_user_data);
		}
	}

cleanup:
	if (success) {
		printk("[muloc] ok rounds=%d anchor_id=%u\n",
		       ret, config->anchor_id);
	} else {
		printk("[muloc] fail reason=%s err=%d\n",
		       exit_reason ? exit_reason : "unknown", exit_code);
	}

	/* Restore channel to 5 for glossy time synchronization */
	if (uwb_driver && uwb_driver->set_channel) {
		uwb_driver->set_channel(ieee802154_dev, 5);
	}
}
