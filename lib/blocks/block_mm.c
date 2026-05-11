#include <zephyr/settings/settings.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/ieee802154/uwb_frame_utils.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/lib/system/node.h>
#include <app/lib/system/hw.h>
#include <app/lib/ranging/twr.h>
#include <app/lib/timesync/time_synchronization.h>
#include <app/lib/logging/log.h>

#include <app/lib/blocks/mtm.h>
#include <app/lib/blocks/blocks.h>
#include <app/lib/ranging/digest.h>

#include <stdio.h>

LOG_MODULE_REGISTER(mm_block);

static bool mm_reference_should_participate(struct mm_reference_block_config *config, deca_short_addr_t node_id, bool *isInitiator) {
    const deca_short_addr_t INITIATOR_ADDR = config->initiator_addr;
    const deca_short_addr_t RESPONDER_ADDR = config->responder_addr;

    if (node_id == INITIATOR_ADDR) {
        *isInitiator = true;
        return true;
    } else if (node_id == RESPONDER_ADDR) {
        *isInitiator = false;
        return true;
    }

    return false;
}

void mm_reference_block_handler(uint64_t rtc_event_time, void *user_data) {
    int ret;
    struct mm_reference_block_config *config = (struct mm_reference_block_config *)user_data;
    struct deca_ranging_digest *digest = NULL;
    const uwb_driver_t *uwb_driver = uwb_driver_get(ieee802154_dev);
    const char *exit_reason = NULL;
    int exit_code = 0;
    bool success = false;

    if(get_node_mode() == NODE_MODE_RANGING_DISABLED) {
        exit_reason = "ranging_disabled";
        goto cleanup;
    }

    if (deca_ranging_digest_alloc(10, &digest) < 0) {
        exit_reason = "digest_alloc";
        goto cleanup;
    }

    uint64_t last_successfull_glossy_sync_rtc;
    struct deca_glossy_result sync_result;
    if (get_last_glossy_result(&sync_result, &last_successfull_glossy_sync_rtc) <= 0) {
        exit_reason = "no_glossy_sync";
        goto cleanup;
    }

    deca_short_addr_t node_id = get_node_addr();
    bool isInitiator = false;

    uint8_t ch = config->channel ? config->channel : 5;

    if (!mm_reference_should_participate(config, node_id, &isInitiator)) {
        exit_reason = "not_participant";
        goto cleanup;
    }

    uint64_t reference_round_start_ts;
    ref_rtc_to_deca(rtc_event_time, &reference_round_start_ts);

    uint64_t local_round_start_ts;
    if (get_deca_local_timestamp(reference_round_start_ts + US_TO_DWT_TS(700), &local_round_start_ts) < 0) {
        exit_reason = "ts_convert";
        goto cleanup;
    }

    struct deca_mm_reference_config conf = {
        .addr = node_id,
        .isInitiator = isInitiator,
        .deca_round_start_ts = local_round_start_ts,
        .respond_interval_us = config->respond_interval_us,
        .guard_period_us = config->guard_period_us,
        .timeout_us = config->timeout_us,
    };

    if (uwb_driver && uwb_driver->set_channel) {
        uwb_driver->set_channel(ieee802154_dev, ch);
    }
    if ((ret = deca_mm_reference(ieee802154_dev, &conf, digest)) < 0) {
        exit_reason = "ranging";
        exit_code = ret;
        if (config->mm_ref_cb) {
            config->mm_ref_cb(MTM_STATUS_ERROR, NULL, config->cb_user_data);
        }
        goto cleanup;
    }

    success = true;
    {
        struct mm_reference_block_result result = {
            .digest = digest,
            .config = config,
            .rtc = rtc_event_time,
            .channel = ch,
        };
        if (config->mm_ref_cb) {
            config->mm_ref_cb(MTM_STATUS_SUCCESS, &result, config->cb_user_data);
        }
    }

  cleanup:
    if (success) {
        printk("[mm_ref] ok ch=%u frames=%u\n",
               ch, digest ? digest->length : 0);
    } else {
        printk("[mm_ref] fail reason=%s err=%d\n",
               exit_reason ? exit_reason : "unknown", exit_code);
    }

    if (digest) {
        deca_ranging_digest_free(digest);
    }

    /* Restore channel to default (5) for glossy time synchronization */
    if (uwb_driver && uwb_driver->set_channel) {
        uwb_driver->set_channel(ieee802154_dev, 5);
    }
}
