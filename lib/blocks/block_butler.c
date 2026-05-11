#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/lib/system/hw.h>
#include <app/lib/system/node.h>
#include <app/lib/timesync/time_synchronization.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/ieee802154/timesync_butler.h>
#include <app/lib/blocks/butler.h>
#include <app/lib/blocks/blocks.h>
#include <app/lib/node_table/node_table.h>

LOG_MODULE_REGISTER(block_butler, LOG_LEVEL_INF);

void butler_block_handler(uint64_t event_time, void *user_data)
{
    struct butler_block_config *config = (struct butler_block_config *)user_data;

    if (!config) {
        LOG_ERR("Butler block called with NULL config");
        return;
    }

    /* Ensure offset-only mode (no skew estimation across rounds) */
    if (time_sync_get_mode() != TIME_SYNC_MODE_OFFSET) {
        time_sync_set_mode(TIME_SYNC_MODE_OFFSET);
    }

    /* Disable free-running: Butler provides the sync source.
     * Free-running makes get_current_reference_time() return ref=local,
     * bypassing the Butler offset entirely. */
    if (time_sync_is_free_running()) {
        time_sync_set_free_running(false);
    }

    /* Set channel if configured */
    if (config->channel) {
        const uwb_driver_t *uwb = uwb_driver_get(ieee802154_dev);
        if (uwb && uwb->set_channel) {
            uwb->set_channel(ieee802154_dev, config->channel);
        }
    }

    struct ieee802154_radio_api *__attribute__((unused)) radio_api =
        (struct ieee802154_radio_api *)ieee802154_dev->api;

    /* Mark sync as dirty -- stale until this round completes */
    time_sync_set_dirty();

    struct butler_flood_config flood_conf = {
        .node_addr = get_node_addr(),
        .max_subslots = config->max_subslots,
        .subslot_duration_us = config->subslot_duration_us,
        .guard_period_us = config->guard_period_us,
        .p_tx_pct = config->p_tx_pct,
    };
    butler_store_config(&flood_conf);

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    struct butler_flood_result result;
    int ret = uwb_butler_flood(ieee802154_dev, &flood_conf, &result);

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    if (ret >= 0 && result.converged) {
        /* Feed into existing time_sync_update() -- clears dirty on success */
        struct deca_glossy_result sync_result = {
            .rtc_clock_pair = result.rtc_clock_pair,
            .deca_clock_pair = result.deca_clock_pair,
            .root_node_id = result.sigma_node_id,
            .dist_to_root = 0,
            .payload_size = 0,
            .payload = NULL,
            .measured_constant_delay_us = -1,
        };
        time_sync_update(event_time, sync_result);

        /* Store tau anchors for beacon-relative scheduling and MTM DWT computation.
         * tau_local: local RTC at tau (scheduler anchor)
         * tau_local_dwt: local DWT at tau (MTM DWT anchor)
         * tau_ref_rtc: sigma's RTC at tau (deterministic event_time base) */
        time_sync_set_tau_local(result.rtc_clock_pair.local);
        time_sync_set_tau_local_dwt(result.deca_clock_pair.local);
        time_sync_set_tau_ref_rtc(result.rtc_clock_pair.ref);

        /* Update node table: sigma is the effective "root" this round */
        if (result.sigma_node_id != get_node_addr()) {
            node_table_update_hop_count(result.sigma_node_id, 1, event_time);
            node_table_set_flags(result.sigma_node_id, NODE_TABLE_FLAG_ROOT);
            node_table_notify_changed();
        }
        time_sync_clear_resync();
    }
    /* If !converged: dirty remains set, subsequent needs_sync blocks skip */

    /* Status logging */
    if (ret >= 0) {
        printk("{\"event\": \"butler\", \"node_id\": \"0x%04hx\", "
               "\"sigma\": \"0x%04hx\", \"converged\": %d, \"rtc\": %lld}\n",
               get_node_addr(), result.sigma_node_id,
               result.converged ? 1 : 0, event_time);

        /* Skew estimation from post-convergence beacon.
         * Two actual TX/RX timestamp pairs (no projections):
         *   SYNC:   local RX (sync_local_rx_dwt) / sender TX (sync_sender_tx_dwt)
         *   Beacon: local RX (beacon_dwt) / sigma TX (sigma_beacon_dwt)
         * Both local timestamps are from this node's DWT clock.
         * Both sigma timestamps are from the sigma's DWT clock (carried in frames).
         * skew = (local_delta - sigma_delta) / sigma_delta * 1e6 [ppm]
         */
        if (result.has_beacon && result.sigma_node_id != get_node_addr()) {
            /* Hop correction: our SYNC RX happened hop_count * subslot after
             * sigma's original TX. Subtract the relay delay from our local RX
             * to align it with sigma's TX instant.
             *
             * corrected_local_rx = sync_local_rx - hop_count * subslot
             * local_delta  = beacon_rx - corrected_local_rx
             * sigma_delta  = beacon_tx - sigma_tx
             * Both deltas now span from the same physical instant to the beacon. */
            uint64_t hop_correction_dwt = US_TO_DWT_TS(
                (uint64_t)result.sync_hop_count * config->subslot_duration_us);
            uint64_t corrected_local_rx = (result.sync_local_rx_dwt - hop_correction_dwt) & UWB_TS_MASK;
            int64_t local_delta = (int64_t)((result.beacon_dwt - corrected_local_rx) & UWB_TS_MASK);
            int64_t sigma_delta = (int64_t)((result.sigma_beacon_dwt - result.sync_sigma_tx_dwt) & UWB_TS_MASK);

            int32_t dwt_skew_ppm = (sigma_delta != 0) ?
                (int32_t)(((local_delta - sigma_delta) * 1000000LL) / sigma_delta) : 0;
            printk("{\"event\": \"butler_skew\", \"node_id\": \"0x%04hx\", "
                   "\"sigma\": \"0x%04hx\", \"dwt_ppm\": %d, \"hops\": %u, "
                   "\"local_delta_us\": %lld, \"sigma_delta_us\": %lld}\n",
                   get_node_addr(), result.sigma_node_id, dwt_skew_ppm,
                   result.sync_hop_count,
                   DWT_TS_TO_US(local_delta), DWT_TS_TO_US(sigma_delta));
        }
    } else {
        printk("{\"event\": \"butler\", \"node_id\": \"0x%04hx\", "
               "\"error\": %d, \"rtc\": %lld}\n",
               get_node_addr(), ret, event_time);
    }
}
