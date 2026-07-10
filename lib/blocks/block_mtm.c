#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include <app/drivers/ieee802154/dw1000.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/ieee802154/uwb_frame_utils.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/lib/system/node.h>
#include <app/lib/system/hw.h>
#include <app/lib/ranging/twr.h>
#include <app/lib/timesync/time_synchronization.h>
#include <app/lib/logging/log.h>

#include <app/lib/scheduling/lower/schedule.h>

#include <app/lib/blocks/mtm.h>
#include <app/lib/management/network_settings.h>
#include <app/lib/blocks/blocks.h>
#include <app/lib/ranging/digest.h>

#ifdef CONFIG_MAC_QUEUE
#include <app/lib/communication/mac_queue.h>
#endif

#include <stdio.h>

/* Set log level for this module */
#undef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INF
LOG_MODULE_REGISTER(mtm_block);

struct mtm_settings {
    bool reject_frames;
    uint16_t fp_index_threshold;
    uint8_t bias_correction_mode; /* bias_correction_mode_t stored as uint8 for settings */
};

static struct mtm_settings settings = {
    .reject_frames = false,
    .fp_index_threshold = 743,
    .bias_correction_mode = BIAS_CORRECTION_TIMESTAMP,
};

void mtm_set_correct_reject_frames(bool enabled) {
    settings.reject_frames = enabled;
    settings_save_one("mtm/reject_frames", &settings.reject_frames,
                    sizeof(settings.reject_frames));
}

void mtm_set_fp_index_threshold(uint16_t threshold) {
    settings.fp_index_threshold = threshold;
    settings_save_one("mtm/fp_threshold", &settings.fp_index_threshold,
                    sizeof(settings.fp_index_threshold));
}

void mtm_set_bias_correction_mode(bias_correction_mode_t mode) {
    settings.bias_correction_mode = (uint8_t)mode;

    settings_save_one("mtm/bias_mode", &settings.bias_correction_mode,
                    sizeof(settings.bias_correction_mode));
}

bias_correction_mode_t mtm_get_bias_correction_mode(void) {
    return (bias_correction_mode_t)settings.bias_correction_mode;
}

bool mtm_get_reject_frames(void) {
    return settings.reject_frames;
}

uint16_t mtm_get_fp_index_threshold(void) {
    return settings.fp_index_threshold;
}

static int settings_set(const char *name, size_t len,
                              settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "reject_frames", &next) && !next) {
        if (len != sizeof(settings.reject_frames)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.reject_frames,
                   sizeof(settings.reject_frames));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "fp_threshold", &next) && !next) {
        if (len != sizeof(settings.fp_index_threshold)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.fp_index_threshold,
                   sizeof(settings.fp_index_threshold));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "bias_mode", &next) && !next) {
        if (len != sizeof(settings.bias_correction_mode)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.bias_correction_mode,
                   sizeof(settings.bias_correction_mode));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    return -ENOENT;
}


void mtm_block_handler(uint64_t rtc_event_time, void *user_data)
{
    struct mtm_block_config *config = (struct mtm_block_config *)user_data;
    const uwb_driver_t *uwb_driver = uwb_driver_get(ieee802154_dev);
    schedule_type_t schedule_type = config->schedule_type;
    bool schedule_allocated = false;
    struct deca_ranging_digest *digest = NULL;
    uint8_t ranging_round_slots_per_phase = config->ranging_round_slots_per_phase;
    uint8_t ranging_round_phases = config->ranging_round_phases;
    uint16_t slot_padding = config->slot_padding_us ? config->slot_padding_us
                            : network_get_slot_padding_us();

    uint64_t reference_round_start_ts;
    uint64_t local_slot_start_ts, reference_slot_start_ts;
    size_t __attribute__((unused)) payload_size = 0;
    uint64_t __attribute__((unused)) ranging_us = 0, __attribute__((unused)) ranging_work_us = 0; // TODO

    if(get_node_mode() == NODE_MODE_RANGING_DISABLED) {
        goto cleanup;
    }

    /* Allocate digest for ranging results */
    if (deca_ranging_digest_alloc(ranging_round_phases * (ranging_round_slots_per_phase + 1), &digest) < 0) {
        LOG_ERR("Failed to allocate digest");
        goto cleanup;
    }

    local_slot_start_ts = uwb_driver->system_timestamp(ieee802154_dev);

    if (get_node_addr() != CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID) {
        if(get_deca_reference_timestamp(local_slot_start_ts, &reference_slot_start_ts) < 0) {
            reference_slot_start_ts = 0;
        }
    } else {
        reference_slot_start_ts = local_slot_start_ts;
    }

    if (time_sync_get_mode() != TIME_SYNC_MODE_OFFSET) {
        ref_rtc_to_deca(rtc_event_time, &reference_round_start_ts);
    }
    /* In offset mode, DWT computation is deferred to below (uses tau_dwt) */

    if (uwb_driver && uwb_driver->set_channel) {
        uwb_driver->set_channel(ieee802154_dev, 5);
    }

    struct deca_schedule *schedule;

    // Handle MAC queue payload integration BEFORE schedule creation
#ifdef CONFIG_MAC_QUEUE
    // Check if we have a pending MAC frame to transmit
    struct mac_queue_frame pending_frame;
    uint8_t extracted_payload[DECA_RANGING_FRAME_MAX_PAYLOAD];
    size_t extracted_len = 0;
    uint8_t *payload_ptr = NULL;

    // Only handle broadcast frames for now (as per design document)
    if (mac_queue_tx_pop(&pending_frame) == 0) {
        if (mac_queue_is_broadcast(&pending_frame)) {
            LOG_DBG("Popped MAC frame for transmission: len=%zu, flags=0x%02x",
                    pending_frame.length, pending_frame.flags);

            // Extract payload portion from the IEEE 802.15.4 frame
            extracted_len = sizeof(extracted_payload);
            int extract_ret = mac_queue_extract_payload(&pending_frame,
                                                       extracted_payload, &extracted_len);

            if (extract_ret == 0 && extracted_len > 0) {
                payload_ptr = extracted_payload;
                LOG_DBG("MAC frame payload extracted: %zu bytes", extracted_len);
            } else {
                LOG_WRN("Failed to extract payload from MAC frame: %d", extract_ret);
                extracted_len = 0;
            }
        } else {
            // Put non-broadcast frame back for now
            // TODO: Support unicast frames in future
            mac_queue_tx_push(&pending_frame, K_NO_WAIT);
            LOG_DBG("Non-broadcast frame returned to queue");
        }
    }
#else
    uint8_t *payload_ptr = NULL;
    size_t extracted_len = 0;
#endif

    // Assign slots based on scheduling type
    int ret = -1;
    switch (schedule_type) {
        case SCHEDULE_BASIC:
            // Basic slot assignment (1:1 mapping)
            LOG_ERR("SCHEDULE_BASIC not implemented\n"); // TODO we could sort ids in the future or something...
            break;

        case SCHEDULE_HASHED:
            // Rotating slot assignment based on ASN
            ret = mtm_hashed_schedule_create(ieee802154_dev, rtc_event_time, ranging_round_phases, ranging_round_slots_per_phase, payload_ptr, extracted_len, slot_padding, &schedule);
            break;

        case SCHEDULE_CONTENTION:
            // Contention scheduling
            ret = mtm_contention_schedule_create(ieee802154_dev, rtc_event_time, ranging_round_phases, ranging_round_slots_per_phase, payload_ptr, extracted_len, slot_padding, &schedule);
            break;

        default:
            LOG_ERR("Unknown schedule type %d\n", schedule_type);
            goto cleanup;
    }

    if (ret < 0) {
        LOG_ERR("Could not allocate schedule. Error: %d\n", ret);
        goto cleanup;
    }

    schedule_allocated = true;

    /* json_log_schedule(&schedule); */
    uint64_t last_successfull_glossy_sync_rtc;
    struct deca_glossy_result sync_result;
    int glossy_result = get_last_glossy_result(&sync_result, &last_successfull_glossy_sync_rtc);
    if (glossy_result > 0) {
        uint64_t local_round_start_ts;
        deca_short_addr_t node_id = get_node_addr();
        int timestamp_result;

        if (time_sync_get_mode() == TIME_SYNC_MODE_OFFSET) {
            /* Beacon mode: compute DWT anchor from tau.
             *
             * tau_ref_dwt (sigma's DWT at tau) is the same on all nodes.
             * tau_local_dwt (this node's DWT at tau) differs per node.
             * deca_offset = tau_local_dwt - tau_ref_dwt converts ref->local.
             *
             * round_start_ref = tau_ref_dwt + time_offset + guard  (same for all nodes)
             * round_start_local = round_start_ref + deca_offset     (per-node conversion)
             */
            uint64_t tau_local_dwt = time_sync_get_tau_local_dwt();
            uint64_t tau_ref_rtc = time_sync_get_tau_ref_rtc();
            if (tau_local_dwt != 0 && tau_ref_rtc != 0 && rtc_event_time >= tau_ref_rtc) {
                /* All nodes anchor directly from their local DWT at tau.
                 * offset_us = time from tau to this block's scheduled start.
                 * All DW1000s run at the same nominal frequency, so
                 * US_TO_DWT_TS(offset) is effectively identical on all nodes. */
                uint64_t offset_us = TICKS_TO_USEC(rtc_event_time - tau_ref_rtc);
                local_round_start_ts = (tau_local_dwt + US_TO_DWT_TS(offset_us + 5000)) & UWB_TS_MASK;
                uint64_t now_dwt = uwb_driver->system_timestamp(ieee802154_dev);
                printk("{\"event\": \"mtm_anchor\", \"node\": \"0x%04hx\", "
                       "\"local_start\": %llu, \"now_dwt\": %llu, "
                       "\"margin_us\": %lld}\n",
                       get_node_addr(), local_round_start_ts, now_dwt,
                       DWT_TS_TO_US((int64_t)local_round_start_ts - (int64_t)now_dwt));
                timestamp_result = 0;
            } else {
                timestamp_result = -ENOTIMEBASE;
            }
        } else {
            /* Glossy mode: existing conversion chain */
            timestamp_result = get_deca_local_timestamp(reference_round_start_ts + US_TO_DWT_TS(700), &local_round_start_ts);
        }

        if (timestamp_result >= 0) {
            struct deca_ranging_configuration conf = {
                .schedule = schedule,
                .addr = node_id,
                .deca_clock_synchronization_instance = &sync_result.deca_clock_pair,
                .deca_round_start_ts  = local_round_start_ts,
                .guard_period_us = 0, // Explicit: no guard period for preamble timeout
                .correct_timestamp_bias = (settings.bias_correction_mode == BIAS_CORRECTION_TIMESTAMP),
                .reject_frames = settings.reject_frames,
                .fp_index_threshold = settings.fp_index_threshold,
                .cfo = true,
            };
            if ((ret = deca_ranging(ieee802154_dev, &conf, digest)) >= 0) {
#ifdef CONFIG_MAC_QUEUE
                // Process received payloads from the digest
                if (digest && digest->length > 0) {
                    for (int i = 0; i < digest->length; i++) {
                        struct deca_ranging_frame_container *container = &digest->frames[i];
                        if (container->frame && container->frame->payload_size > 0) {
                            // Create MAC queue frame from received payload
                            struct mac_queue_frame rx_frame;
                            memset(&rx_frame, 0, sizeof(rx_frame));

                            // Reconstruct IEEE 802.15.4 frame for payload with original sender address
                            int prep_ret = mac_queue_prepare_rx_frame(&rx_frame,
                                container->frame->addr,
                                container->frame->payload, container->frame->payload_size);

                            if (prep_ret == 0) {
                                // Set metadata (timestamp, etc.)
                                rx_frame.metadata = (uint32_t)(rtc_event_time & 0xFFFFFFFF);

                                // Push to RX queue
                                int push_ret = mac_queue_rx_push(&rx_frame);
                                if (push_ret == 0) {
                                    LOG_DBG("Received payload queued: %zu bytes from slot %d",
                                           container->frame->payload_size, i);
                                } else {
                                    LOG_WRN("Failed to queue received payload: %d", push_ret);
                                }
                            } else {
                                LOG_WRN("Failed to prepare RX frame: %d", prep_ret);
                            }
                        }
                    }
                }
#endif

                /* Count received frames */
                int rx_count = 0;
                for (int i = 0; i < digest->length; i++) {
                    if (digest->frames[i].type == DECA_RECEIVED) {
                        rx_count++;
                    }
                }
                printk("{\"event\": \"mtm\", \"node_id\": \"0x%04hx\", "
                       "\"rx\": %d, \"total\": %d, \"rtc\": %lld, "
                       "\"sched_hash\": \"0x%08x\"}\n",
                       get_node_addr(), rx_count, digest->length, rtc_event_time,
                       schedule->schedule_hash);

                struct mtm_block_result result = {
                    .digest = digest,
                    .config = config,
                    .schedule = schedule,
                    .rtc = rtc_event_time
                };

                if(config->mtm_cb) {
                    config->mtm_cb(MTM_STATUS_SUCCESS, &result, config->cb_user_data);
                }
            } else {
                printk("{\"event\": \"mtm\", \"node_id\": \"0x%04hx\", "
                       "\"error\": %d, \"rtc\": %lld, "
                       "\"sched_hash\": \"0x%08x\"}\n",
                       get_node_addr(), ret, rtc_event_time,
                       schedule->schedule_hash);
                if(config->mtm_cb) {
                    config->mtm_cb(MTM_STATUS_ERROR, NULL, config->cb_user_data);
                }
            }
        } else {
            printk("{\"event\": \"timestamp_error\", \"rtc\": %llu, \"code\": %d}\n", rtc_event_time, timestamp_result);
        }
    } else {
        printk("{\"event\": \"sync_error\", \"rtc\": %llu, \"code\": %d}\n", rtc_event_time, glossy_result);
    }

  cleanup:
    if(schedule_allocated) {
        deca_schedule_free(schedule);
    }
    if (digest) {
        deca_ranging_digest_free(digest);
    }
}

// iterate over ranging info frames and print out rx_pacc field from container
// DEBUG: Removed printk statements to prevent JSON logging corruption
int check_scheduling_quality(struct deca_ranging_digest *digest, uint64_t time_since_sync)
{
    // Debug output removed to prevent interference with JSON logging
    for (size_t i = 0; i < digest->length; i++) {
        const struct deca_ranging_frame_container *curr_frame_info = &digest->frames[i];

	if (curr_frame_info->frame == NULL) {
            continue;
	}

        // Processing logic without debug output
        if(curr_frame_info->type == DECA_RECEIVED) {
            // RX frame processing (debug output removed)
        } else {
            // TX frame processing (debug output removed)
        }
    }

    return 0;
}


SETTINGS_STATIC_HANDLER_DEFINE(mtm, "mtm", NULL, settings_set, NULL, NULL);
