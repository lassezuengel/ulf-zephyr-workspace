#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app/lib/logging/log.h>

#include <app/lib/system/node.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>

#include <math.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(logging);

// Struct for distance unit info
struct distance_unit_info {
    enum distance_unit unit;
    const char* unit_string;
    int conversion_factor;
};

// Array of distance unit info
static const struct distance_unit_info distance_units[] = {
    {DIST_MM, "mm", 1000},
    {DIST_CM, "cm", 100},
    {DIST_M, "m", 1},
    {DIST_UNKNOWN, "unknown", 1}
};

const struct distance_unit_info* get_distance_unit_info(enum distance_unit unit) {
    for (int i = 0; i < sizeof(distance_units) / sizeof(distance_units[0]); ++i) {
        if (distance_units[i].unit == unit) {
            return &distance_units[i];
        }
    }
    return &distance_units[sizeof(distance_units) / sizeof(distance_units[0]) - 1];
}

void log_schedule_json(const struct deca_schedule *schedule, int rtc) {
    printk("{\"event\":\"schedule\",\"local_id\":\"0x%x\",\"rtc\":%u,\"slots\":[", get_node_addr(), rtc);
    for(int slot = 0; slot < schedule->slot_count; slot++) {
        if (slot > 0) {
            printk(",");
        }
        printk("%c", "LRTI"[schedule->slots[slot].type]);
    }
    printk("]}\n");
}

void log_schedule_transmission_slots_json(const struct deca_schedule *schedule, int rtc) {
    printk("{\"event\":\"tx_slots\",\"local_id\":\"0x%x\",\"rtc\":%u,\"slots\":[", get_node_addr(), rtc);
    for(int slot = 0, comma = 0; slot < schedule->slot_count; slot++) {
        if (schedule->slots[slot].type == DENSE_TX_SLOT) {
            if(comma) {
                printk(",");
            }
            printk("%u", slot);
            comma = 1;
        }
    }
    printk("]}\n");
}

void log_measurements_json(const struct measurement *measurements,
    int tof_count, uint64_t rtc, enum distance_unit unit, bool output_local_only, const char *additional) {

    if (!IS_ENABLED(CONFIG_SYNCHROFLY_LOG_TWR_JSON)) {
        return;
    }

    deca_short_addr_t local_node_id = get_node_addr();
    printk("{\"event\":\"twr\",\"local_id\":\"0x%x\",\"rtc_round_ts\":%llu,\"unit\":\"%s\",\"t\":[",
        local_node_id, rtc, get_distance_unit_info(unit)->unit_string);

    for (int i = 0, comma = 0; i < tof_count; i++) {
        deca_short_addr_t initiator_id = measurements[i].ranging_initiator_id;
        deca_short_addr_t responder_id = measurements[i].ranging_responder_id;
        int quality = measurements[i].tof_quality;
	float dist =
		time_to_dist(measurements[i].tof) * get_distance_unit_info(unit)->conversion_factor;

	if (output_local_only && initiator_id != local_node_id) {
            continue;
        }
        if(comma) {
            printk(",");
        }
        printk("{\"i\":[\"0x%x\",\"0x%x\"],\"d\":%d,\"q\":%d}", initiator_id, responder_id, (int32_t)(dist), (int) quality);

        comma = 1;
    } printk("]");

    if (additional != NULL) {
        printk(", %s", additional);
    }

    printk("}\n");

    int have_tdoa = 0;
    for (int i = 0; i < tof_count; i++) {
        if (measurements[i].ranging_initiator_id != local_node_id &&
            measurements[i].ranging_responder_id != local_node_id &&
            !isnan(measurements[i].tdoa)) {
            have_tdoa = 1;
            break;
        }
    }

    if(have_tdoa) {
        printk("{\"event\":\"td\",\"local_id\":\"0x%x\",\"rtc_round_ts\": %llu,\"unit\":\"%s\",\"t\":[", local_node_id, rtc, get_distance_unit_info(unit)->unit_string);

        for (int i = 0, c = 0; i < tof_count; i++) {
            deca_short_addr_t initiator = measurements[i].ranging_responder_id, responder = measurements[i].ranging_initiator_id;

            if(initiator != local_node_id && responder != local_node_id && !isnan(measurements[i].tdoa)) {
                float dist = time_to_dist(measurements[i].tdoa) * get_distance_unit_info(unit)->conversion_factor;

                /* It might seem confusing that we also here just use a tuple, since a tdoa measurement
                 * will include 3 nodes (initiator, responder and passive receiver).  However, since we
                 * never calculate tdoa values for other node pairs, we can always assume that the third
                 * entry will be our local id, so we omit that one when outputting */
		printk("%c{\"i\":[\"0x%x\",\"0x%x\"],\"d\":%d,\"q\":%d}", c ? ',' : ' ', initiator, responder,
		       (int32_t)(dist), 100);
                c = 1;
            }
        } printk("]");

        if (additional != NULL) {
            printk(", %s", additional);
        }

        printk("}\n");
    }
}

void log_frames_json(const struct deca_ranging_frame_container *frame_infos,
    int frame_info_count, const struct deca_ranging_configuration *conf, uint64_t rtc, uint8_t print_rx_ts, const char *additional)
{
    for (int i = 0; i < frame_info_count; i++) { // the last phase/phase is not of importance for us
        const struct deca_ranging_frame_container *curr_frame_info = &frame_infos[i];

        if (curr_frame_info->frame != NULL) {
            const struct deca_ranging_frame *curr_frame = curr_frame_info->frame;

            if (curr_frame_info->type == DECA_TRANSMITTED) {
                printk("{\"event\": \"tx\", \"tx_id\": %u, \"rtc_round_ts\": %llu, \"slot\": %u, \"ts\": %llu",
                    conf->addr, rtc, curr_frame_info->slot, curr_frame_info->timestamp);

                // Add TX timestamp output for transmitted frames (timestamps embedded in the frame)
                dwt_ts_t frame_tx_ts = from_packed_uwb_ts(curr_frame->tx_ts);
                printk(", \"frame_tx_ts\": %llu", frame_tx_ts);
            } else {
                printk("{\"event\": \"rx\", \"rx_id\": %u, \"tx_id\": %u,"
                    "\"rtc_round_ts\": %llu, \"slot\": %u,"
                    "\"fp_ampl1\": %u, \"fp_ampl2\": %u, \"fp_ampl3\": %u, \"std_noise\": %u,"
                    "\"fp_index\": %u, \"cir_pwr\": %u, \"cfo\": %d, \"rx_pacc\": %u,"
                    "\"rx_level\": %d, \"ts\": %llu",
                    conf->addr, curr_frame->addr, rtc, curr_frame_info->slot,
                    curr_frame_info->fp_ampl1, curr_frame_info->fp_ampl2,
                    curr_frame_info->fp_ampl3, curr_frame_info->std_noise, curr_frame_info->fp_index >> 6,
                    curr_frame_info->cir_pwr, (int)(curr_frame_info->cfo_ppm * 1e6f),
                    curr_frame_info->rx_pacc, curr_frame_info->rx_level, curr_frame_info->timestamp);
	    }

	    if (print_rx_ts) {
		printk(", \"rx_ts\": [");
                struct deca_tagged_timestamp *frame_timestamps = NULL;
                int rx_ts_count;

                rx_ts_count = deca_ranging_frame_get_tagged_timestamps(curr_frame, &frame_timestamps);

		for (int k = 0; k < rx_ts_count; k++) {
                    dwt_ts_t timestamp_value = from_packed_uwb_ts(frame_timestamps[k].ts);

                    printk("{\"addr\": %u, \"slot\": %u, \"ts\": %llu}",
                           frame_timestamps[k].addr, frame_timestamps[k].slot, timestamp_value);
                    if(k < rx_ts_count - 1) {
                        printk(", ");
                    }
		}
                printk("]");
            }

            if (additional != NULL) {
                printk(", %s", additional);
            }

            printk("}\n");
        }
    }
}
