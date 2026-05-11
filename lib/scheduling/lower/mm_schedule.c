#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/lib/scheduling/lower/schedule_functions.h>
#include <app/lib/scheduling/lower/schedule.h>
#include <app/lib/node_table/node_table.h>
#include <app/lib/system/node.h>

#include "hashing.h"

LOG_MODULE_REGISTER(mm_schedule);

int mtm_mm_hashed_schedule_create(const struct device *dev, uint64_t rtc_event_time, int phases,
    int slots_per_phase, uint8_t *payload, size_t payload_size, frame_cir_callback_t cir_callback, struct deca_schedule **schedule)
{
    int ret = 0;
    deca_short_addr_t local_addr = get_node_addr();

    /* Get node count from node_table + potentially self */
    size_t table_count = node_table_get_count();
    bool self_in_table = node_table_exists(local_addr);
    size_t total_nodes = self_in_table ? table_count : table_count + 1;

    if (total_nodes == 0) {
        LOG_ERR("No nodes in table for schedule creation");
        return -EINVAL;
    }

    deca_short_addr_t transmit_slot_map[total_nodes];

    /* Get sorted node IDs from table */
    size_t node_count = node_table_get_node_ids(transmit_slot_map, total_nodes);

    /* Add self if not already in table (insert sorted) */
    if (!self_in_table) {
        size_t insert_pos = node_count;
        for (size_t i = 0; i < node_count; i++) {
            if (local_addr < transmit_slot_map[i]) {
                insert_pos = i;
                break;
            }
        }
        /* Shift elements to make room */
        for (size_t i = node_count; i > insert_pos; i--) {
            transmit_slot_map[i] = transmit_slot_map[i - 1];
        }
        transmit_slot_map[insert_pos] = local_addr;
        node_count++;
    }

    size_t slot_count = (slots_per_phase + 1) * phases;

    seeded_hash_permute_until_index(transmit_slot_map,
        node_count, slots_per_phase, (uint64_t)rtc_event_time);

    if ((ret = deca_schedule_alloc(slot_count, schedule)) < 0) {
        LOG_ERR("Could not create schedule, not enough memory!\n");
        return ret;
    }

    struct deca_schedule *s = *schedule;

    // print out which nodes are assigned for transmission
    for (int k = 0; k < s->slot_count; k++) {
        int slot = k % (slots_per_phase + 1);
        int phase = k / (slots_per_phase + 1);
        int duration;
        // first slot should always upload the tx buffer to the radio
        if (phase > 0) {
            duration = dwt_calculate_slot_duration(dev,
                slots_per_phase, 0, 200);
        } else {
            duration = dwt_calculate_slot_duration(dev,
                0, payload_size, 200);
        }

        if (slot == 0) {
            s->slots[k].type = DENSE_LOAD_TX_BUFFER;
            s->slots[k].meta.payload_size = 0;
            s->slots[k].meta.load_stored_timestamps = true;
            s->slots[k].meta.max_load_timestamps = slots_per_phase;
            s->slots[k].duration_us = duration;
            if (k == 0 && payload != NULL) {
                s->slots[k].meta.payload_size = payload_size;
                s->slots[k].meta.payload = (uint8_t *)payload;
            }
        } else {
            if (get_node_mode() == NODE_MODE_ACTIVE &&
                slot <= node_count &&
                transmit_slot_map[slot - 1] == local_addr) {
                s->slots[k].type = DENSE_TX_SLOT;
            } else {
                s->slots[k].type = DENSE_RX_SLOT;
                s->slots[k].meta.with_cir_handler = true;
                /*
                 * In case that only_first_path is true, the from_index and to_index specifies a
                 * window size of additional samples around first path i.e., to only get fp index
                 * both shall be 0.
                 */
                s->slots[k].meta.only_first_path = true;
                s->slots[k].meta.from_index = 1;
                s->slots[k].meta.to_index = 1;

                /* s->slots[k].meta.only_first_path = false; */
                /* s->slots[k].meta.from_index = 0; */
                /* s->slots[k].meta.to_index = 20; */
            }

            s->slots[k].duration_us = duration;

            // TODO first load and tx slots can be a lot shorter.
        }
    }

    return s->slot_count;
}
