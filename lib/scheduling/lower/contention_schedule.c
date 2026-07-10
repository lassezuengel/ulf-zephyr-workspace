#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/hash_function.h>
#include <zephyr/random/random.h>

#include <app/lib/scheduling/lower/schedule.h>
#include <app/lib/scheduling/lower/schedule_functions.h>
#include <app/lib/management/network.h>
#include <app/lib/management/network_settings.h>
#include <app/lib/system/node.h>

#include "hashing.h"

LOG_MODULE_REGISTER(contention_schedule);

// Enable this macro to use proper PRNG instead of hash-based scheduling
#define USE_PRNG_SCHEDULING 1

// Global runtime configurable participation probability (can be modified via OpenOCD)
uint32_t runtime_participation_probability = CONFIG_CONTENTION_PARTICIPATION_PROBABILITY;

int mtm_contention_schedule_create(const struct device *dev, uint64_t rtc_event_time, int phases, int slots_per_phase, uint8_t *payload, size_t payload_size, uint16_t slot_padding_us, struct deca_schedule **schedule) {
    int ret = 0;
    deca_short_addr_t local_addr = get_node_addr();

    const uint32_t PARTICIPATION_PROBABILITY = runtime_participation_probability;

    size_t slot_count = (slots_per_phase + 1) * phases;

#if USE_PRNG_SCHEDULING
    // Use Zephyr's proper PRNG for scheduling decisions
    uint8_t tx_slot = (sys_rand32_get() % slots_per_phase) + 1; // +1 because slot 0 is LOAD_TX_BUFFER
    bool should_participate = (sys_rand32_get() % 100) < PARTICIPATION_PROBABILITY;
#else
    // Legacy hash-based scheduling (may have biases)
    uint64_t hash_input = rtc_event_time + local_addr;
    uint32_t hash_output = sys_hash32((void *)&hash_input, sizeof(hash_input));
    uint8_t tx_slot = (hash_output % slots_per_phase) + 1; // +1 because slot 0 is LOAD_TX_BUFFER

    // Determine if this node should participate in ranging
    uint32_t participation_hash = sys_hash32((void *)&hash_input, sizeof(hash_input));
    bool should_participate = (participation_hash % 100) < PARTICIPATION_PROBABILITY;
#endif

    if((ret = deca_schedule_alloc(slot_count, schedule)) < 0) {
        LOG_ERR("Could not create contention schedule, not enough memory!\n");
        return ret;
    }

    struct deca_schedule *s = *schedule;

    LOG_DBG("Contention schedule (%s): node 0x%04x, tx_slot=%d, participate=%s",
#if USE_PRNG_SCHEDULING
            "PRNG",
#else
            "hash",
#endif
            local_addr, tx_slot, should_participate ? "yes" : "no");

    // Build the schedule
    for (int k = 0; k < s->slot_count; k++) {
        int slot = k % (slots_per_phase + 1);
        int phase = k / (slots_per_phase + 1);
        int duration;

        // Calculate slot duration
        if (phase > 0) {
            duration = dwt_calculate_slot_duration(dev, slots_per_phase, 0, slot_padding_us);
        } else {
            duration = dwt_calculate_slot_duration(dev, 0, payload_size, slot_padding_us);
        }

        if(slot == 0) {
            // First slot in each phase: load TX buffer
            s->slots[k].type = DENSE_LOAD_TX_BUFFER;
            s->slots[k].meta.payload_size = 0;
            s->slots[k].meta.load_stored_timestamps = true;
            s->slots[k].meta.max_load_timestamps = slots_per_phase;
            s->slots[k].duration_us = duration;
            if(k == 0 && payload != NULL) {
                s->slots[k].meta.payload_size = payload_size;
                s->slots[k].meta.payload = (uint8_t *) payload;
            }
        } else {
            // Transmission slots: check if this is our slot and if we should participate
            if (get_node_mode() == NODE_MODE_ACTIVE &&
                should_participate &&
                slot == tx_slot) {
                s->slots[k].type = DENSE_TX_SLOT;
            } else {
                s->slots[k].type = DENSE_RX_SLOT;
            }
            s->slots[k].duration_us = duration;
        }
    }

    return s->slot_count;
}
