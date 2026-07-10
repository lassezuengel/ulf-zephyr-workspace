#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/lib/scheduling/lower/schedule.h>
#include <app/lib/system/block_heap.h>

#include "hashing.h"

LOG_MODULE_REGISTER(schedule);

int deca_schedule_alloc(int slot_count, struct deca_schedule **schedule) {
    if (slot_count <= 0 || schedule == NULL) {
        return -EINVAL;
    }

    // Allocate the schedule structure
    *schedule = block_malloc(sizeof(struct deca_schedule));
    if (*schedule == NULL) {
        LOG_ERR("Failed to allocate memory for schedule structure");
        return -ENOMEM;
    }

    // Allocate memory for slot array
    (*schedule)->slots = block_malloc(slot_count * sizeof(struct deca_slot));
    if ((*schedule)->slots == NULL) {
        LOG_ERR("Failed to allocate memory for slots array");
        block_free(*schedule);
        *schedule = NULL;
        return -ENOMEM;
    }

    // Initialize the allocated schedule
    (*schedule)->slot_count = slot_count;
    memset((*schedule)->slots, 0, slot_count * sizeof(struct deca_slot));

    return 0;
}

int deca_schedule_free(struct deca_schedule *schedule) {
    if (schedule == NULL) {
        return -EINVAL;
    }

    if (schedule->slots != NULL) {
        block_free(schedule->slots);
    }

    block_free(schedule);
    return 0;
}
