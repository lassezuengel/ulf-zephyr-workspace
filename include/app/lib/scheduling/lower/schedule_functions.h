#ifndef SCHEDULING_FUNCTION_H
#define SCHEDULING_FUNCTION_H

#include <zephyr/device.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>

typedef int (*frame_cir_callback_t)(int slot, const uint8_t *cir_memory, size_t size);

int mtm_hashed_schedule_create(const struct device *dev, uint64_t rtc_event_time, int phases, int slots_per_phase, uint8_t *payload, size_t payload_size, struct deca_schedule **schedule);
int mtm_contention_schedule_create(const struct device *dev, uint64_t rtc_event_time, int phases, int slots_per_phase, uint8_t *payload, size_t payload_size, struct deca_schedule **schedule);
int mtm_mm_hashed_schedule_create(const struct device *dev, uint64_t rtc_event_time, int phases,
    int slots_per_phase, uint8_t *payload, size_t payload_size, frame_cir_callback_t cir_callback, struct deca_schedule **schedule);

#endif
