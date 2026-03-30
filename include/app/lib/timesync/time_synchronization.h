#ifndef TIME_SYNCHRONIZATION_H
#define TIME_SYNCHRONIZATION_H

#include <stdint.h>
#include <zephyr/kernel.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */

#include <app/lib/blocks/blocks.h>

#define ENOTIMEBASE 150

/* Time conversion macros */
#define TICKS_TO_SECONDS(ticks) ((uint64_t)(ticks) / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define TICKS_TO_MSEC(ticks) ((uint64_t)(ticks) * 1000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define TICKS_TO_USEC(ticks) ((uint64_t)(ticks) * 1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MSEC_TO_TICKS(msec) ((uint64_t)(msec) * CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1000)
#define USEC_TO_TICKS(usec) ((uint64_t)(usec) * CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1000000)

// add also type for callback after work has finished
typedef void (*time_sync_event_finished_callback_t)(uint64_t start_ref_rtc, uint64_t end_ref_rtc, void *user_data);

void time_sync_init(uint8_t is_timesync_root, uint32_t slot_duration_ms);
void time_sync_update(uint64_t event_time, struct deca_glossy_result sync_result);
void time_sync_set_slot_duration(uint32_t slot_duration_ms);

/**
 * @brief Dynamically set whether this node is the timesync root
 *
 * Changes the root mode at runtime. When set to root, the node will initiate
 * Glossy floods. When disabled, it will act as a receiver.
 *
 * @param is_root true to enable root mode, false to disable
 */
void time_sync_set_root_mode(bool is_root);
int get_last_glossy_result(struct deca_glossy_result *result, uint64_t *rtc_event_time);
int get_current_reference_time(uint64_t *local_now, uint64_t *ref_now);
int get_deca_local_timestamp(uint64_t reference_timestamp, uint64_t *local_timestamp);
int get_deca_reference_timestamp(uint64_t local_timestamp, uint64_t *reference_timestamp);
uint64_t calculate_deca_slot_ts(uint64_t slot_start_ref_rtc, uint64_t guard);
int ref_rtc_to_deca(uint64_t ref_rtc_timestamp, uint64_t *deca_timestamp);

// big TODO i am not happy at all yet with how the schedule next events
int slotted_schedule_work_next_slot(block_handler_t block_handler,
    void *block_user_data, time_sync_event_finished_callback_t event_finished_callback, void *callback_user_data);
int schedule_work_at(uint64_t ref_event_start_ts, block_handler_t block_handler,
    void *block_user_data, time_sync_event_finished_callback_t work_callback, void *callback_user_data);

int slotted_schedule_get_asn();

/**
 * @brief Pause the time sync scheduler
 *
 * Stops the event timer and prevents new scheduled work from executing.
 * Use this to temporarily halt UWB activity (e.g., during DFU uploads).
 * Call time_sync_scheduler_resume() to restore normal operation.
 */
void time_sync_scheduler_pause(void);

/**
 * @brief Resume the time sync scheduler
 *
 * Re-enables the scheduler after a pause. Note that any work that was
 * scheduled before the pause will not automatically resume - the block
 * scheduler will need to reschedule work on the next cycle.
 */
void time_sync_scheduler_resume(void);

/**
 * @brief Check if the scheduler is currently paused
 *
 * @return true if paused, false if running normally
 */
bool time_sync_scheduler_is_paused(void);


#endif
