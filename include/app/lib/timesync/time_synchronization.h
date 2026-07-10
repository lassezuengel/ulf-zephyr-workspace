#ifndef TIME_SYNCHRONIZATION_H
#define TIME_SYNCHRONIZATION_H

#include <stdint.h>
#include <zephyr/kernel.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */

#include <app/lib/blocks/blocks.h>

#define ENOTIMEBASE 150

/** Time synchronization mode */
typedef enum {
    TIME_SYNC_MODE_SKEW,    /**< Accumulates skew corrections, needs >=3 updates */
    TIME_SYNC_MODE_OFFSET,  /**< Single-point offset, valid after 1 update */
} time_sync_mode_t;

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

/**
 * @brief Enable or disable free-running mode
 *
 * In free-running mode, get_current_reference_time() returns local time
 * directly without requiring Glossy synchronization. This allows the
 * block scheduler to run superframes without network time sync
 * (e.g., baseline-only for swarm ranging).
 *
 * Unlike root mode, free-running does NOT cause the node to initiate
 * Glossy floods.
 *
 * @param enable true to enable free-running, false to require Glossy sync
 */
void time_sync_set_free_running(bool enable);

/** @brief Check if free-running mode is active */
bool time_sync_is_free_running(void);

/**
 * @brief Set time synchronization mode
 *
 * SKEW mode accumulates skew corrections over multiple rounds (needs >=3).
 * OFFSET mode uses a single-point offset, valid after 1 update.
 */
void time_sync_set_mode(time_sync_mode_t mode);
time_sync_mode_t time_sync_get_mode(void);

/**
 * @brief Mark time sync data as dirty (stale)
 *
 * Called at the start of each sync round. The dirty flag does NOT invalidate
 * the sync state -- the previous offset remains usable for scheduling.
 * It signals that the data has not been confirmed this round.
 */
void time_sync_set_dirty(void);
void time_sync_clear_dirty(void);
bool time_sync_is_dirty(void);

/**
 * @brief Request immediate resync (e.g., Butler frame received during MTM).
 * The scheduler will run Butler block 0 immediately instead of waiting
 * for the next cycle boundary.
 */
void time_sync_request_resync(void);
bool time_sync_resync_requested(void);
void time_sync_clear_resync(void);

/**
 * @brief Store Butler's tau in local ticks (cycle anchor for beacon-relative scheduling)
 *
 * Called by the Butler block handler after successful convergence.
 * The scheduler reads this via time_sync_get_tau_local() to anchor
 * subsequent blocks at fixed offsets from tau.
 */
void time_sync_set_tau_local(uint64_t tau_local_ticks);
uint64_t time_sync_get_tau_local(void);

void time_sync_set_tau_local_dwt(uint64_t tau_local_dwt);
uint64_t time_sync_get_tau_local_dwt(void);

/** Store sigma's RTC at tau (reference domain, for deterministic event_time) */
void time_sync_set_tau_ref_rtc(uint64_t tau_ref_rtc);
uint64_t time_sync_get_tau_ref_rtc(void);

/**
 * @brief Reset accumulated time synchronization state
 *
 * Clears all clock sync data (RTC, DWT, RTC<->DWT sync states),
 * skew counters, and cached Glossy results. Preserves configuration
 * (root mode, slot duration, free-running flag) and scheduler state.
 *
 * Caller must pause the scheduler before calling this function
 * to avoid races with the event worker.
 */
void time_sync_reset_sync_state(void);

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

/**
 * @brief Schedule work at a local (monotonic) tick time
 *
 * Like schedule_work_at() but takes local ticks directly -- no ref-to-local
 * conversion. Used by beacon-relative scheduler where block timing is
 * anchored to tau in local ticks. The event_time passed to block handlers
 * is set to the current reference time (if available) so blocks like MTM
 * that need ref time still work.
 */
int schedule_work_at_local(uint64_t local_event_start_ticks, uint64_t ref_event_time,
    block_handler_t block_handler, void *block_user_data,
    time_sync_event_finished_callback_t work_callback, void *callback_user_data);

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
