#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/net/ieee802154_radio.h>

#include <app/lib/timesync/time_synchronization.h>
#include <app/lib/scheduling/lower/schedule_functions.h>
#include <app/lib/scheduling/upper/block_scheduler.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>

#include "cyclic_timeutil.h"

#include <app/lib/system/radio_arbiter.h>

LOG_MODULE_REGISTER(time_synchronization, CONFIG_TIME_SYNCHRONIZATION_LOG_LEVEL);

#define RANGING_TIMESYNC_WORK_QUEUE_STACK_SIZE CONFIG_SYNCHROFLY_TIMESYNC_WORK_QUEUE_STACK_SIZE

static void event_worker(struct k_work *work_item);

static struct k_work_q time_sync_work_queue;
static K_KERNEL_STACK_DEFINE(time_sync_work_queue_stack,
			     RANGING_TIMESYNC_WORK_QUEUE_STACK_SIZE);

struct  time_sync_work_item {
    struct k_work work;
    block_handler_t block_handler;
    uint64_t work_start_ref_rtc;
    time_sync_event_finished_callback_t event_finished_callback;
    void *block_user_data;
    void *callback_user_data;
};

struct timeutil_sync_config rtc_sync_conf = {
    .ref_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC,
    .local_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC
};

#define UWB_CLOCK_TICKS_PER_SEC ((uint64_t) 499200000 * 128)
struct cyclic_timeutil_sync_config deca_sync_conf = {
	.ref_Hz = UWB_CLOCK_TICKS_PER_SEC,
	.local_Hz = UWB_CLOCK_TICKS_PER_SEC,
	.ref_wrap = 0xFFFFFFFFFF,
        .local_wrap = 0xFFFFFFFFFF,
};

struct cyclic_timeutil_sync_config rtc_deca_sync_conf = {
	.ref_Hz = UWB_CLOCK_TICKS_PER_SEC,
	.local_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC,
	.ref_wrap = 0xFFFFFFFFFF, // this clock does not wrap
        .local_wrap = 0,
};

static struct time_sync_state {
    uint64_t next_event_time;
    uint32_t rtc_timesync_skew_update_count;
    uint32_t deca_timesync_skew_update_count;
    uint8_t is_timesync_root;
    uint32_t systick_slot_duration;
    uint64_t decatick_slot_duration;
    struct timeutil_sync_state rtc_clock_sync_state;
    struct cyclic_timeutil_sync_state deca_clock_sync_state;
    struct cyclic_timeutil_sync_state rtc_deca_clock_sync_state;
    uint64_t last_glossy_event_time;

    struct time_sync_work_item queued_work_item;

    struct deca_glossy_result last_glossy_result;
    bool has_glossy_result;
    bool scheduler_paused;

    /* Saved context for restarting after pause */
    time_sync_event_finished_callback_t saved_callback;
    void *saved_callback_user_data;

    bool free_running;  /* Local time = reference time, no Glossy needed */

    time_sync_mode_t sync_mode;  /* SKEW (Glossy) or OFFSET (Butler) */
    bool sync_dirty;             /* True if sync not confirmed this round */
    uint64_t tau_local_ticks;    /* Butler's tau in local RTC ticks (beacon anchor) */
    uint64_t tau_local_dwt;      /* Butler's tau in local DWT ticks */
    uint64_t tau_ref_rtc;        /* Sigma's RTC at tau (reference domain) */

} state = {
	.next_event_time = 0,
	.rtc_timesync_skew_update_count = 0,
	.deca_timesync_skew_update_count = 0,
	.is_timesync_root = 0,
	.systick_slot_duration = 0,
	.rtc_clock_sync_state = {.cfg = &rtc_sync_conf },
        .deca_clock_sync_state = {.cfg = &deca_sync_conf },
        .rtc_deca_clock_sync_state = {.cfg = &rtc_deca_sync_conf },
	.has_glossy_result = false,
	.scheduler_paused = false,
	.saved_callback = NULL,
	.saved_callback_user_data = NULL,
	.free_running = false,
	.sync_mode = TIME_SYNC_MODE_SKEW,
	.sync_dirty = false,
	.tau_local_ticks = 0,
	.tau_local_dwt = 0,
	.tau_ref_rtc = 0,
};

void time_sync_event_handler()
{
    /* k_work_submit(&state.qubeued_work_item.work); */
    k_work_submit_to_queue(&time_sync_work_queue, &state.queued_work_item.work);
}

K_TIMER_DEFINE(event_timer, time_sync_event_handler, NULL);

int get_current_reference_time(uint64_t *local_now, uint64_t *ref_now)
{
    uint64_t _local_now = k_uptime_ticks();

    if(local_now != NULL) {
        *local_now = _local_now;
    }

    if (state.is_timesync_root || state.free_running) {
        *ref_now = _local_now;
        return 0;
    }

    // Mode-dependent readiness gate
    if (state.sync_mode == TIME_SYNC_MODE_SKEW) {
        // Skew mode: need >=3 updates for accurate skew estimation
        if (state.rtc_timesync_skew_update_count < 3) {
            return -ENOTIMEBASE;
        }
    } else {
        // Offset mode: just need base.ref to be set (1 update)
        if (state.rtc_clock_sync_state.base.ref == 0) {
            return -ENOTIMEBASE;
        }
    }

    if (timeutil_sync_ref_from_local(&state.rtc_clock_sync_state, _local_now, ref_now) < 0) {
        return -ENOTIMEBASE;
    }

    return 0;
}

int get_deca_local_timestamp(uint64_t reference_timestamp, uint64_t *local_timestamp) {
	uint64_t _local_timestamp;

	if(state.is_timesync_root) {
		*local_timestamp = reference_timestamp;
                return 0;
        }

	if (cyclic_timeutil_sync_local_from_ref(&state.deca_clock_sync_state, reference_timestamp,
						&_local_timestamp) < 0) {
		return -ENOTIMEBASE;
	}

        *local_timestamp = _local_timestamp;

        return 0;
}

int get_deca_reference_timestamp(uint64_t local_timestamp, uint64_t *reference_timestamp) {
        uint64_t _reference_timestamp;

        if(state.is_timesync_root) {
                *reference_timestamp = local_timestamp;
                return 0;
        }

        if (cyclic_timeutil_sync_ref_from_local(&state.deca_clock_sync_state, local_timestamp,
                                                &_reference_timestamp) < 0) {
                return -ENOTIMEBASE;
        }

        *reference_timestamp = _reference_timestamp;

        return 0;
}

int ref_rtc_to_deca(uint64_t ref_rtc_timestamp, uint64_t *deca_timestamp) {
    /* uint64_t rtc_ref_delta = ref_rtc_timestamp - state.rtc_clock_sync_state.base.ref; */
    /* uint64_t rtc_ref_delta_dwt_us = US_TO_DWT_TS(TICKS_TO_USEC(rtc_ref_delta)); */
    /* *deca_timestamp = (state.deca_clock_sync_state.base.ref + rtc_ref_delta_dwt_us) % deca_sync_conf.ref_wrap; */

    uint64_t _deca_timestamp;

    if (cyclic_timeutil_sync_ref_from_local(&state.rtc_deca_clock_sync_state, ref_rtc_timestamp,
            &_deca_timestamp) < 0) {
        return -ENOTIMEBASE;
    }

    *deca_timestamp = _deca_timestamp;

    return 0;
}

// guard has to take into account both: uncertainty due to lower clock resolution + clock sync error, you should choose it reasonable high!
uint64_t calculate_deca_slot_ts(uint64_t slot_start_ref_rtc, uint64_t guard_us) {
    // look at how the offset of the slot was at the last offset
    uint64_t last_modulo = (state.deca_clock_sync_state.base.ref % state.decatick_slot_duration);
    uint64_t __attribute__((unused)) modified_slot_rtc = slot_start_ref_rtc;

    uint64_t ref_deca;
    ref_rtc_to_deca(slot_start_ref_rtc, &ref_deca);


    // if we are too close to the slot boundary, we will schedule our transmission into the next slot
    if(last_modulo < US_TO_DWT_TS(guard_us)) {
        /* modified_slot_rtc += USEC_TO_TICKS(guard_us); */
        ref_deca += US_TO_DWT_TS(guard_us);
    } else if(last_modulo > state.decatick_slot_duration - US_TO_DWT_TS(guard_us)) {
        /* modified_slot_rtc += 2*USEC_TO_TICKS(guard_us); */
        ref_deca += 2*US_TO_DWT_TS(guard_us);
    }

    return (ref_deca + (state.decatick_slot_duration - ref_deca % state.decatick_slot_duration))  % deca_sync_conf.ref_wrap;
}

/* uint64_t calculate_deca_slot_ts(uint64_t slot_start_ref_rtc, uint64_t guard_us) { */
/*     // look at how the offset of the slot was at the last offset */
/*     uint64_t deca_ts; */
/*     ref_rtc_to_deca(slot_start_ref_rtc, &deca_ts); */

/*     uint64_t mod = deca_ts % state.decatick_slot_duration; */

/*     // if we are too close to the slot boundary, we will schedule our transmission into the next slot */
/*     if(mod < US_TO_DWT_TS(guard_us) || mod > state.decatick_slot_duration - US_TO_DWT_TS(guard_us)) { */
/*         deca_ts += 2*USEC_TO_TICKS(guard_us); */
/*     } */

/*     return (deca_ts + (state.decatick_slot_duration - deca_ts % state.decatick_slot_duration))  % deca_sync_conf.ref_wrap; */
/* } */

void time_sync_init(uint8_t is_timesync_root, uint32_t slot_duration_ms)
{
    // Stop the timer if it's currently running
    k_timer_stop(&event_timer);

    state.is_timesync_root = is_timesync_root;
    state.rtc_timesync_skew_update_count = 0;
    state.deca_timesync_skew_update_count = 0;
    state.has_glossy_result = false;

    state.decatick_slot_duration = US_TO_DWT_TS(20000);

    // convert slot_duration_ms to ticks
    state.systick_slot_duration = slot_duration_ms * CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1000;

    // reset base and latest data of time_util_sync_state
    state.rtc_clock_sync_state.base.local = 0;
    state.rtc_clock_sync_state.base.ref = 0;
    state.rtc_clock_sync_state.latest.local = 0;
    state.rtc_clock_sync_state.latest.ref = 0;

    state.deca_clock_sync_state.base.local = 0;
    state.deca_clock_sync_state.base.ref = 0;
    state.deca_clock_sync_state.latest.local = 0;
    state.deca_clock_sync_state.latest.ref = 0;

    state.rtc_deca_clock_sync_state.base.local = 0;
    state.rtc_deca_clock_sync_state.base.ref = 0;
    state.rtc_deca_clock_sync_state.latest.local = 0;
    state.rtc_deca_clock_sync_state.latest.ref = 0;

    state.last_glossy_event_time = 0;

    k_work_init(&state.queued_work_item.work, event_worker);

    k_work_queue_start(&time_sync_work_queue, time_sync_work_queue_stack,
		       K_KERNEL_STACK_SIZEOF(time_sync_work_queue_stack),
		       CONFIG_TIME_SYNCHRONIZATION_THREAD_PRIORITY, NULL);
    k_thread_name_set(&time_sync_work_queue.thread, "timesync_wq");
    /* k_work_queue_start(&time_sync_work_queue, time_sync_work_queue_stack, */
    /* K_KERNEL_STACK_SIZEOF(time_sync_work_queue_stack), */
    /* CONFIG_SYSTEM_WORKQUEUE_PRIORITY+1, NULL); */
}

void time_sync_set_slot_duration(uint32_t slot_duration_ms)
{
    state.systick_slot_duration = slot_duration_ms * CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1000;
}

void time_sync_set_root_mode(bool is_root)
{
    state.is_timesync_root = is_root ? 1 : 0;

    if (is_root) {
        /* Reset sync state when becoming root */
        state.rtc_timesync_skew_update_count = 0;
        state.deca_timesync_skew_update_count = 0;
        LOG_INF("Node is now timesync root (Glossy flood initiator)");
    } else {
        LOG_INF("Node is now timesync receiver");
    }
}

void time_sync_set_free_running(bool enable)
{
    state.free_running = enable;
    LOG_INF("Free-running mode %s", enable ? "enabled" : "disabled");
}

bool time_sync_is_free_running(void)
{
    return state.free_running;
}

void time_sync_set_mode(time_sync_mode_t mode)
{
    state.sync_mode = mode;
    LOG_INF("Time sync mode set to %s", mode == TIME_SYNC_MODE_OFFSET ? "OFFSET" : "SKEW");
}

time_sync_mode_t time_sync_get_mode(void)
{
    return state.sync_mode;
}

void time_sync_set_dirty(void)
{
    state.sync_dirty = true;
}

void time_sync_clear_dirty(void)
{
    state.sync_dirty = false;
}

bool time_sync_is_dirty(void)
{
    return state.sync_dirty;
}

static bool resync_requested = false;

void time_sync_request_resync(void)  { resync_requested = true; }
bool time_sync_resync_requested(void) { return resync_requested; }
void time_sync_clear_resync(void)    { resync_requested = false; }

void time_sync_set_tau_local(uint64_t tau_local_ticks)
{
    state.tau_local_ticks = tau_local_ticks;
}

uint64_t time_sync_get_tau_local(void)
{
    return state.tau_local_ticks;
}

void time_sync_set_tau_local_dwt(uint64_t tau_local_dwt)
{
    state.tau_local_dwt = tau_local_dwt;
}

uint64_t time_sync_get_tau_local_dwt(void)
{
    return state.tau_local_dwt;
}

void time_sync_set_tau_ref_rtc(uint64_t tau_ref_rtc)
{
    state.tau_ref_rtc = tau_ref_rtc;
}

uint64_t time_sync_get_tau_ref_rtc(void)
{
    return state.tau_ref_rtc;
}

void time_sync_reset_sync_state(void)
{
    /* Reset skew counters */
    state.rtc_timesync_skew_update_count = 0;
    state.deca_timesync_skew_update_count = 0;

    /* Reset all clock sync states (base + latest pairs) */
    state.rtc_clock_sync_state.base.local = 0;
    state.rtc_clock_sync_state.base.ref = 0;
    state.rtc_clock_sync_state.latest.local = 0;
    state.rtc_clock_sync_state.latest.ref = 0;

    state.deca_clock_sync_state.base.local = 0;
    state.deca_clock_sync_state.base.ref = 0;
    state.deca_clock_sync_state.latest.local = 0;
    state.deca_clock_sync_state.latest.ref = 0;

    state.rtc_deca_clock_sync_state.base.local = 0;
    state.rtc_deca_clock_sync_state.base.ref = 0;
    state.rtc_deca_clock_sync_state.latest.local = 0;
    state.rtc_deca_clock_sync_state.latest.ref = 0;

    /* Reset Glossy state */
    state.last_glossy_event_time = 0;
    state.has_glossy_result = false;
    memset(&state.last_glossy_result, 0, sizeof(state.last_glossy_result));

    LOG_INF("Time sync state reset (config preserved)");
}

/* Maximum tolerable delta between predicted and actual reference time.
 * If a glossy update exceeds this, the sync state is stale and must reset. */
#define SYNC_RESET_THRESHOLD_TICKS (2 * CONFIG_SYS_CLOCK_TICKS_PER_SEC)  /* 2 seconds */

void time_sync_update(uint64_t event_time, struct deca_glossy_result sync_result)
{
    struct timeutil_sync_instant rtc_clock_sync_instant;
    struct cyclic_timeutil_sync_instant deca_clock_synchronization_instant;
    struct cyclic_timeutil_sync_instant rtc_deca_clock_synchronization_instant;

    /* Auto-reset on large time delta: if we have a valid sync state,
     * predict the reference time from the local clock and compare with
     * the actual glossy reference. If they diverge by more than the
     * threshold, the sync state is stale (e.g., after superframe
     * reconfiguration or prolonged sync loss). Reset and start fresh. */
    if (state.rtc_timesync_skew_update_count >= 3 &&
        state.rtc_clock_sync_state.base.ref != 0) {
        uint64_t local_now = sync_result.rtc_clock_pair.local;
        uint64_t actual_ref = sync_result.rtc_clock_pair.ref;
        uint64_t predicted_ref;

        if (timeutil_sync_ref_from_local(&state.rtc_clock_sync_state,
                                          local_now, &predicted_ref) >= 0) {
            int64_t delta = (int64_t)actual_ref - (int64_t)predicted_ref;
            if (delta < 0) delta = -delta;

            if (delta > SYNC_RESET_THRESHOLD_TICKS) {
                LOG_WRN("Time sync discontinuity detected: delta=%lld ticks "
                        "(%.1f s), resetting sync state",
                        delta, (double)delta / CONFIG_SYS_CLOCK_TICKS_PER_SEC);
                time_sync_reset_sync_state();
                /* Fall through to process this glossy result as the first
                 * update of the fresh sync state. */
            }
        }
    }

    /* In offset mode, each round is independent. Reset sync states so this
     * update becomes the new base (skew=1.0). Never accumulate skew across
     * rounds -- sigma may change and clocks are not comparable. */
    if (state.sync_mode == TIME_SYNC_MODE_OFFSET) {
        state.rtc_clock_sync_state.base = (struct timeutil_sync_instant){};
        state.rtc_clock_sync_state.latest = (struct timeutil_sync_instant){};
        state.rtc_clock_sync_state.skew = 0;

        state.deca_clock_sync_state.base = (struct cyclic_timeutil_sync_instant){};
        state.deca_clock_sync_state.latest = (struct cyclic_timeutil_sync_instant){};
        state.deca_clock_sync_state.skew = 0;

        state.rtc_deca_clock_sync_state.base = (struct cyclic_timeutil_sync_instant){};
        state.rtc_deca_clock_sync_state.latest = (struct cyclic_timeutil_sync_instant){};
        state.rtc_deca_clock_sync_state.skew = 0;
    }

    rtc_clock_sync_instant.local = sync_result.rtc_clock_pair.local;
    rtc_clock_sync_instant.ref = sync_result.rtc_clock_pair.ref;
    deca_clock_synchronization_instant.local = sync_result.deca_clock_pair.local;
    deca_clock_synchronization_instant.ref = sync_result.deca_clock_pair.ref;
    rtc_deca_clock_synchronization_instant.local = sync_result.rtc_clock_pair.ref;
    rtc_deca_clock_synchronization_instant.ref = sync_result.deca_clock_pair.ref;

    // Log raw timestamp values upon successful sync round
    LOG_DBG("Glossy sync - RTC local: %llu us, RTC ref: %llu us",
            TICKS_TO_USEC(sync_result.rtc_clock_pair.local),
            TICKS_TO_USEC(sync_result.rtc_clock_pair.ref));
    LOG_DBG("Glossy sync - DWT local: %llu us, DWT ref: %llu us",
            DWT_TS_TO_US(sync_result.deca_clock_pair.local),
            DWT_TS_TO_US(sync_result.deca_clock_pair.ref));

    // Update RTC
    int rtc_update_result = timeutil_sync_state_update(&state.rtc_clock_sync_state, &rtc_clock_sync_instant);
    if (rtc_update_result > 0) {
        float skew;

        skew = timeutil_sync_estimate_skew(&state.rtc_clock_sync_state);
        timeutil_sync_state_set_skew(&state.rtc_clock_sync_state, skew, &rtc_clock_sync_instant);

        LOG_DBG("RTC sync update - skew: %d ppm, base local: %llu us, base ref: %llu us",
                (int)(skew * 1000000),
                TICKS_TO_USEC(state.rtc_clock_sync_state.base.local),
                TICKS_TO_USEC(state.rtc_clock_sync_state.base.ref));

        state.rtc_timesync_skew_update_count++;
    }

    // Update DECA
    if (cyclic_timeutil_sync_state_update(&state.deca_clock_sync_state, &deca_clock_synchronization_instant)) {
        float skew;

        skew = cyclic_timeutil_sync_estimate_skew(&state.deca_clock_sync_state);
        cyclic_timeutil_sync_state_set_skew(&state.deca_clock_sync_state, skew, &deca_clock_synchronization_instant);

        LOG_DBG("DWT sync update - skew: %d ppm, base local: %llu us, base ref: %llu us",
                (int)(skew * 1000000),
                DWT_TS_TO_US(state.deca_clock_sync_state.base.local),
                DWT_TS_TO_US(state.deca_clock_sync_state.base.ref));
    }

    // Update RTC<->DECA
    if (cyclic_timeutil_sync_state_update(&state.rtc_deca_clock_sync_state, &rtc_deca_clock_synchronization_instant)) {
        float skew;

        skew = cyclic_timeutil_sync_estimate_skew(&state.rtc_deca_clock_sync_state);
        cyclic_timeutil_sync_state_set_skew(&state.rtc_deca_clock_sync_state, skew, &rtc_deca_clock_synchronization_instant);

        LOG_DBG("RTC<->DWT sync update - skew: %d ppm, base local: %llu us, base ref: %llu us",
                (int)(skew * 1000000),
                TICKS_TO_USEC(state.rtc_deca_clock_sync_state.base.local),
                DWT_TS_TO_US(state.rtc_deca_clock_sync_state.base.ref));
    }

    // Store the last successful sync result
    state.last_glossy_result = sync_result;
    state.has_glossy_result = true;
    state.last_glossy_event_time = event_time;

    // Sync succeeded this round -- clear dirty flag
    state.sync_dirty = false;
}

int slotted_schedule_work_next_slot(block_handler_t block_handler,
    void *block_user_data, time_sync_event_finished_callback_t event_finished_callback, void *callback_user_data)
{
    int ret;
    uint64_t ref_now, ref_slot_start_ts;

    if( (ret = get_current_reference_time(NULL, &ref_now)) < 0 ) {
        return ret;
    }

    ref_slot_start_ts = ref_now + (state.systick_slot_duration - ref_now % state.systick_slot_duration);

    return schedule_work_at(ref_slot_start_ts, block_handler, block_user_data, event_finished_callback, callback_user_data);
}

static void event_worker(struct k_work *work_item)
{
    struct time_sync_work_item *item = CONTAINER_OF(work_item, struct time_sync_work_item, work);

    uint64_t local_start = k_uptime_ticks();

#if IS_ENABLED(CONFIG_SYNCHROFLY_RADIO_ARBITER)
    radio_arbiter_claim();
#endif
    item->block_handler(state.next_event_time, item->block_user_data);

    uint64_t local_end = k_uptime_ticks();

    /* Pass local (monotonic) timestamps for duration tracking.
     * Reference time can jump between Butler rounds with different sigma,
     * making ref-based duration meaningless in offset mode. */
    item->event_finished_callback(local_start, local_end, item->callback_user_data);
}

// returns the current asn
int slotted_schedule_get_asn()
{
    int ret;
    uint64_t ref_now;
    if ((ret = get_current_reference_time(NULL, &ref_now)) < 0) {
        return ret;
    }

    return (ref_now / state.systick_slot_duration);
}

int schedule_work_at(uint64_t ref_event_start_ts, block_handler_t block_handler,
    void *block_user_data, time_sync_event_finished_callback_t event_finished_callback, void *callback_user_data)
{
    int ret;
    uint64_t ref_now, local_now, local_event_start_ts = 0;
    k_timeout_t next_event_delay;

    /* Don't schedule new work while paused (e.g., during DFU) */
    if (state.scheduler_paused) {
        return -EBUSY;
    }

    if( (ret = get_current_reference_time(&local_now, &ref_now)) < 0 ) {
	return ret;
    }

    state.next_event_time = ref_event_start_ts;

    // convert back into local time frame
    if (state.is_timesync_root || state.free_running) {
        local_event_start_ts = ref_event_start_ts;
        next_event_delay = K_TIMEOUT_ABS_TICKS(ref_event_start_ts);
    } else {
        timeutil_sync_local_from_ref(&state.rtc_clock_sync_state, ref_event_start_ts, &local_event_start_ts);
        next_event_delay = K_TIMEOUT_ABS_TICKS(local_event_start_ts);
    }
    LOG_DBG("swa d=%lld local=%llu now=%llu",
            (int64_t)local_event_start_ts - (int64_t)local_now,
            local_event_start_ts, local_now);

    /* Save callback context for restart after pause */
    state.saved_callback = event_finished_callback;
    state.saved_callback_user_data = callback_user_data;

    // create a work item for the passed event_handler, user_data and event_finished_callback
    state.queued_work_item.work_start_ref_rtc = ref_event_start_ts;
    state.queued_work_item.block_handler = block_handler;
    state.queued_work_item.event_finished_callback = event_finished_callback;
    state.queued_work_item.block_user_data = block_user_data;
    state.queued_work_item.callback_user_data = callback_user_data;
    /* k_work_init is done once in time_sync_init(). Calling it here
     * would reinitialize the work item while event_worker is still
     * executing on the timesync_wq -- undefined behavior that
     * eventually corrupts the work queue and silently stops it. */

    k_timer_start(&event_timer, next_event_delay, K_NO_WAIT);

    return 0;
}

int schedule_work_at_local(uint64_t local_event_start_ticks, uint64_t ref_event_time,
    block_handler_t block_handler, void *block_user_data,
    time_sync_event_finished_callback_t event_finished_callback,
    void *callback_user_data)
{
    if (state.scheduler_paused) {
        return -EBUSY;
    }

    /* Store the caller-provided reference time for block handlers.
     * In beacon mode, this is tau_ref_rtc + block_offset (deterministic). */
    state.next_event_time = ref_event_time;

    /* Save callback context for restart after pause */
    state.saved_callback = event_finished_callback;
    state.saved_callback_user_data = callback_user_data;

    state.queued_work_item.work_start_ref_rtc = local_event_start_ticks;
    state.queued_work_item.block_handler = block_handler;
    state.queued_work_item.event_finished_callback = event_finished_callback;
    state.queued_work_item.block_user_data = block_user_data;
    state.queued_work_item.callback_user_data = callback_user_data;

    k_timer_start(&event_timer, K_TIMEOUT_ABS_TICKS(local_event_start_ticks), K_NO_WAIT);

    return 0;
}

int get_last_glossy_result(struct deca_glossy_result *result, uint64_t *rtc_event_time)
{
    if (!state.has_glossy_result) {
        return -1;
    }

    // last glossy round was not scheduled in root timebase (or respectively on slot boundary)

    *result = state.last_glossy_result;
    *rtc_event_time = state.last_glossy_event_time;

    if(state.last_glossy_event_time == 0) {
        return 0;
    } else {
        return 1;
    }
}

void time_sync_scheduler_pause(void)
{
    if (state.scheduler_paused) {
        return;
    }

    state.scheduler_paused = true;
    k_timer_stop(&event_timer);

    /* Cancel any pending work in the queue */
    k_work_cancel(&state.queued_work_item.work);

    LOG_INF("Time sync scheduler paused");
}

void time_sync_scheduler_resume(void)
{
    if (!state.scheduler_paused) {
        return;
    }

    state.scheduler_paused = false;
    LOG_INF("Time sync scheduler resumed");

    /* Restart the scheduling chain if we have saved callback context */
    if (state.saved_callback != NULL) {
        uint64_t ref_now;
        if (get_current_reference_time(NULL, &ref_now) == 0) {
            /* Call the callback with current time to restart the chain.
             * The callback (background_schedule_next_network_event) will
             * schedule the next slot based on current ASN.
             */
            LOG_INF("Restarting scheduler chain");
            state.saved_callback(ref_now, ref_now, state.saved_callback_user_data);
        } else {
            LOG_WRN("Cannot restart scheduler - no time base");
        }
    }
}

bool time_sync_scheduler_is_paused(void)
{
    return state.scheduler_paused;
}
