#include <app/lib/timesync/time_synchronization.h>
#include <app/lib/management/network_settings.h>
#include <app/lib/system/hw.h>
#include <app/lib/system/node.h>

#include <app/lib/scheduling/upper/block_scheduler.h>
#include <app/lib/scheduling/upper/block_type_registry.h>
#include <app/lib/management/superframe_settings.h>

#include <app/lib/blocks/blocks.h>
#include <app/lib/blocks/glossy.h>
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
#include <app/lib/blocks/butler.h>
#endif
#include <app/lib/stats/firmware_stats.h>
#if IS_ENABLED(CONFIG_SYNCHROFLY_SWARM_RANGING)
#include <app/lib/swarm_ranging/swarm_ranging_core.h>
#include <app/lib/swarm_ranging/swarm_uwb_interface.h>
#include <app/lib/blocks/block_types.h>
#endif

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(block_scheduler);

struct network_scheduler_settings {
    uint32_t initial_warmup_period_ms;
};

static struct network_scheduler_settings settings = {
    .initial_warmup_period_ms = CONFIG_SYNCHROFLY_BLOCK_SCHEDULER_WARMUP_MS,
};

// Default glossy config for warmup phase
static struct glossy_block_config warmup_glossy_config;

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
static struct butler_block_config warmup_butler_config;
#endif

static bool warmup_config_initialized = false;

static void init_warmup_configs(void) {
    if (warmup_config_initialized) {
        return;
    }

    warmup_glossy_config.max_depth = network_get_glossy_max_depth();
    warmup_glossy_config.transmission_delay_us = network_get_glossy_transmission_delay_us();
    warmup_glossy_config.guard_period_us = network_get_glossy_guard_us();

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
    /* Default Butler warmup config -- reasonable for 5-10 node networks */
    warmup_butler_config.max_subslots = 100;
    warmup_butler_config.subslot_duration_us = 1000;
    warmup_butler_config.guard_period_us = 500;
    warmup_butler_config.p_tx_pct = 5;
    warmup_butler_config.channel = 5;
#endif

    warmup_config_initialized = true;
    LOG_DBG("Warmup configs initialized from network settings");
}

// Execution context passed through callbacks
struct superframe_exec_ctx {
    struct superframe *sframe;
    superframe_callback_t done_callback;
    void *done_callback_user_data;
    bool warmup_complete;
    int last_executed_block;  // Track which block we last ran for timing validation
    /* Beacon-relative mode state */
    uint64_t tau_local_ticks;  // Cycle anchor: tau in local ticks (from last successful Butler)
    int next_block_index;      // Next block to schedule (0 = Butler, 1..N = post-Butler blocks)
};

// Static execution context (only one superframe can run at a time)
static struct superframe_exec_ctx exec_ctx;

// Max measured execution time per block (in ms)
static uint16_t block_max_execution_ms[CONFIG_SYNCHROFLY_SUPERFRAME_MAX_SLOTS];

void block_scheduler_reset_timing_stats(void) {
    memset(block_max_execution_ms, 0, sizeof(block_max_execution_ms));
}

#if IS_ENABLED(CONFIG_SYNCHROFLY_SWARM_RANGING)
static bool swarm_ranging_active = false;

/**
 * Check if the superframe contains a swarm ranging block.
 * Init or deinit swarm ranging accordingly.
 */
static void sync_swarm_ranging_state(struct superframe *sf)
{
    bool has_swarm = false;
    for (int i = 0; i < sf->block_count; i++) {
        if (sf->blocks[i].type == BLOCK_TYPE_SWARM_RANGING) {
            has_swarm = true;
            break;
        }
    }

    if (has_swarm && !swarm_ranging_active) {
        uint16_t addr = swarm_uwb_get_address();
        int ret = ranging_init(addr);
        if (ret == 0) {
            swarm_ranging_active = true;
            LOG_INF("Swarm ranging initialized (addr: 0x%04x)", addr);
        } else {
            LOG_ERR("Failed to init swarm ranging: %d", ret);
        }
    } else if (!has_swarm && swarm_ranging_active) {
        ranging_deinit();
        swarm_ranging_active = false;
        LOG_INF("Swarm ranging deinitialized (no block in superframe)");
    }
}
#endif

/**
 * Compute timing information for the superframe.
 * Resolves default durations and calculates cumulative offsets.
 */
static void compute_superframe_timing(struct superframe *sf) {
    uint32_t offset = 0;
    uint32_t default_duration = network_get_scheduler_slot_duration_ms();

    for (int i = 0; i < sf->block_count; i++) {
        sf->block_start_offsets_ms[i] = offset;

        uint32_t dur = sf->blocks[i].duration_ms;
        if (dur == 0) {
            dur = default_duration;
            sf->blocks[i].duration_ms = dur;  // Store resolved duration
        }
        offset += dur;
    }
    sf->total_duration_ms = offset;

    LOG_INF("Superframe timing: %d blocks, total %u ms", sf->block_count, sf->total_duration_ms);
    for (int i = 0; i < sf->block_count; i++) {
        LOG_DBG("  Block %d: offset=%u ms, duration=%u ms",
                i, sf->block_start_offsets_ms[i], sf->blocks[i].duration_ms);
    }
}

/**
 * Find which block should be executing at a given reference time.
 */
static int get_block_index_at_time(struct superframe *sf, uint64_t ref_time_ms) {
    if (sf->total_duration_ms == 0) {
        return 0;
    }

    uint32_t cycle_time = ref_time_ms % sf->total_duration_ms;

    for (int i = 0; i < sf->block_count; i++) {
        uint32_t block_end = sf->block_start_offsets_ms[i] + sf->blocks[i].duration_ms;
        if (cycle_time < block_end) {
            return i;
        }
    }
    return 0;  // Should not reach here
}

/**
 * Get the reference time (in ms) when the next block should start.
 */
static uint64_t get_next_block_start_time_ms(struct superframe *sf, uint64_t ref_now_ms) {
    if (sf->total_duration_ms == 0) {
        return ref_now_ms;
    }

    uint32_t cycle_time = ref_now_ms % sf->total_duration_ms;
    uint64_t cycle_start = ref_now_ms - cycle_time;

    // Find current block
    int current_block = get_block_index_at_time(sf, ref_now_ms);

    // Next block start is current block's end
    uint32_t current_block_end_offset = sf->block_start_offsets_ms[current_block]
                                       + sf->blocks[current_block].duration_ms;

    // Handle wrap to next cycle
    if (current_block_end_offset >= sf->total_duration_ms) {
        return cycle_start + sf->total_duration_ms;  // Start of next cycle
    }

    return cycle_start + current_block_end_offset;
}

/**
 * Beacon-relative scheduling callback.
 * Butler (block 0) anchors each cycle at tau. Subsequent blocks are chained
 * at fixed local-time offsets from tau. No modulo, no reference time for
 * scheduling decisions.
 */
static void schedule_next_block_beacon(uint64_t event_start_ticks,
                                       uint64_t event_finished_ticks,
                                       void *user_data)
{
    struct superframe_exec_ctx *ctx = (struct superframe_exec_ctx *)user_data;
    struct superframe *sf = ctx->sframe;

    init_warmup_configs();

    if (get_node_mode() == NODE_MODE_DISABLED) {
        if (ctx->done_callback) {
            ctx->done_callback(SUPERFRAME_STATUS_DISABLED, ctx->done_callback_user_data);
        }
        return;
    }

    /* Timing validation of previous block (uses local timestamps) */
    if (event_start_ticks > 0 && ctx->last_executed_block >= 0) {
        uint64_t actual_duration_ms = TICKS_TO_MSEC(event_finished_ticks - event_start_ticks);
        int block_idx = ctx->last_executed_block;
        uint32_t expected_duration = sf->blocks[block_idx].duration_ms;

        if (block_idx < CONFIG_SYNCHROFLY_SUPERFRAME_MAX_SLOTS) {
            if (actual_duration_ms > block_max_execution_ms[block_idx]) {
                block_max_execution_ms[block_idx] = (uint16_t)MIN(actual_duration_ms, UINT16_MAX);
            }
        }

        if (actual_duration_ms > expected_duration) {
            LOG_WRN("Block %d exceeded time: %llu ms > %u ms",
                    block_idx, actual_duration_ms, expected_duration);
        }

        firmware_stats_inc(STAT_ID_SCHED_BLOCKS_EXECUTED);
    }

    /* Warmup: run Butler repeatedly until shared reference time exceeds threshold.
     * tau_ref_rtc is sigma's RTC at tau -- shared across all converged nodes.
     * All nodes cross the threshold within 1-2 rounds of each other. */
    if (!ctx->warmup_complete) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
        butler_block_handler(0, &warmup_butler_config);

        uint64_t tau_ref = time_sync_get_tau_ref_rtc();
        if (tau_ref > 0) {
            uint64_t ref_ms = TICKS_TO_MSEC(tau_ref);
            uint32_t cycle_ms = sf->total_duration_ms;
            uint64_t warmup_end_ms = settings.initial_warmup_period_ms;
            if (cycle_ms > 0) {
                warmup_end_ms = ((warmup_end_ms + cycle_ms - 1) / cycle_ms) * cycle_ms;
            }
            if (ref_ms >= warmup_end_ms) {
                ctx->warmup_complete = true;
                LOG_INF("Butler warmup complete at ref=%llu ms (target=%llu ms)",
                        ref_ms, warmup_end_ms);
            }
        }

        if (!ctx->warmup_complete) {
            k_msleep(5);
            schedule_next_block_beacon(0, 0, ctx);
            return;
        }
#endif
    }

    /* --- Resync requested (disabled for now, may be re-enabled for partition merge) --- */
#if 0
    if (time_sync_resync_requested()) {
        time_sync_clear_resync();
        if (!time_sync_is_dirty()) {
            uint64_t new_tau = time_sync_get_tau_local();
            if (new_tau != 0) {
                ctx->tau_local_ticks = new_tau;
            }
            ctx->next_block_index = 1;
            ctx->last_executed_block = 0;
            LOG_WRN("Inline resync complete -- restarting superframe");
            schedule_next_block_beacon(0, 0, ctx);
            return;
        }
        ctx->next_block_index = 0;
        ctx->last_executed_block = -1;
        uint64_t now = k_uptime_ticks();
        schedule_work_at_local(now + MSEC_TO_TICKS(1), 1,
                               sf->blocks[0].block_handler,
                               sf->blocks[0].config,
                               schedule_next_block_beacon, ctx);
        LOG_WRN("Resync requested -- running Butler immediately");
        return;
    }
#endif

    /* --- Schedule Butler (block 0) for next cycle --- */
    if (ctx->next_block_index == 0) {
        uint64_t butler_start;
        if (ctx->tau_local_ticks > 0) {
            /* Project forward from last known tau to the next cycle boundary.
             * cycle_start = tau - butler_radio_time.
             * If we missed cycles, skip ahead to the next valid one. */
            struct butler_block_config *bc =
                (struct butler_block_config *)sf->blocks[0].config;
            uint32_t radio_us = bc->guard_period_us + bc->subslot_duration_us +
                                (uint32_t)bc->max_subslots * bc->subslot_duration_us;
            uint64_t cycle_start = ctx->tau_local_ticks - MSEC_TO_TICKS(radio_us / 1000);
            uint64_t now = k_uptime_ticks();
            uint64_t total_ticks = MSEC_TO_TICKS(sf->total_duration_ms);
            if (now > cycle_start) {
                uint64_t elapsed = now - cycle_start;
                uint64_t cycles = (elapsed + total_ticks - 1) / total_ticks; /* ceil */
                butler_start = cycle_start + cycles * total_ticks;
            } else {
                butler_start = cycle_start + total_ticks;
            }
        } else {
            /* First cycle or no tau yet -- start now */
            butler_start = k_uptime_ticks() + MSEC_TO_TICKS(1);
        }

        ctx->last_executed_block = 0;
        ctx->next_block_index = 1;

        /* Pass tau_ref_rtc as event_time for Butler. This ends up in
         * state.last_glossy_event_time via time_sync_update(), which MTM
         * checks via get_last_glossy_result() (returns 0 if event_time==0). */
        uint64_t butler_ref_event = time_sync_get_tau_ref_rtc();
        if (butler_ref_event == 0) {
            butler_ref_event = 1;  /* Ensure non-zero for first boot */
        }
        int err = schedule_work_at_local(butler_start, butler_ref_event,
                                         sf->blocks[0].block_handler,
                                         sf->blocks[0].config,
                                         schedule_next_block_beacon, ctx);
        if (err) {
            LOG_ERR("Failed to schedule Butler: %d", err);
        }
        return;
    }

    /* --- Butler just completed: update tau anchor --- */
    if (ctx->last_executed_block == 0) {
        uint64_t new_tau = time_sync_get_tau_local();
        if (new_tau != 0 && new_tau != ctx->tau_local_ticks) {
            ctx->tau_local_ticks = new_tau;
        }
        /* If Butler didn't converge (new_tau == 0 or unchanged), keep old tau.
         * Dirty flag is set, so needs_sync blocks will be skipped below. */

        if (ctx->tau_local_ticks == 0) {
            /* No Butler ever converged -- skip to next cycle */
            LOG_WRN("No tau available, retrying Butler in 100ms");
            ctx->next_block_index = 0;
            ctx->last_executed_block = -1;
            uint64_t retry = k_uptime_ticks() + MSEC_TO_TICKS(100);
            schedule_work_at_local(retry, 0,
                                   sf->blocks[0].block_handler,
                                   sf->blocks[0].config,
                                   schedule_next_block_beacon, ctx);
            return;
        }
    }

    /* --- Find next runnable block (skip dirty needs_sync blocks) --- */
    int next_idx = ctx->next_block_index;

    /* Butler radio time: tau = own_rtc + startup_guard + max_subslots * subslot.
     * startup_guard = guard_period_us + subslot_duration_us.
     * cycle_start = tau - butler_radio_time.
     * All nodes with the same sigma derive the same cycle_start. */
    struct butler_block_config *butler_conf =
        (struct butler_block_config *)sf->blocks[0].config;
    uint32_t butler_radio_us = butler_conf->guard_period_us +
                               butler_conf->subslot_duration_us +
                               (uint32_t)butler_conf->max_subslots *
                               butler_conf->subslot_duration_us;
    uint32_t butler_radio_ms = butler_radio_us / 1000;

    for (int skipped = 0; skipped < sf->block_count; skipped++) {
        if (next_idx >= sf->block_count) {
            /* Past last block -- go to next cycle */
            ctx->next_block_index = 0;
            schedule_next_block_beacon(event_start_ticks, event_finished_ticks, ctx);
            return;
        }

        struct block_config *candidate = &sf->blocks[next_idx];
        if (candidate->block_handler == NULL) {
            next_idx++;
            continue;
        }

        const struct block_type_info *info = block_type_get_info(candidate->type);
        if (info && info->needs_sync && time_sync_is_dirty()) {
            LOG_DBG("Block %d (%s) skipped: sync dirty", next_idx, info->name);
            next_idx++;
            continue;
        }
        break;
    }

    if (next_idx >= sf->block_count) {
        /* All remaining blocks skipped -- next cycle */
        ctx->next_block_index = 0;
        schedule_next_block_beacon(event_start_ticks, event_finished_ticks, ctx);
        return;
    }

    /* Compute block start relative to cycle_start = tau - butler_radio_time.
     * block_start = cycle_start + block_start_offsets_ms[next_idx]
     *             = tau - butler_radio_ms + block_start_offsets_ms[next_idx]
     * All nodes with the same sigma compute the same schedule. */
    uint32_t offset_from_tau_ms = sf->block_start_offsets_ms[next_idx] - butler_radio_ms;
    uint64_t block_start = ctx->tau_local_ticks + MSEC_TO_TICKS(offset_from_tau_ms);

    /* Compute deterministic ref_event_time: tau_ref_rtc + same offset.
     * Used by MTM for DWT anchor and hash seeding. */
    uint64_t tau_ref = time_sync_get_tau_ref_rtc();
    uint64_t ref_event = tau_ref + MSEC_TO_TICKS(offset_from_tau_ms);

    /* If target in past (Butler overran), schedule immediately */
    uint64_t now = k_uptime_ticks();
    if (block_start < now) {
        LOG_WRN("Block %d target in past by %llu ms, running immediately",
                next_idx, TICKS_TO_MSEC(now - block_start));
        block_start = now + MSEC_TO_TICKS(1);
    }

    ctx->last_executed_block = next_idx;
    ctx->next_block_index = next_idx + 1;

    int err = schedule_work_at_local(block_start, ref_event,
                                     sf->blocks[next_idx].block_handler,
                                     sf->blocks[next_idx].config,
                                     schedule_next_block_beacon, ctx);
    if (err) {
        LOG_ERR("schedule_work_at_local failed: %d (blk=%d)", err, next_idx);
    }
}

/**
 * Main scheduling callback - schedules the next block based on reference time.
 */
static void schedule_next_block(uint64_t event_start_ticks,
                                uint64_t event_finished_ticks,
                                void *user_data)
{
    struct superframe_exec_ctx *ctx = (struct superframe_exec_ctx *)user_data;
    struct superframe *sf = ctx->sframe;

    init_warmup_configs();

    if (get_node_mode() == NODE_MODE_DISABLED) {
        if (ctx->done_callback) {
            ctx->done_callback(SUPERFRAME_STATUS_DISABLED, ctx->done_callback_user_data);
        }
        return;
    }

    // Get current reference time
    uint64_t ref_now_ticks;
    if (get_current_reference_time(NULL, &ref_now_ticks) < 0) {
        LOG_WRN("No ref_time, sync fallback");
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
        if (time_sync_get_mode() == TIME_SYNC_MODE_OFFSET) {
            butler_block_handler(0, &warmup_butler_config);
        } else
#endif
        {
            glossy_block_handler(0, &warmup_glossy_config);
        }
        k_msleep(5);
        schedule_next_block(0, 0, ctx);
        return;
    }
    uint64_t ref_now_ms = TICKS_TO_MSEC(ref_now_ticks);

    // Check timing of previous block (if any)
    if (event_start_ticks > 0 && ctx->last_executed_block >= 0) {
        uint64_t actual_duration_ms = TICKS_TO_MSEC(event_finished_ticks - event_start_ticks);
        int block_idx = ctx->last_executed_block;
        uint32_t expected_duration = sf->blocks[block_idx].duration_ms;

        // Update max execution time for this block
        if (block_idx < CONFIG_SYNCHROFLY_SUPERFRAME_MAX_SLOTS) {
            if (actual_duration_ms > block_max_execution_ms[block_idx]) {
                block_max_execution_ms[block_idx] = (uint16_t)MIN(actual_duration_ms, UINT16_MAX);
            }
        }

        if (actual_duration_ms > expected_duration) {
            LOG_WRN("Block %d exceeded time: %llu ms > %u ms",
                    block_idx, actual_duration_ms, expected_duration);
        }

        firmware_stats_inc(STAT_ID_SCHED_BLOCKS_EXECUTED);
    }

    // Warmup phase: run glossy until we have stable sync
    if (!ctx->warmup_complete) {
        /*
         * Compute a network-wide warmup end time so ALL nodes transition at
         * the same reference-time boundary, regardless of when each node
         * first acquired sync.  We align to the superframe cycle length so
         * the first block starts cleanly at a cycle boundary.
         *
         * warmup_end = first cycle boundary >= initial_warmup_period_ms
         *
         * Since ref_now_ms is on a shared reference clock (the root's
         * uptime), every node sees the same value and will exit warmup at
         * the same instant.
         */
        uint32_t cycle_ms = sf->total_duration_ms;
        uint64_t warmup_end_ms = settings.initial_warmup_period_ms;
        if (cycle_ms > 0) {
            /* Round up to next cycle boundary */
            warmup_end_ms = ((warmup_end_ms + cycle_ms - 1) / cycle_ms) * cycle_ms;
        }

        if (ref_now_ms < warmup_end_ms) {
            // Still in warmup - schedule glossy
            uint32_t warmup_slot_duration = network_get_scheduler_slot_duration_ms();
            uint64_t warmup_slot_ticks = MSEC_TO_TICKS(warmup_slot_duration);
            uint64_t next_slot_ticks = ref_now_ticks + (warmup_slot_ticks - ref_now_ticks % warmup_slot_ticks);

            ctx->last_executed_block = -1;  // Warmup, not a real block
            schedule_work_at(next_slot_ticks, glossy_block_handler, &warmup_glossy_config,
                           schedule_next_block, ctx);
            return;
        }

        ctx->warmup_complete = true;
        LOG_INF("Warmup complete at ref_time=%llu ms (target=%llu ms), starting superframe",
                ref_now_ms, warmup_end_ms);
    }

    // Normal operation: find which block to run next
    int current_block_idx = get_block_index_at_time(sf, ref_now_ms);
    int next_block_idx = (current_block_idx + 1) % sf->block_count;

    // Calculate when the next block should start
    uint64_t next_start_ms = get_next_block_start_time_ms(sf, ref_now_ms);
    uint64_t next_start_ticks = MSEC_TO_TICKS(next_start_ms);

    /* Skip blocks that need sync when sync is dirty.
     * Iterate forward (non-recursively) to find the next runnable block.
     * Stop after one full cycle to avoid infinite loops. */
    for (int skipped = 0; skipped < sf->block_count; skipped++) {
        struct block_config *candidate = &sf->blocks[next_block_idx];
        if (candidate->type != BLOCK_TYPE_NONE && candidate->block_handler != NULL) {
            const struct block_type_info *info = block_type_get_info(candidate->type);
            if (info && info->needs_sync && time_sync_is_dirty()) {
                LOG_DBG("Block %d (%s) skipped: sync dirty",
                        next_block_idx, info->name);
                next_block_idx = (next_block_idx + 1) % sf->block_count;
                next_start_ms = get_next_block_start_time_ms(sf, next_start_ms);
                next_start_ticks = MSEC_TO_TICKS(next_start_ms);
                continue;
            }
        }
        break;  /* Found a runnable block (or NULL handler to skip) */
    }

    struct block_config *next_block = &sf->blocks[next_block_idx];
    if (!next_block->block_handler) {
        /* All blocks skipped or NULL handler -- schedule next cycle */
        next_start_ms = get_next_block_start_time_ms(sf, next_start_ms);
        next_start_ticks = MSEC_TO_TICKS(next_start_ms);
        schedule_work_at(next_start_ticks, ctx->sframe->blocks[0].block_handler,
                        ctx->sframe->blocks[0].config, schedule_next_block, ctx);
        return;
    }

    ctx->last_executed_block = next_block_idx;
    int sched_err = schedule_work_at(next_start_ticks, next_block->block_handler,
                    next_block->config, schedule_next_block, ctx);
    if (sched_err) {
        LOG_ERR("schedule_work_at failed: %d (blk=%d)", sched_err, next_block_idx);
    }
}

int superframe_alloc(uint16_t block_count, struct superframe **sframe) {
    if (block_count == 0 || sframe == NULL) {
        return -EINVAL;
    }

    // Allocate the sframe structure
    *sframe = k_malloc(sizeof(struct superframe));
    if (*sframe == NULL) {
        LOG_ERR("Failed to allocate memory for sframe structure");
        return -ENOMEM;
    }

    // Allocate memory for blocks array
    (*sframe)->blocks = k_malloc(block_count * sizeof(struct block_config));
    if ((*sframe)->blocks == NULL) {
        LOG_ERR("Failed to allocate memory for blocks array");
        k_free(*sframe);
        *sframe = NULL;
        return -ENOMEM;
    }

    // Allocate memory for block start offsets array
    (*sframe)->block_start_offsets_ms = k_malloc(block_count * sizeof(uint32_t));
    if ((*sframe)->block_start_offsets_ms == NULL) {
        LOG_ERR("Failed to allocate memory for block offsets array");
        k_free((*sframe)->blocks);
        k_free(*sframe);
        *sframe = NULL;
        return -ENOMEM;
    }

    // Initialize the allocated sframe
    (*sframe)->block_count = block_count;
    (*sframe)->total_duration_ms = 0;
    memset((*sframe)->blocks, 0, block_count * sizeof(struct block_config));
    memset((*sframe)->block_start_offsets_ms, 0, block_count * sizeof(uint32_t));

    return 0;
}

int superframe_free(struct superframe *sframe) {
    if (sframe == NULL) {
        return -EINVAL;
    }

    if (sframe->block_start_offsets_ms != NULL) {
        k_free(sframe->block_start_offsets_ms);
    }

    if (sframe->blocks != NULL) {
        k_free(sframe->blocks);
    }

    k_free(sframe);
    return 0;
}

int execute_superframe(struct superframe *sframe, bool repeat, superframe_callback_t callback, void *user_data)
{
    init_warmup_configs();

    if (!repeat) {
        LOG_ERR("Non-repeating superframe not implemented");
        if (callback) {
            callback(SUPERFRAME_STATUS_ERROR, user_data);
        }
        return -ENOTSUP;
    }

    if (get_node_mode() == NODE_MODE_DISABLED) {
        if (callback) {
            callback(SUPERFRAME_STATUS_DISABLED, user_data);
        }
        return -1;
    }

    // Compute timing information for the superframe
    compute_superframe_timing(sframe);

    // Reset timing stats for new superframe execution
    block_scheduler_reset_timing_stats();

#if IS_ENABLED(CONFIG_SYNCHROFLY_SWARM_RANGING)
    sync_swarm_ranging_state(sframe);
#endif

    // Initialize execution context
    exec_ctx.sframe = sframe;
    exec_ctx.done_callback = callback;
    exec_ctx.done_callback_user_data = user_data;
    exec_ctx.warmup_complete = false;
    exec_ctx.last_executed_block = -1;
    exec_ctx.tau_local_ticks = 0;
    exec_ctx.next_block_index = 0;

    if (sframe->scheduling_mode == SUPERFRAME_MODE_BEACON_RELATIVE) {
        /* Beacon-relative: warmup handled inside schedule_next_block_beacon */
        LOG_INF("Starting beacon-relative superframe (%u blocks, %u ms)",
                sframe->block_count, sframe->total_duration_ms);
        schedule_next_block_beacon(0, 0, &exec_ctx);
    } else {
        /* Modulo mode (Glossy): existing warmup + scheduling */
        if (time_sync_is_free_running()) {
            exec_ctx.warmup_complete = true;
            LOG_INF("Free-running mode: skipping time sync and warmup");
        } else {
            LOG_INF("Waiting for initial time synchronization...");
            int sync_attempts = 0;
            uint64_t ref_time_dummy;
            while (get_current_reference_time(NULL, &ref_time_dummy) < 0) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
                if (time_sync_get_mode() == TIME_SYNC_MODE_OFFSET) {
                    butler_block_handler(0, &warmup_butler_config);
                } else
#endif
                {
                    glossy_block_handler(0, &warmup_glossy_config);
                }
                sync_attempts++;
                k_msleep(5);
            }
            LOG_INF("Time sync acquired after %d sync rounds", sync_attempts);
        }
        schedule_next_block(0, 0, &exec_ctx);
    }

    return 0;
}

int block_scheduler_get_timing_stats(uint16_t *max_times_ms, uint8_t max_count)
{
    if (!max_times_ms || max_count == 0) {
        return -EINVAL;
    }

    uint8_t count = MIN(max_count, CONFIG_SYNCHROFLY_SUPERFRAME_MAX_SLOTS);
    memcpy(max_times_ms, block_max_execution_ms, count * sizeof(uint16_t));

    return count;
}

int block_scheduler_rebuild_superframe(void)
{
    /* Pause scheduler: stops timer, cancels pending work */
    time_sync_scheduler_pause();

    /* Free old superframe */
    if (exec_ctx.sframe) {
        superframe_free(exec_ctx.sframe);
        exec_ctx.sframe = NULL;
    }

    /* Build new superframe from current settings */
    struct superframe *new_sframe;
    int err = superframe_settings_build(&new_sframe);
    if (err) {
        LOG_ERR("Superframe rebuild failed: %d (scheduler paused)", err);
        return err;
    }

    /* Compute timing and reset stats */
    compute_superframe_timing(new_sframe);
    block_scheduler_reset_timing_stats();

    /* Install new superframe into exec context */
    exec_ctx.sframe = new_sframe;
    exec_ctx.last_executed_block = -1;
    exec_ctx.warmup_complete = time_sync_is_free_running();

#if IS_ENABLED(CONFIG_SYNCHROFLY_SWARM_RANGING)
    sync_swarm_ranging_state(new_sframe);
#endif

    LOG_INF("Superframe rebuilt: %u blocks, %u ms total, free_running=%d",
            new_sframe->block_count, new_sframe->total_duration_ms,
            time_sync_is_free_running());

    /* Resume scheduler: restarts scheduling chain via saved callback */
    time_sync_scheduler_resume();

    return 0;
}
