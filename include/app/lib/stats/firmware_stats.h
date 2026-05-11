/*
 * On-demand firmware statistics collection.
 *
 * Provides a central module for tracking scalar counters/gauges that are
 * activated at runtime via BLE subscription.  Only subscribed stats consume
 * a tracking slot; the hot-path (inc/set) is ISR-safe via atomics.
 *
 * Stat ID layout (uint32):
 *   [category:8][type:8][qualifier:16]
 *
 * The qualifier encodes a neighbor address for per-neighbor stats,
 * or 0x0000 for global (aggregate) stats.
 */

#ifndef FIRMWARE_STATS_H
#define FIRMWARE_STATS_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Stat ID construction ------------------------------------------------ */

#define STAT_ID(cat, type, qual) \
	(((uint32_t)(cat) << 24) | ((uint32_t)(type) << 16) | ((qual) & 0xFFFFU))
#define STAT_CAT(id)   ((uint8_t)((id) >> 24))
#define STAT_TYPE(id)  ((uint8_t)((id) >> 16))
#define STAT_QUAL(id)  ((uint16_t)(id))

/* ---- Categories ---------------------------------------------------------- */

#define STAT_CAT_RANGING      0x01
#define STAT_CAT_RADIO        0x02
#define STAT_CAT_MAC          0x03
#define STAT_CAT_SCHEDULER    0x04
#define STAT_CAT_NODE_TABLE   0x05
#define STAT_CAT_SYNC         0x06

/* ---- Ranging stats (global, qualifier=0) --------------------------------- */

#define STAT_RANGING_ROUNDS_TOTAL     0x01  /* total ranging rounds started */
#define STAT_RANGING_ROUNDS_SUCCESS   0x02  /* rounds with >= 1 measurement  */
#define STAT_RANGING_MEASUREMENTS     0x03  /* total distance measurements   */
#define STAT_RANGING_FAILED_FRAMES    0x04  /* frames that failed in digest  */

/* ---- Node table stats ---------------------------------------------------- */

#define STAT_NT_UPDATES_TOTAL   0x01  /* global: total node_table_update calls */
#define STAT_NT_EVICTIONS       0x02  /* global: node evictions */
#define STAT_NT_MEAS_COUNT      0x03  /* per-neighbor (qualifier=node_addr)    */

/* ---- Scheduler stats (global) -------------------------------------------- */

#define STAT_SCHED_BLOCKS_EXECUTED  0x01
#define STAT_SCHED_SUPERFRAMES      0x02

/* ---- Convenience macros for common stat IDs ------------------------------ */

#define STAT_ID_RANGING_ROUNDS_TOTAL    STAT_ID(STAT_CAT_RANGING, STAT_RANGING_ROUNDS_TOTAL, 0)
#define STAT_ID_RANGING_ROUNDS_SUCCESS  STAT_ID(STAT_CAT_RANGING, STAT_RANGING_ROUNDS_SUCCESS, 0)
#define STAT_ID_RANGING_MEASUREMENTS    STAT_ID(STAT_CAT_RANGING, STAT_RANGING_MEASUREMENTS, 0)
#define STAT_ID_RANGING_FAILED_FRAMES   STAT_ID(STAT_CAT_RANGING, STAT_RANGING_FAILED_FRAMES, 0)
#define STAT_ID_NT_UPDATES_TOTAL        STAT_ID(STAT_CAT_NODE_TABLE, STAT_NT_UPDATES_TOTAL, 0)
#define STAT_ID_NT_EVICTIONS            STAT_ID(STAT_CAT_NODE_TABLE, STAT_NT_EVICTIONS, 0)
#define STAT_ID_SCHED_BLOCKS_EXECUTED   STAT_ID(STAT_CAT_SCHEDULER, STAT_SCHED_BLOCKS_EXECUTED, 0)
#define STAT_ID_SCHED_SUPERFRAMES       STAT_ID(STAT_CAT_SCHEDULER, STAT_SCHED_SUPERFRAMES, 0)

/* Per-neighbor measurement count: STAT_ID(NODE_TABLE, MEAS_COUNT, neighbor_addr) */
#define STAT_ID_NT_MEAS_COUNT(addr) \
	STAT_ID(STAT_CAT_NODE_TABLE, STAT_NT_MEAS_COUNT, (addr))

/* ---- API ----------------------------------------------------------------- */

#ifdef CONFIG_FIRMWARE_STATS

/**
 * Subscribe to a stat -- allocates a tracking slot.
 * Returns 0 on success, -ENOMEM if no free slots, -EALREADY if active.
 */
int firmware_stats_subscribe(uint32_t stat_id);

/**
 * Unsubscribe -- frees the tracking slot.
 * Returns 0 on success, -ENOENT if not found.
 */
int firmware_stats_unsubscribe(uint32_t stat_id);

/**
 * Increment a counter by 1.  No-op if stat_id is not subscribed.
 * ISR-safe.
 */
void firmware_stats_inc(uint32_t stat_id);

/**
 * Add delta to a counter.  No-op if not subscribed.  ISR-safe.
 */
void firmware_stats_add(uint32_t stat_id, uint32_t delta);

/**
 * Set a gauge value.  No-op if not subscribed.  ISR-safe.
 */
void firmware_stats_set(uint32_t stat_id, uint32_t value);

/**
 * Read current value of a subscribed stat.
 * Returns 0 on success (value written to *out_value), -ENOENT if not subscribed.
 */
int firmware_stats_read(uint32_t stat_id, uint32_t *out_value);

/**
 * Reset a subscribed stat to zero.
 * Returns 0 on success, -ENOENT if not subscribed.
 */
int firmware_stats_reset(uint32_t stat_id);

/**
 * Check whether a stat is currently subscribed.
 */
bool firmware_stats_is_active(uint32_t stat_id);

/**
 * Return the number of currently active (subscribed) stats.
 */
uint8_t firmware_stats_active_count(void);

/**
 * Register a callback invoked (via k_work) whenever a stat value changes.
 * Used by the BLE service to trigger bt_gatt_notify.
 */
void firmware_stats_set_notify_callback(void (*cb)(void));

/**
 * Check if a stat is dirty (changed since last check) and atomically clear
 * the dirty flag.  Writes the current value to *out_value if dirty.
 * Returns true if the stat was dirty, false otherwise.
 */
bool firmware_stats_check_and_clear_dirty(uint32_t stat_id, uint32_t *out_value);

#else /* !CONFIG_FIRMWARE_STATS -- compile-time stubs (zero cost) */

static inline int  firmware_stats_subscribe(uint32_t id) { return -ENOSYS; }
static inline int  firmware_stats_unsubscribe(uint32_t id) { return -ENOSYS; }
static inline void firmware_stats_inc(uint32_t id) { }
static inline void firmware_stats_add(uint32_t id, uint32_t d) { }
static inline void firmware_stats_set(uint32_t id, uint32_t v) { }
static inline int  firmware_stats_read(uint32_t id, uint32_t *v) { return -ENOSYS; }
static inline int  firmware_stats_reset(uint32_t id) { return -ENOSYS; }
static inline bool firmware_stats_is_active(uint32_t id) { return false; }
static inline uint8_t firmware_stats_active_count(void) { return 0; }

#endif /* CONFIG_FIRMWARE_STATS */

#endif /* FIRMWARE_STATS_H */
