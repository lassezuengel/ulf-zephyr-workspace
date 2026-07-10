/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Node table implementation
 */

#include <app/lib/node_table/node_table.h>
#include <app/lib/system/node.h>
#include <app/lib/stats/firmware_stats.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <string.h>
#include <errno.h>

#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
#include <app/lib/ranging/time_series_filters.h>
#endif

LOG_MODULE_REGISTER(node_table, CONFIG_NODE_TABLE_LOG_LEVEL);

#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
/* Runtime configuration for distance filtering */
static enum distance_filter_strategy filter_strategy =
    IS_ENABLED(CONFIG_NODE_TABLE_DISTANCE_FILTER_ENABLED)
        ? DISTANCE_FILTER_SAVITZKY_GOLAY
        : DISTANCE_FILTER_NONE;

#if defined(CONFIG_NODE_TABLE_DISTANCE_FILTER_WINDOW_SIZE)
static uint8_t filter_window_size = CONFIG_NODE_TABLE_DISTANCE_FILTER_WINDOW_SIZE;
#else
static uint8_t filter_window_size = 5;
#endif

#if defined(CONFIG_NODE_TABLE_DISTANCE_FILTER_POLY_ORDER)
static uint8_t filter_poly_order = CONFIG_NODE_TABLE_DISTANCE_FILTER_POLY_ORDER;
#else
static uint8_t filter_poly_order = 2;
#endif

static uint8_t distance_series_capacity = CONFIG_NODE_TABLE_DISTANCE_SERIES_CAPACITY;

static bool outlier_filter_enabled =
    IS_ENABLED(CONFIG_NODE_TABLE_OUTLIER_FILTER_ENABLED);
static uint8_t outlier_filter_threshold = CONFIG_NODE_TABLE_OUTLIER_FILTER_THRESHOLD;

#endif /* CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED */

#define NODE_ID_INVALID 0xFFFF

/* Eviction timer: 0 = disabled, >0 = timeout in ms */
static uint32_t eviction_timeout_ms = CONFIG_NODE_TABLE_EVICTION_TIMEOUT_MS;

static void node_table_evict_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(node_table_evict_work, node_table_evict_work_handler);

static struct node_entry entries[CONFIG_NODE_TABLE_MAX_ENTRIES];
static size_t entry_count = 0;
static node_table_changed_cb_t change_callback = NULL;

/* Dirty tracking for deferred notifications (notify=false) */
static uint16_t dirty_nodes[CONFIG_NODE_TABLE_MAX_ENTRIES];
static size_t dirty_count;

K_MUTEX_DEFINE(node_table_mutex);

/* Internal: mark a node as dirty for deferred notification */
static void mark_dirty_unlocked(uint16_t node_id)
{
    /* Deduplicate */
    for (size_t i = 0; i < dirty_count; i++) {
        if (dirty_nodes[i] == node_id) {
            return;
        }
    }
    if (dirty_count < CONFIG_NODE_TABLE_MAX_ENTRIES) {
        dirty_nodes[dirty_count++] = node_id;
    }
}

/* Internal: find entry by node_id, returns NULL if not found */
static struct node_entry *find_entry_unlocked(uint16_t node_id)
{
    for (size_t i = 0; i < entry_count; i++) {
        if (entries[i].node_id == node_id) {
            return &entries[i];
        }
    }
    return NULL;
}

/* Internal: create new entry, returns NULL if table full or if node_id is self */
static struct node_entry *create_entry_unlocked(uint16_t node_id)
{
    /* Never add ourselves to the node table */
    if (node_id == get_node_addr()) {
        return NULL;
    }

    if (entry_count >= CONFIG_NODE_TABLE_MAX_ENTRIES) {
        LOG_WRN("Node table full, cannot add 0x%04x", node_id);
        return NULL;
    }

    struct node_entry *entry = &entries[entry_count];
    memset(entry, 0, sizeof(*entry));
    entry->node_id = node_id;
    entry_count++;

    LOG_DBG("Created entry for node 0x%04x (count=%zu)", node_id, entry_count);
    return entry;
}

/* Internal: free heap resources owned by entry */
static void free_entry_resources(struct node_entry *entry)
{
    if (!entry) return;
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    if (entry->distance_series) {
        ts_float_destroy(entry->distance_series);
        entry->distance_series = NULL;
    }
    if (entry->filtered_distance_series) {
        ts_float_destroy(entry->filtered_distance_series);
        entry->filtered_distance_series = NULL;
    }
#endif
}

/* Internal: remove entry by swapping with last */
static void remove_entry_unlocked(struct node_entry *entry)
{
    if (!entry || entry_count == 0) {
        return;
    }

    uint16_t removed_id = entry->node_id;
    size_t index = entry - entries;

    if (index >= entry_count) {
        return;
    }

    /* Free heap resources before overwriting */
    free_entry_resources(entry);

    /* Swap with last entry if not already last */
    if (index < entry_count - 1) {
        memcpy(entry, &entries[entry_count - 1], sizeof(*entry));
    }

    entry_count--;
    LOG_DBG("Removed node 0x%04x (count=%zu)", removed_id, entry_count);

    mark_dirty_unlocked(removed_id);
}

int node_table_init(void)
{
    k_mutex_lock(&node_table_mutex, K_FOREVER);

    memset(entries, 0, sizeof(entries));
    for (size_t i = 0; i < CONFIG_NODE_TABLE_MAX_ENTRIES; i++) {
        entries[i].node_id = NODE_ID_INVALID;
    }
    entry_count = 0;
    change_callback = NULL;

    k_mutex_unlock(&node_table_mutex);

    LOG_DBG("Node table initialized (max %d entries)", CONFIG_NODE_TABLE_MAX_ENTRIES);

    /* Start eviction timer if configured */
    if (eviction_timeout_ms > 0) {
        k_work_reschedule(&node_table_evict_work,
                          K_MSEC(eviction_timeout_ms / 2));
        LOG_INF("Node table eviction started (%u ms)", eviction_timeout_ms);
    }

    return 0;
}

#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
/**
 * Check if a new sample should be rejected as an outlier.
 * Uses Mean Absolute Deviation (MAD) from the series mean.
 * Returns true if the sample should be REJECTED.
 */
static bool is_outlier(const ts_series_float_t *series, float new_value)
{
    if (!outlier_filter_enabled) {
        return false;
    }

    size_t n = ts_float_length(series);
    if (n < 3) {
        return false;
    }

    float buf[64];
    size_t count = ts_float_copy_last(series, (n < 64) ? n : 64, buf);

    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        sum += buf[i];
    }
    float mean = sum / (float)count;

    float mad_sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        float diff = buf[i] - mean;
        mad_sum += (diff < 0.0f) ? -diff : diff;
    }
    float mad = mad_sum / (float)count;

    if (mad < 0.001f) {
        return false;
    }

    float deviation = new_value - mean;
    if (deviation < 0.0f) {
        deviation = -deviation;
    }

    float threshold = (float)outlier_filter_threshold / 10.0f;
    return deviation > (threshold * mad);
}
#endif /* CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED */

int node_table_update(uint16_t node_id, int32_t distance_mm, uint64_t rtc)
{
    int ret = 0;
    bool is_new = false;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        entry = create_entry_unlocked(node_id);
        is_new = true;
    }
    if (!entry) {
        ret = -ENOMEM;
        goto out;
    }

#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    /* Lazily create raw distance series */
    if (!entry->distance_series) {
        entry->distance_series = ts_float_create(distance_series_capacity);
        if (!entry->distance_series) {
            LOG_ERR("Failed to create distance series for 0x%04x", node_id);
        }
    }
    /* Lazily create filtered (outlier-gated) distance series */
    if (!entry->filtered_distance_series) {
        entry->filtered_distance_series = ts_float_create(distance_series_capacity);
        if (!entry->filtered_distance_series) {
            LOG_ERR("Failed to create filtered distance series for 0x%04x", node_id);
        }
    }

    if (entry->distance_series) {
        float distance_m = (float)distance_mm / 1000.0f;

        /* Evaluate outlier BEFORE appending to raw (uncontaminated stats) */
        bool rejected = is_outlier(entry->distance_series, distance_m);

        /* Raw series: ALWAYS append */
        ts_float_append(entry->distance_series, distance_m);

        /* Filtered series: append only non-outliers */
        if (entry->filtered_distance_series) {
            if (rejected) {
                LOG_WRN("Outlier rejected 0x%04x: %.3f m (> %.1fx MAD)",
                        node_id, (double)distance_m,
                        (double)((float)outlier_filter_threshold / 10.0f));
            } else {
                ts_float_append(entry->filtered_distance_series, distance_m);
            }
        }
    }
#else
    entry->last_distance_mm = distance_mm;
#endif
    entry->last_seen_rtc = rtc;
    /* Mark as neighbor since we have direct ranging */
    entry->flags |= NODE_TABLE_FLAG_NEIGHBOR;

    if (is_new) {
        LOG_DBG("New neighbor 0x%04x: dist=%dmm", node_id, distance_mm);
    } else {
        LOG_DBG("Update 0x%04x: dist=%dmm", node_id, distance_mm);
    }

    mark_dirty_unlocked(node_id);

    firmware_stats_inc(STAT_ID_NT_UPDATES_TOTAL);
    firmware_stats_inc(STAT_ID_NT_MEAS_COUNT(node_id));

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

int node_table_update_twr(uint16_t node_id, const struct node_twr_timestamps *twr)
{
    int ret = 0;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        ret = -ENOENT;
        goto out;
    }

    memcpy(&entry->last_twr, twr, sizeof(*twr));

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

int node_table_update_hop_count(uint16_t node_id, uint8_t hop_count, uint64_t rtc)
{
    int ret = 0;
    bool is_new = false;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        entry = create_entry_unlocked(node_id);
        is_new = true;
    }
    if (!entry) {
        ret = -ENOMEM;
        goto out;
    }

    entry->hop_count = hop_count;
    entry->last_seen_rtc = rtc;

    if (hop_count == 0) {
        entry->flags |= NODE_TABLE_FLAG_NEIGHBOR;
    }

    if (is_new) {
        LOG_DBG("New node 0x%04x: hops=%d", node_id, hop_count);
    } else {
        LOG_DBG("Update 0x%04x: hops=%d", node_id, hop_count);
    }

    mark_dirty_unlocked(node_id);

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

int node_table_update_rssi(uint16_t node_id, int8_t rssi, uint64_t rtc)
{
    int ret = 0;
    bool is_new = false;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        entry = create_entry_unlocked(node_id);
        is_new = true;
    }
    if (!entry) {
        ret = -ENOMEM;
        goto out;
    }

    entry->rssi = rssi;
    entry->last_seen_rtc = rtc;

    if (is_new) {
        LOG_DBG("New node 0x%04x: rssi=%ddBm", node_id, rssi);
    } else {
        LOG_DBG("Update 0x%04x: rssi=%ddBm", node_id, rssi);
    }

    mark_dirty_unlocked(node_id);

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

int node_table_set_flags(uint16_t node_id, uint8_t flags)
{
    int ret = 0;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        ret = -ENOENT;
        goto out;
    }

    entry->flags |= flags;
    mark_dirty_unlocked(node_id);

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

int node_table_clear_flags(uint16_t node_id, uint8_t flags)
{
    int ret = 0;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        ret = -ENOENT;
        goto out;
    }

    entry->flags &= ~flags;
    mark_dirty_unlocked(node_id);

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

struct node_entry *node_table_get(uint16_t node_id)
{
    struct node_entry *entry;

    k_mutex_lock(&node_table_mutex, K_FOREVER);
    entry = find_entry_unlocked(node_id);
    k_mutex_unlock(&node_table_mutex);

    return entry;
}

int node_table_get_copy(uint16_t node_id, struct node_entry *out)
{
    if (!out) {
        return -EINVAL;
    }

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        k_mutex_unlock(&node_table_mutex);
        return -ENOENT;
    }

    memcpy(out, entry, sizeof(*out));

    k_mutex_unlock(&node_table_mutex);
    return 0;
}

bool node_table_exists(uint16_t node_id)
{
    bool exists;

    k_mutex_lock(&node_table_mutex, K_FOREVER);
    exists = (find_entry_unlocked(node_id) != NULL);
    k_mutex_unlock(&node_table_mutex);

    return exists;
}

size_t node_table_get_count(void)
{
    size_t count;

    k_mutex_lock(&node_table_mutex, K_FOREVER);
    count = entry_count;
    k_mutex_unlock(&node_table_mutex);

    return count;
}

size_t node_table_get_all(struct node_entry *out, size_t max)
{
    size_t copied;

    if (!out || max == 0) {
        return 0;
    }

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    copied = (entry_count < max) ? entry_count : max;
    memcpy(out, entries, copied * sizeof(struct node_entry));

    k_mutex_unlock(&node_table_mutex);

    return copied;
}

size_t node_table_get_neighbors(struct node_entry *out, size_t max)
{
    size_t copied = 0;

    if (!out || max == 0) {
        return 0;
    }

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    for (size_t i = 0; i < entry_count && copied < max; i++) {
        if ((entries[i].flags & NODE_TABLE_FLAG_NEIGHBOR) ||
            entries[i].hop_count == 0) {
            memcpy(&out[copied], &entries[i], sizeof(struct node_entry));
            copied++;
        }
    }

    k_mutex_unlock(&node_table_mutex);

    return copied;
}

size_t node_table_get_anchors(struct node_entry *out, size_t max)
{
    size_t copied = 0;

    if (!out || max == 0) {
        return 0;
    }

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    for (size_t i = 0; i < entry_count && copied < max; i++) {
        if (entries[i].flags & NODE_TABLE_FLAG_ANCHOR) {
            memcpy(&out[copied], &entries[i], sizeof(struct node_entry));
            copied++;
        }
    }

    k_mutex_unlock(&node_table_mutex);

    return copied;
}

int node_table_update_position(uint16_t node_id, float x, float y, float z, uint64_t rtc)
{
    int ret = 0;
    bool is_new = false;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        entry = create_entry_unlocked(node_id);
        is_new = true;
    }
    if (!entry) {
        ret = -ENOMEM;
        goto out;
    }

    entry->pos_x = x;
    entry->pos_y = y;
    entry->pos_z = z;
    entry->last_seen_rtc = rtc;
    entry->flags |= NODE_TABLE_FLAG_ANCHOR;

    if (is_new) {
        LOG_DBG("New anchor 0x%04x: pos=(%.3f, %.3f, %.3f)", node_id, (double)x, (double)y, (double)z);
    } else {
        LOG_DBG("Update anchor 0x%04x: pos=(%.3f, %.3f, %.3f)", node_id, (double)x, (double)y, (double)z);
    }

    mark_dirty_unlocked(node_id);

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

struct node_entry *node_table_get_by_index(size_t index)
{
    struct node_entry *entry = NULL;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    if (index < entry_count) {
        entry = &entries[index];
    }

    k_mutex_unlock(&node_table_mutex);

    return entry;
}

void node_table_expire(uint64_t current_rtc, uint64_t timeout_ticks)
{
    k_mutex_lock(&node_table_mutex, K_FOREVER);

    /* Iterate backwards to handle removal during iteration */
    for (size_t i = entry_count; i > 0; i--) {
        struct node_entry *entry = &entries[i - 1];
        uint64_t age = current_rtc - entry->last_seen_rtc;

        if (age > timeout_ticks) {
            LOG_DBG("Expiring node 0x%04x (age=%llu ticks)", entry->node_id, age);
            firmware_stats_inc(STAT_ID_NT_EVICTIONS);
            remove_entry_unlocked(entry);
        }
    }

    k_mutex_unlock(&node_table_mutex);
}

void node_table_remove(uint16_t node_id)
{
    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (entry) {
        remove_entry_unlocked(entry);
    }

    k_mutex_unlock(&node_table_mutex);
}

void node_table_clear(void)
{
    k_mutex_lock(&node_table_mutex, K_FOREVER);

    /* Free resources for each removed entry */
    for (size_t i = 0; i < entry_count; i++) {
        free_entry_resources(&entries[i]);
    }

    memset(entries, 0, sizeof(entries));
    for (size_t i = 0; i < CONFIG_NODE_TABLE_MAX_ENTRIES; i++) {
        entries[i].node_id = NODE_ID_INVALID;
    }
    entry_count = 0;

    k_mutex_unlock(&node_table_mutex);

    LOG_DBG("Node table cleared");
}

void node_table_register_callback(node_table_changed_cb_t cb)
{
    k_mutex_lock(&node_table_mutex, K_FOREVER);
    change_callback = cb;
    k_mutex_unlock(&node_table_mutex);
}

void node_table_notify_changed(void)
{
    uint16_t ids[CONFIG_NODE_TABLE_MAX_ENTRIES];
    size_t count;

    /* Copy dirty IDs under mutex, then clear dirty set */
    k_mutex_lock(&node_table_mutex, K_FOREVER);
    count = dirty_count;
    memcpy(ids, dirty_nodes, count * sizeof(uint16_t));
    dirty_count = 0;
    k_mutex_unlock(&node_table_mutex);

    /* Fire callbacks outside mutex with a COPY of each entry.
     * Previous code passed a bare pointer from node_table_get() which
     * could be invalidated by concurrent updates (swap-with-last removal,
     * field overwrites from other threads). */
    if (!change_callback) {
        return;
    }
    for (size_t i = 0; i < count; i++) {
        struct node_entry snapshot;
        if (node_table_get_copy(ids[i], &snapshot) == 0) {
            change_callback(ids[i], &snapshot);
        }
    }
}

size_t node_table_get_node_ids(uint16_t *out, size_t max)
{
    if (!out || max == 0) {
        return 0;
    }

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    size_t copied = (entry_count < max) ? entry_count : max;
    for (size_t i = 0; i < copied; i++) {
        out[i] = entries[i].node_id;
    }

    k_mutex_unlock(&node_table_mutex);

    /* Sort for deterministic ordering across all nodes */
    for (size_t i = 0; i + 1 < copied; i++) {
        for (size_t j = i + 1; j < copied; j++) {
            if (out[i] > out[j]) {
                uint16_t tmp = out[i];
                out[i] = out[j];
                out[j] = tmp;
            }
        }
    }

    return copied;
}

/* ========================================================================== */
/* Distance time series API implementation                                    */
/* ========================================================================== */

int node_table_get_distance_m(uint16_t node_id, float *distance_m)
{
    if (!distance_m) {
        return -EINVAL;
    }

    int ret = 0;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        ret = -ENOENT;
        goto out;
    }

#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    if (!entry->distance_series || !ts_float_last(entry->distance_series, distance_m)) {
        ret = -ENODATA;
        goto out;
    }
#else
    if (entry->last_distance_mm == 0) {
        ret = -ENODATA;
        goto out;
    }
    *distance_m = (float)entry->last_distance_mm / 1000.0f;
#endif

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

int node_table_get_distance_mm(uint16_t node_id, int32_t *distance_mm)
{
    float distance_m;
    int ret = node_table_get_distance_m(node_id, &distance_m);
    if (ret == 0) {
        *distance_mm = (int32_t)(distance_m * 1000.0f);
    }
    return ret;
}

int node_table_get_filtered_distance_m(uint16_t node_id, float *distance_m)
{
    if (!distance_m) {
        return -EINVAL;
    }

    int ret = 0;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        ret = -ENOENT;
        goto out;
    }

#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    /* Pick best series for filtering:
     * - If outlier filter active and filtered series has data, use it
     * - Otherwise fall back to raw series
     */
    ts_series_float_t *source = entry->filtered_distance_series;
    if (!source || !outlier_filter_enabled || ts_float_length(source) == 0) {
        source = entry->distance_series;
    }
    if (!source) {
        ret = -ENODATA;
        goto out;
    }

    /* Try to apply filter based on strategy */
    if (filter_strategy != DISTANCE_FILTER_NONE) {
        int filter_ret = -ENOTSUP;
        size_t series_len = ts_float_length(source);

        switch (filter_strategy) {
        case DISTANCE_FILTER_MOVING_AVERAGE:
            filter_ret = ts_filter_moving_average_float(
                source, filter_window_size, distance_m);
            break;
        case DISTANCE_FILTER_SAVITZKY_GOLAY:
            filter_ret = ts_filter_savgol_float(
                source, filter_window_size,
                filter_poly_order, distance_m);
            break;
        default:
            break;
        }

        if (filter_ret == 0) {
            /* Filter succeeded */
            goto out;
        }
        /* Filter failed (not enough samples, etc.) - fall back to raw */
        LOG_WRN("Filter strategy=%d failed for 0x%04x (ret=%d, samples=%zu, win=%u, cap=%u)",
                filter_strategy, node_id, filter_ret, series_len,
                filter_window_size, distance_series_capacity);
    }

    /* Return latest value from source series */
    if (!ts_float_last(source, distance_m)) {
        ret = -ENODATA;
        goto out;
    }
#else
    if (entry->last_distance_mm == 0) {
        ret = -ENODATA;
        goto out;
    }
    *distance_m = (float)entry->last_distance_mm / 1000.0f;
#endif

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

int node_table_get_filtered_distance_mm(uint16_t node_id, int32_t *distance_mm)
{
    float distance_m;
    int ret = node_table_get_filtered_distance_m(node_id, &distance_m);
    if (ret == 0) {
        *distance_mm = (int32_t)(distance_m * 1000.0f);
    }
    return ret;
}

void node_table_set_distance_filter_enabled(bool enabled)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    filter_strategy = enabled ? DISTANCE_FILTER_SAVITZKY_GOLAY : DISTANCE_FILTER_NONE;
    uint8_t val = (uint8_t)filter_strategy;
    settings_save_one("ntfilt/strategy", &val, sizeof(val));
    LOG_INF("Distance filter %s (strategy=%d)", enabled ? "enabled" : "disabled",
            filter_strategy);
#else
    (void)enabled;
    LOG_WRN("Distance series not enabled, filter setting ignored");
#endif
}

bool node_table_get_distance_filter_enabled(void)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    return filter_strategy != DISTANCE_FILTER_NONE;
#else
    return false;
#endif
}

void node_table_set_distance_filter_strategy(enum distance_filter_strategy strategy)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    filter_strategy = strategy;
    uint8_t val = (uint8_t)strategy;
    settings_save_one("ntfilt/strategy", &val, sizeof(val));
    LOG_INF("Distance filter strategy set to %d", strategy);
#else
    (void)strategy;
    LOG_WRN("Distance series not enabled, filter strategy ignored");
#endif
}

enum distance_filter_strategy node_table_get_distance_filter_strategy(void)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    return filter_strategy;
#else
    return DISTANCE_FILTER_NONE;
#endif
}

void node_table_set_distance_filter_window_size(uint8_t window_size)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    if (window_size < 3) window_size = 3;
    if (window_size > 33) window_size = 33;
    filter_window_size = window_size;
    settings_save_one("ntfilt/win", &filter_window_size, sizeof(filter_window_size));
    LOG_INF("Filter window size set to %u", window_size);
#else
    (void)window_size;
#endif
}

uint8_t node_table_get_distance_filter_window_size(void)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    return filter_window_size;
#else
    return 0;
#endif
}

void node_table_set_distance_filter_poly_order(uint8_t poly_order)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    if (poly_order < 1) poly_order = 1;
    if (poly_order > 5) poly_order = 5;
    filter_poly_order = poly_order;
    settings_save_one("ntfilt/poly", &filter_poly_order, sizeof(filter_poly_order));
    LOG_INF("Filter poly order set to %u", poly_order);
#else
    (void)poly_order;
#endif
}

uint8_t node_table_get_distance_filter_poly_order(void)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    return filter_poly_order;
#else
    return 0;
#endif
}

void node_table_set_distance_series_capacity(uint8_t capacity)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    if (capacity < 1) capacity = 1;
    if (capacity > 64) capacity = 64;

    distance_series_capacity = capacity;
    settings_save_one("ntfilt/cap", &distance_series_capacity, sizeof(distance_series_capacity));

    /* Resize all existing series (both raw and filtered) */
    k_mutex_lock(&node_table_mutex, K_FOREVER);
    for (size_t i = 0; i < entry_count; i++) {
        if (entries[i].distance_series) {
            int ret = ts_float_resize(entries[i].distance_series, capacity);
            if (ret != 0) {
                LOG_ERR("Failed to resize series for 0x%04x (ret=%d)",
                        entries[i].node_id, ret);
            }
        }
        if (entries[i].filtered_distance_series) {
            int ret = ts_float_resize(entries[i].filtered_distance_series, capacity);
            if (ret != 0) {
                LOG_ERR("Failed to resize filtered series for 0x%04x (ret=%d)",
                        entries[i].node_id, ret);
            }
        }
    }
    k_mutex_unlock(&node_table_mutex);

    LOG_INF("Distance series capacity set to %u", capacity);
#else
    (void)capacity;
    LOG_WRN("Distance series not enabled, capacity setting ignored");
#endif
}

uint8_t node_table_get_distance_series_capacity(void)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    return distance_series_capacity;
#else
    return 0;
#endif
}

/* ========================================================================== */
/* Outlier pre-filter API implementation                                      */
/* ========================================================================== */

void node_table_set_outlier_filter_enabled(bool enabled)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    outlier_filter_enabled = enabled;
    uint8_t val = enabled ? 1 : 0;
    settings_save_one("ntfilt/outlier", &val, sizeof(val));
    LOG_INF("Outlier pre-filter %s", enabled ? "enabled" : "disabled");
#else
    (void)enabled;
#endif
}

bool node_table_get_outlier_filter_enabled(void)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    return outlier_filter_enabled;
#else
    return false;
#endif
}

void node_table_set_outlier_filter_threshold(uint8_t threshold_x10)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    if (threshold_x10 < 10) threshold_x10 = 10;
    if (threshold_x10 > 100) threshold_x10 = 100;
    outlier_filter_threshold = threshold_x10;
    settings_save_one("ntfilt/outlier_th", &outlier_filter_threshold,
                      sizeof(outlier_filter_threshold));
    LOG_INF("Outlier threshold set to %.1fx MAD",
            (double)((float)threshold_x10 / 10.0f));
#else
    (void)threshold_x10;
#endif
}

uint8_t node_table_get_outlier_filter_threshold(void)
{
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    return outlier_filter_threshold;
#else
    return 30;
#endif
}

/* ========================================================================== */
/* MM ranging data API implementation                                         */
/* ========================================================================== */

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)

int node_table_update_mm(uint16_t node_id, const struct node_mm_update *mm)
{
    if (!mm) return -EINVAL;

    int ret = 0;

    k_mutex_lock(&node_table_mutex, K_FOREVER);

    struct node_entry *entry = find_entry_unlocked(node_id);
    if (!entry) {
        LOG_WRN("update_mm: node 0x%04x not in table", node_id);
        ret = -ENOENT;
        goto out;
    }

    LOG_DBG("update_mm 0x%04x: twr=%d um ch=%u phase=%d mrad",
            node_id, mm->twr_distance_um, mm->channel, mm->phase_mrad);

    entry->last_twr_distance_um = mm->twr_distance_um;
    entry->last_phase_mrad = mm->phase_mrad;
    entry->last_coarse_phase_mrad = mm->coarse_phase_mrad;
    entry->last_mm_channel = mm->channel;
    entry->last_cfo_ppb = mm->cfo_ppb;

    mark_dirty_unlocked(node_id);

out:
    k_mutex_unlock(&node_table_mutex);
    return ret;
}

/*
 * Legacy stubs -- kept so twr_mm.c and network_settings_service.c still link.
 * No-ops: time series are no longer stored in the node table.
 */
#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
ts_series_t *node_table_get_mm_twr_series(uint16_t node_id) { (void)node_id; return NULL; }
ts_series_t *node_table_get_mm_phase_series(uint16_t node_id, int ch) { (void)node_id; (void)ch; return NULL; }
ts_series_t *node_table_get_mm_d_diff_series(uint16_t node_id) { (void)node_id; return NULL; }
#endif

void node_table_set_mm_filter_window_size(uint8_t size) { (void)size; }
uint8_t node_table_get_mm_filter_window_size(void) { return 0; }
void node_table_set_mm_filter_poly_order(uint8_t order) { (void)order; }
uint8_t node_table_get_mm_filter_poly_order(void) { return 0; }
void node_table_set_mm_series_capacity(uint8_t capacity) { (void)capacity; }
uint8_t node_table_get_mm_series_capacity(void) { return 0; }
void node_table_set_mm_phase_filter_window_size(uint8_t size) { (void)size; }
uint8_t node_table_get_mm_phase_filter_window_size(void) { return 0; }
void node_table_set_mm_phase_filter_poly_order(uint8_t order) { (void)order; }
uint8_t node_table_get_mm_phase_filter_poly_order(void) { return 0; }
void node_table_set_mm_twr_filter_window_size(uint8_t size) { (void)size; }
uint8_t node_table_get_mm_twr_filter_window_size(void) { return 0; }
void node_table_set_mm_twr_filter_poly_order(uint8_t order) { (void)order; }
uint8_t node_table_get_mm_twr_filter_poly_order(void) { return 0; }

#endif /* CONFIG_NODE_TABLE_MM_ENABLED */

/* ========================================================================== */
/* Eviction timer API implementation                                          */
/* ========================================================================== */

static void node_table_evict_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint32_t timeout = eviction_timeout_ms;
    if (timeout == 0) {
        return;
    }

    /* Note: last_seen_rtc is populated with k_uptime_ticks() from swarm
     * ranging and with block reference time from MTM digest processing.
     * On non-root nodes these can diverge slightly due to time sync
     * corrections, but the discrepancy is sub-ms -- negligible for
     * eviction timeouts on the order of seconds. */
    uint64_t now = k_uptime_ticks();
    uint64_t timeout_ticks = k_ms_to_ticks_ceil64(timeout);

    node_table_expire(now, timeout_ticks);
    node_table_notify_changed();

    /* Reschedule at half the timeout interval */
    k_work_reschedule(&node_table_evict_work, K_MSEC(timeout / 2));
}

void node_table_set_eviction_timeout_ms(uint32_t timeout_ms)
{
    eviction_timeout_ms = timeout_ms;

    if (timeout_ms == 0) {
        k_work_cancel_delayable(&node_table_evict_work);
        LOG_INF("Node table eviction disabled");
    } else {
        k_work_reschedule(&node_table_evict_work, K_MSEC(timeout_ms / 2));
        LOG_INF("Node table eviction set to %u ms", timeout_ms);
    }
}

uint32_t node_table_get_eviction_timeout_ms(void)
{
    return eviction_timeout_ms;
}

/* ========================================================================== */
/* NVS settings persistence for filter configuration                          */
/* ========================================================================== */

#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)

#define NTFILT_SETTINGS_LOAD(name, field) \
    if (settings_name_steq(key, name, &next) && !next) { \
        if (len != sizeof(field)) return -EINVAL; \
        return read_cb(cb_arg, &field, sizeof(field)); \
    }

static int ntfilt_settings_set(const char *key, size_t len,
                               settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    uint8_t u8val;

    /* strategy is stored as uint8_t but filter_strategy is an enum (4 bytes) */
    if (settings_name_steq(key, "strategy", &next) && !next) {
        if (len != sizeof(u8val)) return -EINVAL;
        int rc = read_cb(cb_arg, &u8val, sizeof(u8val));
        if (rc < 0) return rc;
        filter_strategy = (enum distance_filter_strategy)u8val;
        return 0;
    }

    /* outlier_enabled is stored as uint8_t (0/1) */
    if (settings_name_steq(key, "outlier", &next) && !next) {
        if (len != sizeof(u8val)) return -EINVAL;
        int rc = read_cb(cb_arg, &u8val, sizeof(u8val));
        if (rc < 0) return rc;
        outlier_filter_enabled = (u8val != 0);
        return 0;
    }

    NTFILT_SETTINGS_LOAD("win", filter_window_size);
    NTFILT_SETTINGS_LOAD("poly", filter_poly_order);
    NTFILT_SETTINGS_LOAD("cap", distance_series_capacity);
    NTFILT_SETTINGS_LOAD("outlier_th", outlier_filter_threshold);


    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(ntfilt, "ntfilt", NULL, ntfilt_settings_set, NULL, NULL);

#endif /* CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED */
