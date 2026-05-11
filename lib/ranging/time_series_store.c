#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include <app/lib/ranging/time_series_store.h>

LOG_MODULE_REGISTER(time_series_store, CONFIG_LOG_DEFAULT_LEVEL);

#ifndef CONFIG_MM_TS_DEFAULT_CAPACITY
#define CONFIG_MM_TS_DEFAULT_CAPACITY 10
#endif

struct ts_series {
    ts_series_key_t key;
    double *buffer;
    size_t capacity;
    size_t head;   // next write index
    size_t count;  // current number of stored samples (<= capacity)
    struct ts_series *next;
};

static struct ts_series *g_head = NULL;

static bool keys_equal(const ts_series_key_t *a, const ts_series_key_t *b)
{
    return a->initiator_id == b->initiator_id &&
           a->responder_id == b->responder_id &&
           a->series_id == b->series_id;
}

static struct ts_series *find_series(const ts_series_key_t *key)
{
    for (struct ts_series *cur = g_head; cur; cur = cur->next) {
        if (keys_equal(&cur->key, key)) return cur;
    }
    return NULL;
}

ts_series_t *ts_get_or_create(const ts_series_key_t *key, size_t capacity)
{
    if (!key) return NULL;
    if (capacity == 0) capacity = CONFIG_MM_TS_DEFAULT_CAPACITY;

    struct ts_series *s = find_series(key);
    if (s) return s;

    s = k_malloc(sizeof(struct ts_series));
    if (!s) {
        LOG_ERR("ts: alloc series failed");
        return NULL;
    }
    memset(s, 0, sizeof(*s));
    s->key = *key;
    s->capacity = capacity;
    s->buffer = k_malloc(capacity * sizeof(double));
    if (!s->buffer) {
        LOG_ERR("ts: alloc buffer failed (cap=%zu)", capacity);
        k_free(s);
        return NULL;
    }
    memset(s->buffer, 0, capacity * sizeof(double));

    // insert at head
    s->next = g_head;
    g_head = s;
    return s;
}

int ts_append(ts_series_t *series, double value)
{
    if (!series || !series->buffer || series->capacity == 0) return -EINVAL;
    series->buffer[series->head] = value;
    series->head = (series->head + 1) % series->capacity;
    if (series->count < series->capacity) series->count++;
    return 0;
}

size_t ts_copy_last(const ts_series_t *series, size_t max_len, double *out)
{
    if (!series || !out || max_len == 0 || series->count == 0 || !series->buffer) return 0;
    size_t n = series->count < max_len ? series->count : max_len;
    // start index of oldest among the n elements
    size_t start = (series->head + series->capacity - n) % series->capacity;
    for (size_t i = 0; i < n; i++) {
        out[i] = series->buffer[(start + i) % series->capacity];
    }
    return n;
}

bool ts_last(const ts_series_t *series, double *out_value)
{
    if (!series || series->count == 0 || !out_value || !series->buffer) return false;
    size_t last_index = (series->head + series->capacity - 1) % series->capacity;
    *out_value = series->buffer[last_index];
    return true;
}

void ts_clear(ts_series_t *series)
{
    if (!series || !series->buffer) return;
    memset(series->buffer, 0, series->capacity * sizeof(double));
    series->head = 0;
    series->count = 0;
}

void ts_free_pair(uint16_t initiator_id, uint16_t responder_id)
{
    struct ts_series **pp = &g_head;
    while (*pp) {
        struct ts_series *cur = *pp;
        if (cur->key.initiator_id == initiator_id && cur->key.responder_id == responder_id) {
            *pp = cur->next;
            if (cur->buffer) k_free(cur->buffer);
            k_free(cur);
        } else {
            pp = &cur->next;
        }
    }
}

size_t ts_length(const ts_series_t *series)
{
    return series ? series->count : 0;
}

size_t ts_capacity(const ts_series_t *series)
{
    return series ? series->capacity : 0;
}

ts_series_t *ts_create(size_t capacity)
{
    if (capacity == 0) capacity = CONFIG_MM_TS_DEFAULT_CAPACITY;

    struct ts_series *s = k_malloc(sizeof(struct ts_series));
    if (!s) {
        LOG_ERR("ts_create: alloc series failed");
        return NULL;
    }
    memset(s, 0, sizeof(*s));
    s->capacity = capacity;
    s->buffer = k_malloc(capacity * sizeof(double));
    if (!s->buffer) {
        LOG_ERR("ts_create: alloc buffer failed (cap=%zu)", capacity);
        k_free(s);
        return NULL;
    }
    memset(s->buffer, 0, capacity * sizeof(double));
    /* NOT inserted into g_head -- standalone ownership */
    return s;
}

void ts_destroy(ts_series_t *series)
{
    if (!series) return;
    if (series->buffer) {
        k_free(series->buffer);
    }
    k_free(series);
}

int ts_resize(ts_series_t *series, size_t new_capacity)
{
    if (!series || new_capacity == 0) return -EINVAL;
    if (new_capacity == series->capacity) return 0;

    double *new_buf = k_malloc(new_capacity * sizeof(double));
    if (!new_buf) {
        LOG_ERR("ts_resize: alloc failed (cap=%zu)", new_capacity);
        return -ENOMEM;
    }
    memset(new_buf, 0, new_capacity * sizeof(double));

    if (series->buffer) {
        k_free(series->buffer);
    }
    series->buffer = new_buf;
    series->capacity = new_capacity;
    series->head = 0;
    series->count = 0;
    return 0;
}

/* ========================================================================== */
/* Float time series implementation                                           */
/* ========================================================================== */

struct ts_series_float {
    ts_series_key_t key;
    float *buffer;
    size_t capacity;
    size_t head;   // next write index
    size_t count;  // current number of stored samples (<= capacity)
    struct ts_series_float *next;
};

static struct ts_series_float *g_float_head = NULL;

static struct ts_series_float *find_float_series(const ts_series_key_t *key)
{
    for (struct ts_series_float *cur = g_float_head; cur; cur = cur->next) {
        if (keys_equal(&cur->key, key)) return cur;
    }
    return NULL;
}

ts_series_float_t *ts_float_get_or_create(const ts_series_key_t *key, size_t capacity)
{
    if (!key) return NULL;
    if (capacity == 0) capacity = CONFIG_MM_TS_DEFAULT_CAPACITY;

    struct ts_series_float *s = find_float_series(key);
    if (s) return s;

    s = k_malloc(sizeof(struct ts_series_float));
    if (!s) {
        LOG_ERR("ts_float: alloc series failed");
        return NULL;
    }
    memset(s, 0, sizeof(*s));
    s->key = *key;
    s->capacity = capacity;
    s->buffer = k_malloc(capacity * sizeof(float));
    if (!s->buffer) {
        LOG_ERR("ts_float: alloc buffer failed (cap=%zu)", capacity);
        k_free(s);
        return NULL;
    }
    memset(s->buffer, 0, capacity * sizeof(float));

    // insert at head
    s->next = g_float_head;
    g_float_head = s;
    return s;
}

int ts_float_append(ts_series_float_t *series, float value)
{
    if (!series || !series->buffer || series->capacity == 0) return -EINVAL;
    series->buffer[series->head] = value;
    series->head = (series->head + 1) % series->capacity;
    if (series->count < series->capacity) series->count++;
    return 0;
}

size_t ts_float_copy_last(const ts_series_float_t *series, size_t max_len, float *out)
{
    if (!series || !out || max_len == 0 || series->count == 0 || !series->buffer) return 0;
    size_t n = series->count < max_len ? series->count : max_len;
    // start index of oldest among the n elements
    size_t start = (series->head + series->capacity - n) % series->capacity;
    for (size_t i = 0; i < n; i++) {
        out[i] = series->buffer[(start + i) % series->capacity];
    }
    return n;
}

bool ts_float_last(const ts_series_float_t *series, float *out_value)
{
    if (!series || series->count == 0 || !out_value || !series->buffer) return false;
    size_t last_index = (series->head + series->capacity - 1) % series->capacity;
    *out_value = series->buffer[last_index];
    return true;
}

void ts_float_clear(ts_series_float_t *series)
{
    if (!series || !series->buffer) return;
    memset(series->buffer, 0, series->capacity * sizeof(float));
    series->head = 0;
    series->count = 0;
}

void ts_float_free_pair(uint16_t initiator_id, uint16_t responder_id)
{
    struct ts_series_float **pp = &g_float_head;
    while (*pp) {
        struct ts_series_float *cur = *pp;
        if (cur->key.initiator_id == initiator_id && cur->key.responder_id == responder_id) {
            *pp = cur->next;
            if (cur->buffer) k_free(cur->buffer);
            k_free(cur);
        } else {
            pp = &cur->next;
        }
    }
}

size_t ts_float_length(const ts_series_float_t *series)
{
    return series ? series->count : 0;
}

size_t ts_float_capacity(const ts_series_float_t *series)
{
    return series ? series->capacity : 0;
}

int ts_float_resize(ts_series_float_t *series, size_t new_capacity)
{
    if (!series || new_capacity == 0) return -EINVAL;
    if (new_capacity == series->capacity) return 0;

    float *new_buf = k_malloc(new_capacity * sizeof(float));
    if (!new_buf) {
        LOG_ERR("ts_float_resize: alloc failed (cap=%zu)", new_capacity);
        return -ENOMEM;
    }
    memset(new_buf, 0, new_capacity * sizeof(float));

    if (series->buffer) {
        k_free(series->buffer);
    }
    series->buffer = new_buf;
    series->capacity = new_capacity;
    series->head = 0;
    series->count = 0;
    return 0;
}

ts_series_float_t *ts_float_create(size_t capacity)
{
    if (capacity == 0) capacity = CONFIG_MM_TS_DEFAULT_CAPACITY;

    struct ts_series_float *s = k_malloc(sizeof(struct ts_series_float));
    if (!s) {
        LOG_ERR("ts_float_create: alloc series failed");
        return NULL;
    }
    memset(s, 0, sizeof(*s));
    s->capacity = capacity;
    s->buffer = k_malloc(capacity * sizeof(float));
    if (!s->buffer) {
        LOG_ERR("ts_float_create: alloc buffer failed (cap=%zu)", capacity);
        k_free(s);
        return NULL;
    }
    memset(s->buffer, 0, capacity * sizeof(float));
    /* NOT inserted into g_float_head -- standalone ownership */
    return s;
}

void ts_float_destroy(ts_series_float_t *series)
{
    if (!series) return;
    if (series->buffer) {
        k_free(series->buffer);
    }
    k_free(series);
}
