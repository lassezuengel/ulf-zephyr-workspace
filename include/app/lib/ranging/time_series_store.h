#ifndef TIME_SERIES_STORE_H
#define TIME_SERIES_STORE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct ts_series ts_series_t;

typedef struct {
    uint16_t initiator_id;
    uint16_t responder_id;
    uint16_t series_id; // user-defined identifier for the series kind
} ts_series_key_t;

// Create or get a time series identified by (initiator,responder,series_id).
// capacity is the maximum number of samples to retain (ring buffer).
// Returns NULL on allocation failure.
ts_series_t *ts_get_or_create(const ts_series_key_t *key, size_t capacity);

// Append a new sample value to the series (ring buffer semantics).
// Returns 0 on success, negative on error.
int ts_append(ts_series_t *series, double value);

// Copy up to max_len most recent samples into out (chronological order oldest→newest).
// Returns the number of samples copied.
size_t ts_copy_last(const ts_series_t *series, size_t max_len, double *out);

// Get the last (most recent) sample; returns true on success.
bool ts_last(const ts_series_t *series, double *out_value);

// Remove all samples in a series; keep capacity and key.
void ts_clear(ts_series_t *series);

// Free all series for a given pair (initiator,responder).
void ts_free_pair(uint16_t initiator_id, uint16_t responder_id);

// Query functions
size_t ts_length(const ts_series_t *series);
size_t ts_capacity(const ts_series_t *series);

#endif // TIME_SERIES_STORE_H
