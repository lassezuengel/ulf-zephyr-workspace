#ifndef TIME_SERIES_FILTERS_H
#define TIME_SERIES_FILTERS_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <app/lib/ranging/time_series_store.h>

// Apply Savitzky–Golay to the last `window_size` samples of a series.
// - window_size will be coerced to odd if even.
// - poly_order must be < window_size.
// - On success, writes filtered center value to *out_value and returns 0.
// - Returns -ENODATA if not enough samples, -EINVAL for bad args, -E2BIG for window too large.
int ts_filter_savgol(const ts_series_t *series,
                     size_t window_size,
                     uint8_t poly_order,
                     double *out_value);

// Simple moving average over the last `window_size` samples.
// Returns 0 on success, -ENODATA if not enough samples, -EINVAL on bad args.
int ts_filter_moving_average(const ts_series_t *series,
                             size_t window_size,
                             double *out_value);

// Phase-aware filter: unwrap -> filter (savgol or movavg) -> rewrap to [0, 2pi].
// poly_order > 0: Savitzky-Golay. poly_order == 0: moving average.
// window_size coerced to odd. Returns 0 on success.
int ts_filter_phase(const ts_series_t *series,
                    size_t window_size,
                    uint8_t poly_order,
                    double *out_value);

/* ========================================================================== */
/* Float time series filter functions                                         */
/* ========================================================================== */

// Apply Savitzky-Golay to the last `window_size` samples of a float series.
// - window_size will be coerced to odd if even.
// - poly_order must be < window_size.
// - On success, writes filtered center value to *out_value and returns 0.
// - Returns -ENODATA if not enough samples, -EINVAL for bad args, -E2BIG for window too large.
int ts_filter_savgol_float(const ts_series_float_t *series,
                           size_t window_size,
                           uint8_t poly_order,
                           float *out_value);

// Simple moving average over the last `window_size` samples of a float series.
// Returns 0 on success, -ENODATA if not enough samples, -EINVAL on bad args.
int ts_filter_moving_average_float(const ts_series_float_t *series,
                                   size_t window_size,
                                   float *out_value);

#endif // TIME_SERIES_FILTERS_H

