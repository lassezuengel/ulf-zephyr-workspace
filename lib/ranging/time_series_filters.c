#include <string.h>
#include <errno.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include <app/lib/ranging/time_series_filters.h>
#include <app/lib/ranging/mm_constants.h>
#include <app/lib/ranging/phase_utils.h>
#ifdef CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY
#include "iterative/savgolFilter.h"
#else
#ifndef MAX_WINDOW
#define MAX_WINDOW 33
#endif
#endif

LOG_MODULE_REGISTER(ts_filters, CONFIG_LOG_DEFAULT_LEVEL);

int ts_filter_moving_average(const ts_series_t *series,
                             size_t window_size,
                             double *out_value)
{
    if (!series || !out_value || window_size == 0) return -EINVAL;
    if (window_size <= 1) { if (!ts_last(series, out_value)) return -ENODATA; return 0; }
    if (ts_length(series) < window_size) return -ENODATA;

    // Copy last window_size samples
    static double buf[16];
    if (window_size > (sizeof(buf)/sizeof(buf[0]))) return -E2BIG;
    size_t n = ts_copy_last(series, window_size, buf);
    if (n < window_size) return -ENODATA;

    double sum = 0.0;
    for (size_t i = 0; i < n; i++) sum += buf[i];
    *out_value = sum / (double)n;
    return 0;
}

#ifdef CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY
int ts_filter_savgol(const ts_series_t *series,
                     size_t window_size,
                     uint8_t poly_order,
                     double *out_value)
{
    if (!series || !out_value || window_size == 0) return -EINVAL;
    if (window_size <= 1) { if (!ts_last(series, out_value)) return -ENODATA; return 0; }
    if (ts_length(series) < window_size) return -ENODATA;

    // Coerce to odd window and validate final size
    if ((window_size % 2) == 0) {
        window_size += 1;
    }
    if (window_size > MAX_WINDOW) return -E2BIG;
    if (poly_order >= window_size) return -EINVAL;

    // Static buffers - safe since function won't be called concurrently
    static double tmp[ MAX_WINDOW ];
    static MqsRawDataPoint_t in[ MAX_WINDOW ];
    static MqsRawDataPoint_t out[ MAX_WINDOW ];

    size_t n = ts_copy_last(series, window_size, tmp);
    if (n < window_size) return -ENODATA;

    for (size_t i = 0; i < window_size; i++) {
        in[i].phaseAngle = (float)tmp[i];
        out[i].phaseAngle = 0.0f;
    }

    uint8_t half = (uint8_t)((window_size - 1) / 2);
    int rc = mes_savgolFilter(in, window_size, half, out, poly_order, 0 /*centered*/, 0);
    if (rc != 0) {
        LOG_WRN("ts_filter_savgol failed: %d (win=%zu, poly=%u)", rc, window_size, poly_order);
        return -EINVAL;
    }

    *out_value = (double)out[half].phaseAngle;
    return 0;
}
#else
int ts_filter_savgol(const ts_series_t *series,
                     size_t window_size,
                     uint8_t poly_order,
                     double *out_value)
{
    if (!series || !out_value || window_size == 0) return -EINVAL;
    if (window_size <= 1) { if (!ts_last(series, out_value)) return -ENODATA; return 0; }
    (void)poly_order;
    return -ENOTSUP;
}
#endif

/* ========================================================================== */
/* Phase-aware filter: unwrap -> filter -> rewrap                             */
/* ========================================================================== */

int ts_filter_phase(const ts_series_t *series, size_t window_size,
                    uint8_t poly_order, double *out_value)
{
    if (!series || !out_value || window_size == 0) return -EINVAL;
    if (window_size <= 1) { if (!ts_last(series, out_value)) return -ENODATA; return 0; }
    if (ts_length(series) < window_size) return -ENODATA;

    if ((window_size % 2) == 0) {
        window_size += 1;
    }
    if (window_size > MAX_WINDOW) return -E2BIG;
    if (poly_order > 0 && poly_order >= window_size) return -EINVAL;

    static double wrapped_buf[MAX_WINDOW];
    static double unwrapped_buf[MAX_WINDOW];

    size_t n = ts_copy_last(series, window_size, wrapped_buf);
    if (n < window_size) return -ENODATA;

    phase_unwrap(wrapped_buf, unwrapped_buf, window_size);

    double filtered;

#ifdef CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY
    uint8_t half = (uint8_t)((window_size - 1) / 2);
    if (poly_order > 0) {
        static MqsRawDataPoint_t sg_in[MAX_WINDOW];
        static MqsRawDataPoint_t sg_out[MAX_WINDOW];

        for (size_t i = 0; i < window_size; i++) {
            sg_in[i].phaseAngle = (float)unwrapped_buf[i];
            sg_out[i].phaseAngle = 0.0f;
        }

        int rc = mes_savgolFilter(sg_in, window_size, half, sg_out,
                                  poly_order, 0, 0);
        if (rc != 0) {
            LOG_WRN("ts_filter_phase savgol failed: %d", rc);
            return -EINVAL;
        }
        filtered = (double)sg_out[half].phaseAngle;
    } else
#else
    (void)poly_order;
#endif
    {
        /* Moving average on unwrapped data */
        double sum = 0.0;
        for (size_t i = 0; i < window_size; i++) sum += unwrapped_buf[i];
        filtered = sum / (double)window_size;
    }

    *out_value = phase_wrap(filtered);
    return 0;
}

/* ========================================================================== */
/* Float time series filter implementations                                   */
/* ========================================================================== */

int ts_filter_moving_average_float(const ts_series_float_t *series,
                                   size_t window_size,
                                   float *out_value)
{
    if (!series || !out_value || window_size == 0) return -EINVAL;
    if (window_size <= 1) { if (!ts_float_last(series, out_value)) return -ENODATA; return 0; }
    if (ts_float_length(series) < window_size) return -ENODATA;

    // Copy last window_size samples
    static float buf[34];
    if (window_size > (sizeof(buf)/sizeof(buf[0]))) return -E2BIG;
    size_t n = ts_float_copy_last(series, window_size, buf);
    if (n < window_size) return -ENODATA;

    float sum = 0.0f;
    for (size_t i = 0; i < n; i++) sum += buf[i];
    *out_value = sum / (float)n;
    return 0;
}

#ifdef CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY
int ts_filter_savgol_float(const ts_series_float_t *series,
                           size_t window_size,
                           uint8_t poly_order,
                           float *out_value)
{
    if (!series || !out_value || window_size == 0) return -EINVAL;
    if (window_size <= 1) { if (!ts_float_last(series, out_value)) return -ENODATA; return 0; }
    if (ts_float_length(series) < window_size) return -ENODATA;

    // Coerce to odd window and validate final size
    if ((window_size % 2) == 0) {
        window_size += 1;
    }
    if (window_size > MAX_WINDOW) return -E2BIG;
    if (poly_order >= window_size) return -EINVAL;

    // Static buffers - safe since function won't be called concurrently
    static float tmp[MAX_WINDOW];
    static MqsRawDataPoint_t in[MAX_WINDOW];
    static MqsRawDataPoint_t out[MAX_WINDOW];

    size_t n = ts_float_copy_last(series, window_size, tmp);
    if (n < window_size) return -ENODATA;

    // MqsRawDataPoint_t uses float phaseAngle - direct copy, no conversion
    for (size_t i = 0; i < window_size; i++) {
        in[i].phaseAngle = tmp[i];
        out[i].phaseAngle = 0.0f;
    }

    uint8_t half = (uint8_t)((window_size - 1) / 2);
    int rc = mes_savgolFilter(in, window_size, half, out, poly_order, 0 /*centered*/, 0);
    if (rc != 0) {
        LOG_WRN("ts_filter_savgol_float failed: %d (win=%zu, poly=%u)", rc, window_size, poly_order);
        return -EINVAL;
    }

    *out_value = out[half].phaseAngle;
    return 0;
}
#else
int ts_filter_savgol_float(const ts_series_float_t *series,
                           size_t window_size,
                           uint8_t poly_order,
                           float *out_value)
{
    if (!series || !out_value || window_size == 0) return -EINVAL;
    if (window_size <= 1) { if (!ts_float_last(series, out_value)) return -ENODATA; return 0; }
    (void)poly_order;
    return -ENOTSUP;
}
#endif
