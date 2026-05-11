#include <app/lib/ranging/radial_velocity.h>
#include <app/lib/ranging/mm_constants.h>
#include <math.h>
#include <errno.h>

#define MAX_WINDOW 64

int radial_velocity_from_phase_series(const ts_series_t *phase_series,
                                      const ts_series_t *rtc_series,
                                      uint8_t channel,
                                      size_t window,
                                      struct radial_velocity_result *result)
{
    if (!phase_series || !rtc_series || !result || window < 2 || window > MAX_WINDOW) {
        return -EINVAL;
    }

    double lambda = get_channel_wavelength(channel);
    if (lambda <= 0.0) {
        return -EINVAL;
    }

    size_t phase_len = ts_length(phase_series);
    size_t rtc_len = ts_length(rtc_series);
    if (phase_len < window || rtc_len < window) {
        result->valid = false;
        return -ENODATA;
    }

    double phase_buf[MAX_WINDOW];
    double rtc_buf[MAX_WINDOW];

    size_t n_phase = ts_copy_last(phase_series, window, phase_buf);
    size_t n_rtc = ts_copy_last(rtc_series, window, rtc_buf);
    if (n_phase < window || n_rtc < window) {
        result->valid = false;
        return -ENODATA;
    }

    /* Unwrap phase in-place (numpy-style) */
    for (size_t i = 1; i < window; i++) {
        double diff = phase_buf[i] - phase_buf[i - 1];
        if (diff > MM_PI) {
            phase_buf[i] -= 2.0 * MM_PI;
        } else if (diff < -MM_PI) {
            phase_buf[i] += 2.0 * MM_PI;
        }
    }

    /* Finite difference: newest minus oldest */
    double dphi = phase_buf[window - 1] - phase_buf[0];
    double dt_ticks = rtc_buf[window - 1] - rtc_buf[0];
    double dt_s = dt_ticks / (double)INTERNAL_CLOCK_FREQ;

    if (dt_s < 1e-6) {
        result->valid = false;
        return -ENODATA;
    }

    /*
     * v = -(dphi / dt) * lam_half / (2*pi)
     * where lam_half = lambda / 2  (two-way ranging halves effective wavelength)
     */
    double lam_half = lambda / 2.0;
    double velocity = -(dphi / dt_s) * lam_half / (2.0 * MM_PI);

    result->velocity_ms = (float)velocity;
    result->dt_s = (float)dt_s;
    result->valid = true;
    return 0;
}
