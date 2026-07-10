#ifndef RADIAL_VELOCITY_H
#define RADIAL_VELOCITY_H

#include <stdint.h>
#include <stdbool.h>
#include <app/lib/ranging/time_series_store.h>

/**
 * @file radial_velocity.h
 * @brief Stateless radial velocity estimation from phase time series
 *
 * Pure computation module -- no internal state. Operates on existing
 * ts_series_t phase ring buffers and their RTC timestamp companions.
 *
 * Formula: v = -(dphi / dt) * (lambda/2) / (2*pi)
 * where lambda = c / f_center and the factor of 2 accounts for two-way
 * phase accumulation in ranging.
 */

struct radial_velocity_result {
    float velocity_ms;    /* m/s, positive = moving away */
    float dt_s;           /* time delta used (seconds) */
    bool valid;           /* false if insufficient data */
};

/**
 * @brief Compute radial velocity from a phase time series and its timestamp companion.
 *
 * Copies the last @p window samples from both series, unwraps phase,
 * computes finite-difference dphi/dt, and converts to velocity.
 *
 * @param phase_series  Time series of fine phase values (radians, [0, 2*pi))
 * @param rtc_series    Companion time series of RTC ticks (double-encoded uint64)
 * @param channel       UWB channel number (for wavelength calculation)
 * @param window        Number of samples for finite difference (>= 2).
 *                      Uses oldest and newest within the window.
 *                      Larger windows smooth out noise; typical range 3-11.
 * @param result        Output velocity result
 * @return 0 on success, -ENODATA if insufficient samples, -EINVAL on bad args
 */
int radial_velocity_from_phase_series(const ts_series_t *phase_series,
                                      const ts_series_t *rtc_series,
                                      uint8_t channel,
                                      size_t window,
                                      struct radial_velocity_result *result);

#endif /* RADIAL_VELOCITY_H */
