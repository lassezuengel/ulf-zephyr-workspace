#include <app/lib/ranging/ambiguity_resolution.h>
#include <app/lib/ranging/mm_constants.h>
#include <app/lib/ranging/phase_recovery.h>
#include <app/lib/ranging/time_series_store.h>
#include <app/lib/ranging/time_series_filters.h>
#include <app/lib/ranging/twr.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(ambiguity_resolution, CONFIG_AMBIGUITY_RESOLUTION_LOG_LEVEL);

#include <app/lib/ranging/ts_series_ids.h>

int resolve_ambiguity_dual_channel_ts(const ts_series_t *s_twr,
                                      const ts_series_t *s_phi_a,
                                      const ts_series_t *s_phi_b,
                                      ts_series_t *s_d_diff,
                                      uint16_t initiator_id, uint16_t responder_id,
                                      uint8_t channel_a, uint8_t channel_b,
                                      size_t window_phase, uint8_t poly_phase,
                                      size_t window_twr, uint8_t poly_twr,
                                      size_t window_diff, uint8_t poly_diff,
                                      struct mm_distance_result *result)
{
    if (!s_twr || !s_phi_a || !s_phi_b || !result) {
        LOG_ERR("resolve_ts: NULL input");
        return -EINVAL;
    }

    // Ensure we have enough recent samples
    size_t n_twr = ts_length(s_twr);
    size_t n_a = ts_length(s_phi_a);
    size_t n_b = ts_length(s_phi_b);
    size_t n_min = n_twr;
    if (n_a < n_min) n_min = n_a;
    if (n_b < n_min) n_min = n_b;

    if (n_min == 0) return -ENODATA;

    double phi_a_last, phi_b_last, twr_sm;

    // TWR filtering (window_twr=1 means passthrough via filter functions)
    int rc;
    if (poly_twr > 0) {
        rc = ts_filter_savgol(s_twr, window_twr, poly_twr, &twr_sm);
    } else {
        rc = ts_filter_moving_average(s_twr, window_twr, &twr_sm);
    }
    if (rc != 0) {
        if (!ts_last(s_twr, &twr_sm)) return rc;
    }

    // Phase filtering with unwrap-filter-rewrap (window_phase=1 means passthrough)
    int rc_a = ts_filter_phase(s_phi_a, window_phase, poly_phase, &phi_a_last);
    if (rc_a != 0) {
        if (!ts_last(s_phi_a, &phi_a_last)) return -ENODATA;
    }
    int rc_b = ts_filter_phase(s_phi_b, window_phase, poly_phase, &phi_b_last);
    if (rc_b != 0) {
        if (!ts_last(s_phi_b, &phi_b_last)) return -ENODATA;
    }

    // Calculate wavelengths and lambda_new
    double lambda_a = get_channel_wavelength(channel_a)*0.5;
    double lambda_b = get_channel_wavelength(channel_b)*0.5;
    double lambda_new = get_diff_wavelength(channel_a, channel_b)*0.5;
    if (lambda_a <= 0 || lambda_b <= 0 || !isfinite(lambda_new) || lambda_new <= 0) {
        LOG_ERR("resolve_ts: invalid wavelengths a=%.6f b=%.6f new=%.6f", lambda_a, lambda_b, lambda_new);
        return -EINVAL;
    }

    // Compute d_diff from phase difference anchored to TWR
    double delta_phi = _mod(phi_a_last - phi_b_last, 2.0*MM_PI);
    int n_diff = (int)floor(twr_sm / lambda_new);
    double d_diff = ((2.0*MM_PI-delta_phi) / (2.0 * MM_PI) + n_diff) * lambda_new;

    // Cycle-slip correction: keep d_diff within +/-lambda_new/2 of TWR
    /* double diff_error = d_diff - twr_sm; */
    /* if (diff_error < (-lambda_new / 2.0)) { */
    /*     d_diff += lambda_new; */
    /* } else if (diff_error > lambda_new / 2.0) { */
    /*     d_diff -= lambda_new; */
    /* } */

    // d_diff filtering (window_diff=1 means passthrough via filter functions)
    double d_diff_smoothed = d_diff;

    if (!s_d_diff) {
        ts_series_key_t key_d_diff = { initiator_id, responder_id, TS_ID_D_PHASE_DIFF };
        s_d_diff = ts_get_or_create(&key_d_diff, mm_get_time_series_size());
    }

    if (s_d_diff) {
        ts_append(s_d_diff, d_diff);

        int rc_filter;
        if (poly_diff > 0) {
            rc_filter = ts_filter_savgol(s_d_diff, window_diff, poly_diff, &d_diff_smoothed);
        } else {
            rc_filter = ts_filter_moving_average(s_d_diff, window_diff, &d_diff_smoothed);
        }
        if (rc_filter != 0) {
            d_diff_smoothed = d_diff;
            LOG_WRN("d_diff filtering failed, using current value: %.6f", d_diff_smoothed);
        }
    } else {
        LOG_WRN("Failed to create d_diff time series, using current computed value");
    }

    // Per-channel distance from phases anchored to d_diff_smoothed
    int N_a = (int)floor(d_diff_smoothed / lambda_a);
    double d_a = ((2*MM_PI-phi_a_last) / (2.0 * MM_PI) + N_a) * lambda_a;
    double err_a = d_a - d_diff_smoothed;
    if (err_a < -(lambda_a / 2.0)) {
        d_a += lambda_a;
    } else if (err_a > lambda_a / 2.0) {
        d_a -= lambda_a;
    }

    int N_b = (int)floor(d_diff_smoothed / lambda_b);
    double d_b = ((2*MM_PI-phi_b_last) / (2.0 * MM_PI) + N_b) * lambda_b;
    double err_b = d_b - d_diff_smoothed;
    if (err_b < -(lambda_b / 2.0)) {
        d_b += lambda_b;
    } else if (err_b > lambda_b / 2.0) {
        d_b -= lambda_b;
    }

    double distance_mm = (d_a + d_b) / 2.0;

    memset(result, 0, sizeof(*result));
    result->distance_mm = (float)distance_mm;
    result->lambda_diff = lambda_new;
    result->n_diff = (int)floor(d_diff_smoothed / lambda_new);

    result->distance_twr_smoothed = twr_sm;
    result->phase_diff_smoothed = delta_phi;
    result->d_diff_smoothed = d_diff_smoothed;
    result->d_diff_minus = d_diff_smoothed - lambda_new;
    result->d_diff_plus = d_diff_smoothed + lambda_new;
    result->phase_a_filtered = phi_a_last;
    result->phase_b_filtered = phi_b_last;

    LOG_INF("TS ambiguity: d_twr_sm=%.6f, d_diff_sm=%.6f, phi_diff_sm=%.6f, d_a=%.6f, d_b=%.6f, final=%.6f",
            twr_sm, d_diff_smoothed, delta_phi, d_a, d_b, distance_mm);

    return 0;
}

bool validate_mm_distance_result(const struct mm_distance_result *result,
                                double coarse_distance_twr,
                                double max_distance_error)
{
    if (!result) {
        LOG_ERR("Result is NULL");
        return false;
    }

    // Check distance error
    double distance_error = fabs((double)result->distance_mm - coarse_distance_twr);
    if (distance_error > max_distance_error) {
        LOG_WRN("Large distance error: %.6f > %.6f", distance_error, max_distance_error);
        return false;
    }

    // Check for reasonable distance values
    if ((double)result->distance_mm < 0.0 || (double)result->distance_mm > 1000.0) {  // 1km max
        LOG_WRN("Unreasonable distance: %.6f m", (double)result->distance_mm);
        return false;
    }

    // Check lambda_diff is positive
    if (result->lambda_diff <= 0.0) {
        LOG_ERR("Invalid lambda_diff: %.6f", result->lambda_diff);
        return false;
    }

    LOG_DBG("MM distance result validation passed: "
           "dist=%.6f, error=%.6f, lambda_diff=%.6f",
           (double)result->distance_mm, distance_error, result->lambda_diff);

    return true;
}
