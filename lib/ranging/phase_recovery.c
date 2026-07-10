#include <app/lib/ranging/phase_recovery.h>
#include <app/lib/ranging/mm_constants.h>
#include <app/lib/ranging/time_series_store.h>
#include <app/lib/ranging/time_series_filters.h>
#include <app/lib/ranging/twr.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <errno.h>

LOG_MODULE_REGISTER(phase_recovery, CONFIG_PHASE_RECOVERY_LOG_LEVEL);

#include <app/lib/ranging/ts_series_ids.h>

double _mod(double a, double b) {
    double result = fmod(a, b);
    if (result < 0) {
        result += b;
    }
    return result;
}

double wrap_to_pi(double angle) {
    while (angle > MM_PI) angle -= 2*MM_PI;
    while (angle < -MM_PI) angle += 2*MM_PI;
    return angle;
}


// Pure phase computation functions (new interface)
double compute_coarse_phase_mm(double phase_poll, double phase_resp)
{
    // Coarse phase recovery: combine poll and response phases
    // This matches the Python reference: (poll_phase + resp_phase) % (2 * π)
    double combined_phase = phase_poll + phase_resp;

    // Use _mod function for consistent behavior with Python % operator
    double result = _mod(combined_phase, 2.0 * MM_PI);

    LOG_DBG("Coarse phase computation: poll=%.6f, resp=%.6f, combined=%.6f",
           phase_poll, phase_resp, result);

    return result;
}

double compute_fine_phase_mm(double coarse_phase, double phase_final, double phase_post_final,
                            uint16_t initiator_id, uint16_t responder_id, uint8_t channel)
{
    // Fine phase recovery using final and post-final phases
    // This matches the Python reference implementation using _mod to match Python % behavior
    double residual_phase_raw = _mod(phase_final - phase_post_final, 2.0 * MM_PI);
    double residual_phase = residual_phase_raw;  // Default to raw value

#ifdef CONFIG_MM_RANGING_ENABLE_RESIDUAL_PHASE_FILTERING
    // Store residual phase in time series if filtering is enabled and node IDs are provided
    if (initiator_id != 0 || responder_id != 0) {
        // Create unique time series key including channel for multi-channel support
        ts_series_key_t key = {
            .initiator_id = initiator_id,
            .responder_id = responder_id,
            .series_id = TS_ID_RESIDUAL_PHASE + channel  // Channel-specific residual phase series
        };

        ts_series_t *series = ts_get_or_create(&key, mm_get_time_series_size());
        if (series) {
            // Add raw residual phase to time series
            if (ts_append(series, residual_phase_raw) == 0) {
                // Apply filtering based on configuration
                int ret = -1;

#ifdef CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY
                ret = ts_filter_savgol(series,
                                     CONFIG_MM_RANGING_FILTER_WINDOW_SIZE,
                                     CONFIG_SAVITZKY_GOLAY_POLY_ORDER,
                                     &residual_phase);
#else
                ret = ts_filter_moving_average(series,
                                             CONFIG_MM_RANGING_FILTER_WINDOW_SIZE,
                                             &residual_phase);
#endif

                if (ret < 0) {
                    LOG_DBG("Residual phase filter failed (ret=%d), using raw value", ret);
                    residual_phase = residual_phase_raw;
                }
            } else {
                LOG_WRN("Failed to add residual phase to time series");
            }
        } else {
            LOG_WRN("Failed to get/create residual phase time series for pair %u-%u ch%u",
                   initiator_id, responder_id, channel);
        }
    }
#endif

    double fine_phase = _mod(coarse_phase - residual_phase, 2.0 * MM_PI);

    LOG_DBG("Fine phase computation: coarse=%.6f, final=%.6f, post_final=%.6f, residual_raw=%.6f, residual_filtered=%.6f, fine=%.6f",
           coarse_phase, phase_final, phase_post_final, residual_phase_raw, residual_phase, fine_phase);

    return fine_phase;
}

int compute_fine_phase_mm_detailed(double coarse_phase, double phase_final, double phase_post_final,
                                  uint16_t initiator_id, uint16_t responder_id, uint8_t channel,
                                  struct fine_phase_result *result)
{
    if (!result) {
        return -EINVAL;
    }
    
    // Fine phase recovery using final and post-final phases
    // This matches the Python reference implementation using _mod to match Python % behavior
    double residual_phase_raw = _mod(phase_final - phase_post_final, 2.0 * MM_PI);
    double residual_phase = residual_phase_raw;  // Default to raw value
    
#ifdef CONFIG_MM_RANGING_ENABLE_RESIDUAL_PHASE_FILTERING
    // Store residual phase in time series if filtering is enabled and node IDs are provided
    if (initiator_id != 0 || responder_id != 0) {
        // Create unique time series key including channel for multi-channel support
        ts_series_key_t key = {
            .initiator_id = initiator_id,
            .responder_id = responder_id,
            .series_id = TS_ID_RESIDUAL_PHASE + channel  // Channel-specific residual phase series
        };
        ts_series_t *series = ts_get_or_create(&key, mm_get_time_series_size());
        if (series) {
            // Add raw residual phase to time series
            if (ts_append(series, residual_phase_raw) == 0) {
                // Apply filtering based on configuration
                int ret = -1;
#ifdef CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY
                ret = ts_filter_savgol(series,
                                     CONFIG_MM_RANGING_FILTER_WINDOW_SIZE,
                                     CONFIG_SAVITZKY_GOLAY_POLY_ORDER,
                                     &residual_phase);
#else
                ret = ts_filter_moving_average(series,
                                             CONFIG_MM_RANGING_FILTER_WINDOW_SIZE,
                                             &residual_phase);
#endif
                if (ret < 0) {
                    LOG_DBG("Residual phase filter failed (ret=%d), using raw value", ret);
                    residual_phase = residual_phase_raw;
                }
            } else {
                LOG_WRN("Failed to add residual phase to time series");
            }
        } else {
            LOG_WRN("Failed to get/create residual phase time series for pair %u-%u ch%u",
                   initiator_id, responder_id, channel);
        }
    }
#endif

    double fine_phase = _mod(coarse_phase - residual_phase, 2.0 * MM_PI);
    
    // Fill the result structure
    result->coarse_phase = coarse_phase;
    result->residual_phase_raw = residual_phase_raw;
    result->residual_phase_smoothed = residual_phase;
    result->fine_phase = fine_phase;
    
    LOG_DBG("Detailed fine phase computation: coarse=%.6f, final=%.6f, post_final=%.6f, residual_raw=%.6f, residual_smoothed=%.6f, fine=%.6f",
           coarse_phase, phase_final, phase_post_final, residual_phase_raw, residual_phase, fine_phase);
    
    return 0;
}


double correct_cfo_phase(double raw_phase, double cfo_ppm, double time_of_flight)
{
    if (fabs(cfo_ppm) < 1e-9) {  // Avoid division by very small numbers
        LOG_DBG("CFO correction: negligible CFO (%.3f ppm), no correction needed", cfo_ppm);
        return raw_phase;
    }

    // Convert CFO from ppm to frequency offset
    // CFO correction based on time-of-flight and frequency offset
    double cfo_phase_error = 2 * MM_PI * (cfo_ppm * 1e-6) * time_of_flight;

    // Subtract the CFO-induced phase error
    double corrected_phase = raw_phase - cfo_phase_error;

    // Wrap to [0, 2π] range
    corrected_phase = WRAP_PHASE(corrected_phase);


    LOG_DBG("CFO correction: raw=%.6f, cfo_ppm=%.3f, tof=%.9f, error=%.6f, corrected=%.6f",
           raw_phase, cfo_ppm, time_of_flight, cfo_phase_error, corrected_phase);

    return corrected_phase;
}

