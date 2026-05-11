#ifndef AMBIGUITY_RESOLUTION_H
#define AMBIGUITY_RESOLUTION_H

#include <stdint.h>
#include <stdbool.h>
#include <app/lib/ranging/twr.h>
#include <app/lib/ranging/time_series_store.h>

/**
 * @file ambiguity_resolution.h
 * @brief Ambiguity resolution algorithms for mm-accurate UWB ranging
 *
 * Implements dual-channel and single-channel ambiguity resolution based
 * on the nils-nicklaus Python implementation. Uses phase differences
 * between channels to resolve integer wavelength ambiguities.
 */

/**
 * @brief Time-series based dual-channel ambiguity resolution
 *
 * Uses recent histories of coarse TWR distance and per-channel phases to:
 *  - smooth coarse TWR distance
 *  - compute d_phase_diff sequence and anchor to smoothed TWR within +/-lambda/2
 *  - smooth d_phase_diff and compute final per-channel distances averaged
 *
 * All filters use window_size=1 as passthrough (no filtering).
 * poly_order=0 selects moving average, poly_order>0 selects Savitzky-Golay.
 *
 * @param s_twr   Time series of coarse TWR distances (meters)
 * @param s_phi_a Time series of channel A fine phases (radians, wrapped)
 * @param s_phi_b Time series of channel B fine phases (radians, wrapped)
 * @param s_d_diff Time series for d_phase_diff (or NULL for global store)
 * @param initiator_id Node ID of the initiator for time series key generation
 * @param responder_id Node ID of the responder for time series key generation
 * @param channel_a Channel A number
 * @param channel_b Channel B number
 * @param window_phase Window for phase filtering (1=passthrough)
 * @param poly_phase   Polynomial order for phases (0=movavg)
 * @param window_twr   Window for TWR distance smoothing (1=passthrough)
 * @param poly_twr     Polynomial order for TWR (0=movavg)
 * @param window_diff  Window for d_phase_diff smoothing (1=passthrough)
 * @param poly_diff    Polynomial order for d_phase_diff (0=movavg)
 * @param result Output distance result
 * @return 0 on success, negative on error
 */
int resolve_ambiguity_dual_channel_ts(const ts_series_t *s_twr,
                                      const ts_series_t *s_phi_a,
                                      const ts_series_t *s_phi_b,
                                      ts_series_t *s_d_diff,
                                      uint16_t initiator_id, uint16_t responder_id,
                                      uint8_t channel_a, uint8_t channel_b,
                                      size_t window_phase, uint8_t poly_phase,
                                      size_t window_twr, uint8_t poly_twr,
                                      size_t window_diff, uint8_t poly_diff,
                                      struct mm_distance_result *result);

/**
 * @brief Validate mm-accurate distance result
 *
 * Performs sanity checks on the ambiguity resolution result including
 * distance error bounds and reasonableness checks.
 *
 * @param result MM distance result to validate
 * @param coarse_distance_twr Original coarse TWR distance for comparison
 * @param max_distance_error Maximum allowed error vs TWR distance (meters)
 * @return True if result passes validation, false otherwise
 */
bool validate_mm_distance_result(const struct mm_distance_result *result,
                                double coarse_distance_twr,
                                double max_distance_error);

#endif // AMBIGUITY_RESOLUTION_H
