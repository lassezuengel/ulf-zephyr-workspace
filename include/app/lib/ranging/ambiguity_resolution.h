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
 * @brief Resolve range ambiguity using dual-channel phase measurements
 *
 * Uses individual phase measurements from two UWB channels to resolve the integer
 * number of wavelengths and calculate mm-accurate distance. Implements the
 * proven Python algorithm with dual-channel averaging for maximum accuracy.
 *
 * @param coarse_distance_twr Coarse distance from TWR (meters)
 * @param filtered_phase_ch5 Filtered phase measurement from channel 5 (radians)
 * @param filtered_phase_ch3 Filtered phase measurement from channel 3 (radians)
 * @param channel_5 Channel 5 number (typically 5)
 * @param channel_3 Channel 3 number (typically 3)
 * @param result Output structure for mm-accurate distance result
 * @return 0 on success, negative error code on failure
 */
int resolve_ambiguity_dual_channel(double coarse_distance_twr,
                                  double filtered_phase_ch5,
                                  double filtered_phase_ch3,
                                  uint8_t channel_5, uint8_t channel_3,
                                  struct mm_distance_result *result);

/**
 * @brief Time-series based dual-channel ambiguity resolution
 *
 * Uses recent histories of coarse TWR distance and per-channel phases to:
 *  - smooth coarse TWR distance (SG)
 *  - compute d_phase_diff sequence and anchor to smoothed TWR within ±λ/2
 *  - smooth d_phase_diff (SG) and compute final per-channel distances averaged
 *
 * @param s_twr   Time series of coarse TWR distances (meters)
 * @param s_phi_a Time series of channel A fine phases (radians, wrapped)
 * @param s_phi_b Time series of channel B fine phases (radians, wrapped)
 * @param initiator_id Node ID of the initiator for time series key generation
 * @param responder_id Node ID of the responder for time series key generation
 * @param channel_a Channel A number
 * @param channel_b Channel B number
 * @param window_phase SG window for phases (odd, >= 5)
 * @param poly_phase   SG polynomial order for phases
 * @param window_diff  SG window for d_phase_diff smoothing (odd, >= 5)
 * @param poly_diff    SG polynomial order for d_phase_diff
 * @param result Output distance result
 * @return 0 on success, negative on error
 */
int resolve_ambiguity_dual_channel_ts(const ts_series_t *s_twr,
                                      const ts_series_t *s_phi_a,
                                      const ts_series_t *s_phi_b,
                                      uint16_t initiator_id, uint16_t responder_id,
                                      uint8_t channel_a, uint8_t channel_b,
                                      size_t window_phase, uint8_t poly_phase,
                                      size_t window_diff, uint8_t poly_diff,
                                      struct mm_distance_result *result);

/**
 * @brief Validate mm-accurate distance result
 *
 * Performs sanity checks on the ambiguity resolution result including
 * confidence thresholds, distance error bounds, and reasonableness checks.
 *
 * @param result MM distance result to validate
 * @param coarse_distance_twr Original coarse TWR distance for comparison
 * @param max_distance_error Maximum allowed error vs TWR distance (meters)
 * @param min_confidence Minimum required confidence level (0-1)
 * @return True if result passes validation, false otherwise
 */
bool validate_mm_distance_result(const struct mm_distance_result *result,
                                double coarse_distance_twr,
                                double max_distance_error,
                                double min_confidence);

#endif // AMBIGUITY_RESOLUTION_H
