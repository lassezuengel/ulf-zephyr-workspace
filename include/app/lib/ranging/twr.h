#ifndef RANGING_ENGINE_H
#define RANGING_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>

// TODO should be moved into more general module
enum distance_unit {
    DIST_MM,
    DIST_CM,
    DIST_M,
    DIST_UNKNOWN // Added an unknown type for error handling
};

enum extraction_mode {
  EXTRACT_LOCAL_ONLY,
  EXTRACT_ALL
};

struct measurement
{
    uint16_t ranging_initiator_id, ranging_responder_id;
    float tof, tdoa;
    float tof_quality, tdoa_quality; // tdoa is also affected by rssi of the passively received frames
    double cfo;
};

struct measurement_mm
{
    uint16_t ranging_initiator_id, ranging_responder_id;
    float tof, tdoa;
    float tof_quality, tdoa_quality; // tdoa is also affected by rssi of the passively received frames
    double cfo;

    // Distance difference from phase (for backward compatibility)
    double d_diff;
    double d_diff_minus;    // d_diff - lambda_diff (n_diff - 1)
    double d_diff_plus;     // d_diff + lambda_diff (n_diff + 1)

    // Individual phase measurements per slot
    double phase_init;      // Poll phase (slot 1)
    double phase_resp;      // Response phase (slot 2)
    double phase_final;     // Final phase (slot 4)
    double phase_post_final; // Post-final phase (slot 5)

    // Dual-channel phase measurements
    double phase_chan_5;    // Channel 5 recovered phase
    double phase_chan_3;    // Channel 3 recovered phase
    double delta_phi;       // Phase difference between channels

    // Recovered and filtered phases - per channel
    double coarse_phase_ch5;    // Coarse phase recovery channel 5
    double fine_phase_ch5;      // Fine phase recovery channel 5
    double residual_phase_raw_ch5; // Raw residual phase channel 5 (final - post_final)
    double coarse_phase_ch3;    // Coarse phase recovery channel 3
    double fine_phase_ch3;      // Fine phase recovery channel 3
    double residual_phase_raw_ch3; // Raw residual phase channel 3 (final - post_final)

    // Backward compatibility (using ch5 values)
    double coarse_phase;    // Coarse phase recovery (ch5)
    double fine_phase;      // Fine phase recovery (ch5)
    double residual_phase_raw; // Raw residual phase (ch5)
    double filtered_phase;  // After windowed filtering

    // Distance estimates
    float distance_twr;     // Coarse distance from TWR (m)
    float distance_mm;      // Fine mm-accurate distance (m)

    // Range bias debugging
    float range_bias_m;     // Applied range bias correction (m)
    float distance_twr_raw; // Raw TWR distance before bias correction (m)

    // Channel information
    uint8_t channel_5;      // Channel 5 number
    uint8_t channel_3;      // Channel 3 number
};

// Result structure for ambiguity resolution
struct mm_distance_result {
    float distance_mm;      // Final mm-accurate distance
    double lambda_diff;     // Difference wavelength used
    int n_diff;            // Integer ambiguity resolved

    // Smoothed intermediate values
    double distance_twr_smoothed;  // Smoothed TWR distance
    double phase_diff_smoothed;    // Smoothed phase difference
    double d_diff_smoothed;        // Smoothed difference distance
    double d_diff_minus;           // d_diff_smoothed - lambda_diff
    double d_diff_plus;            // d_diff_smoothed + lambda_diff

    // Filtered per-channel phases (unwrap-filter-rewrap)
    double phase_a_filtered;       // Filtered phase channel A [0, 2pi]
    double phase_b_filtered;       // Filtered phase channel B [0, 2pi]
};

float time_to_dist(float tof);

/* ========================================================================== */
/* MM template extraction (public for digest processors)                      */
/* ========================================================================== */

/**
 * @brief Propagation time result from DS-TWR calculation
 */
struct propagation_time {
    float tof;
    double cfo;
};

/**
 * @brief 4-message MM TWR template extracted from a ranging digest
 */
struct mm_twr_template {
    uint16_t initiator_id;
    uint16_t responder_id;
    uint8_t channel;

    uint64_t tx_poll, rx_poll;
    uint64_t tx_resp, rx_resp;
    uint64_t tx_final, rx_final;
    uint64_t tx_post_final, rx_post_final;

    double phase_poll;
    double phase_resp;
    double phase_final;
    double phase_post_final;

    double cfo_poll;
    double cfo_resp;
};

/**
 * @brief Extract a 4-message MM TWR template from a ranging digest
 */
struct mm_twr_template build_mm_template_from_digest(
    const struct deca_ranging_digest *digest,
    uint16_t initiator_id, uint16_t responder_id,
    uint8_t channel);

/**
 * @brief Check if an MM template has all required timestamps and phases
 */
bool mm_template_complete(const struct mm_twr_template *tmpl);

/**
 * @brief Calculate propagation time from an MM TWR template (uses first 6 timestamps)
 */
struct propagation_time mm_calculate_propagation_time(const struct mm_twr_template *tmpl);

int estimate_distances(const struct device *dev, const struct deca_ranging_digest *digest, uint8_t channel, bool estimateOnlyLocal,
		       deca_short_addr_t local_addr, struct measurement *measurements,
		       int max_measurements);

// Single-channel version (existing)
int estimate_distances_mm(const struct device *dev,
                          const struct deca_ranging_digest *digest, uint8_t channel, bool estimateOnlyLocal,
			deca_short_addr_t local_addr, struct measurement_mm *measurements, int max_measurements);

// Dual-channel version (new)
int estimate_distances_mm_dual(const struct device *dev,
                               const struct deca_ranging_digest *digest_ch_a,
                               const struct deca_ranging_digest *digest_ch_b,
                               uint8_t channel_a,
                               uint8_t channel_b,
                               bool estimateOnlyLocal,
                               deca_short_addr_t local_addr,
                               struct measurement_mm *measurements,
                               int max_measurements);
// Optional series identifiers for storing time series per pair
typedef struct {
    uint16_t series_phase_a;   // series id for phase of channel_a
    uint16_t series_phase_b;   // series id for phase of channel_b
    uint16_t series_twr;       // series id for coarse TWR distance
    uint16_t series_delta;     // series id for delta phase-derived distance
} mm_series_ids_t;

struct node_twr_timestamps; /* forward declaration from node_table.h */

/**
 * Extract DS-TWR timestamps for a specific node pair from a digest.
 *
 * @param digest Ranging digest containing raw frame data
 * @param initiator_id Initiator address
 * @param responder_id Responder address
 * @param out Output struct for the 6 core timestamps
 * @return 0 on success (complete DS-TWR), -1 if timestamps incomplete
 */
int twr_extract_timestamps(const struct deca_ranging_digest *digest,
                           uint16_t initiator_id, uint16_t responder_id,
                           struct node_twr_timestamps *out);

/**
 * Set/get the runtime time series capacity for MM ranging.
 * Overrides CONFIG_MM_RANGING_TIME_SERIES_SIZE at runtime.
 */
void mm_set_time_series_size(uint8_t size);
uint8_t mm_get_time_series_size(void);

#endif
