#ifndef RANGING_ENGINE_H
#define RANGING_ENGINE_H

#include <stdint.h>
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
    float confidence;       // Confidence in mm distance (0-1)

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
    float confidence;       // Confidence level (0-1)
    double lambda_diff;     // Difference wavelength used
    int n_diff;            // Integer ambiguity resolved
    
    // Smoothed intermediate values
    double distance_twr_smoothed;  // Smoothed TWR distance
    double phase_diff_smoothed;    // Smoothed phase difference
    double d_diff_smoothed;        // Smoothed difference distance
};

float time_to_dist(float tof);

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
#endif
