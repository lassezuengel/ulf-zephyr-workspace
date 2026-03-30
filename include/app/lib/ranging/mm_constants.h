#ifndef MM_CONSTANTS_H
#define MM_CONSTANTS_H

#include <math.h>
#include <stdint.h>

/**
 * @file mm_constants.h
 * @brief Constants for millimeter-accurate UWB ranging
 *
 * Based on nils-nicklaus Python implementation for dual-channel
 * phase recovery and ambiguity resolution.
 */

// UWB Channel center frequencies (Hz)
#define FC_CHANNEL_1    3494.4e6    // Channel 1
#define FC_CHANNEL_2    3993.6e6    // Channel 2
#define FC_CHANNEL_3    4492.8e6    // Channel 3
#define FC_CHANNEL_4    3993.6e6    // Channel 4 (same as 2)
#define FC_CHANNEL_5    6489.6e6    // Channel 5
#define FC_CHANNEL_7    6489.6e6    // Channel 7 (same as 5)

// Physical constants
#define SPEED_OF_LIGHT_M_S  299702547.236      // Speed of light in m/s
#define MM_PI               3.14159265358979323846

// Filter configuration defaults - now configurable via Kconfig
#define MM_FILTER_WINDOW_SIZE_DEFAULT   CONFIG_MM_RANGING_FILTER_WINDOW_SIZE
#define MM_MAX_NODE_PAIRS              10
#define MM_MAX_MEASUREMENTS            50

// Wavelength calculations for common channels
#define LAMBDA_CH5      (SPEED_OF_LIGHT_M_S / (2.0 * FC_CHANNEL_5))
#define LAMBDA_CH3      (SPEED_OF_LIGHT_M_S / (2.0 * FC_CHANNEL_3))
#define LAMBDA_DIFF     (SPEED_OF_LIGHT_M_S / (2.0 * fabs(FC_CHANNEL_5 - FC_CHANNEL_3)))

// Clock frequencies from nils-nicklaus constants
#define INTERNAL_CLOCK_FREQ     32768           // Internal RTC frequency
#define TS_CLOCK_FREQ           (128 * 499.2e6) // Timestamp clock frequency

// Confidence thresholds for ambiguity resolution
#define MM_MIN_CONFIDENCE       0.7
#define MM_MAX_PHASE_DIFF       (MM_PI / 2.0)

// Convenience macros
#define WRAP_PHASE(x)   (fmod((x) + 2*MM_PI, 2*MM_PI))
#define UNWRAP_DIFF(x)  (((x) > MM_PI) ? (x) - 2*MM_PI : (((x) < -MM_PI) ? (x) + 2*MM_PI : (x)))

/**
 * @brief Get center frequency for a given UWB channel
 * @param channel Channel number (1, 2, 3, 4, 5, 7)
 * @return Center frequency in Hz, or 0 if invalid channel
 */
static inline double get_channel_frequency(uint8_t channel) {
    switch (channel) {
        case 1: return FC_CHANNEL_1;
        case 2: return FC_CHANNEL_2;
        case 3: return FC_CHANNEL_3;
        case 4: return FC_CHANNEL_4;
        case 5: return FC_CHANNEL_5;
        case 7: return FC_CHANNEL_5;  // Same as channel 5
        default: return 0.0;
    }
}

/**
 * @brief Get wavelength for a given UWB channel
 * @param channel Channel number
 * @return Wavelength in meters
 */
static inline double get_channel_wavelength(uint8_t channel) {
    double fc = get_channel_frequency(channel);
    return (fc > 0) ? (SPEED_OF_LIGHT_M_S / fc) : 0.0;
}

/**
 * @brief Calculate difference wavelength between two channels
 * @param ch1 First channel
 * @param ch2 Second channel
 * @return Difference wavelength in meters
 */
static inline double get_diff_wavelength(uint8_t ch1, uint8_t ch2) {
    double f1 = get_channel_frequency(ch1);
    double f2 = get_channel_frequency(ch2);
    if (f1 <= 0 || f2 <= 0) return 0.0;
    return SPEED_OF_LIGHT_M_S / (fabs(f1 - f2));
}

#endif // MM_CONSTANTS_H
