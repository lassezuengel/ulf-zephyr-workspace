#ifndef PHASE_UTILS_H
#define PHASE_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <app/lib/ranging/mm_constants.h>
#include <app/lib/ranging/measurement_filter.h>

/**
 * @file phase_utils.h
 * @brief Phase-specific utilities for MM ranging
 * 
 * Provides phase unwrapping, wrapping, and filtered phase processing
 * utilities that work with the generic measurement filtering framework.
 */

/**
 * @brief Unwrap phase array to handle 2π discontinuities
 * 
 * Removes 2π jumps from phase measurements to create smooth continuous
 * phase evolution suitable for filtering operations.
 * 
 * @param wrapped Array of wrapped phase values in [0, 2π] or [-π, π]
 * @param unwrapped Output array for unwrapped values
 * @param length Length of both arrays
 */
void phase_unwrap(const double *wrapped, double *unwrapped, size_t length);

/**
 * @brief Wrap phase value to [0, 2π] range
 * @param phase Input phase value (can be any range)
 * @return Wrapped phase in [0, 2π]
 */
static inline double phase_wrap(double phase) {
    return WRAP_PHASE(phase);
}

/**
 * @brief Wrap phase value to [-π, π] range
 * @param phase Input phase value (can be any range)
 * @return Wrapped phase in [-π, π]
 */
static inline double phase_wrap_symmetric(double phase) {
    double wrapped = phase_wrap(phase);  // First wrap to [0, 2π]
    if (wrapped > MM_PI) {
        wrapped -= 2.0 * MM_PI;         // Convert to [-π, π]
    }
    return wrapped;
}

/**
 * @brief Filter a single phase value with proper unwrapping/wrapping
 * 
 * This function handles the complete phase filtering pipeline:
 * 1. Maintains internal unwrapping state for the filter
 * 2. Applies the generic filter to unwrapped values
 * 3. Wraps the result back to [0, 2π] range
 * 
 * @param phase_value New phase measurement to filter
 * @param filter Generic filter instance to use
 * @return Filtered and wrapped phase value in [0, 2π]
 */
double filter_wrapped_phase(double phase_value, filter_t *filter);

/**
 * @brief Calculate phase difference with proper wrapping
 * 
 * Computes phase1 - phase2 with result wrapped to [-π, π] range
 * for optimal difference calculation.
 * 
 * @param phase1 First phase value
 * @param phase2 Second phase value  
 * @return Phase difference in [-π, π]
 */
static inline double phase_difference(double phase1, double phase2) {
    double diff = phase1 - phase2;
    return phase_wrap_symmetric(diff);
}

/**
 * @brief Check if phase values are within valid range
 * @param phase Phase value to validate
 * @return true if phase is finite and reasonable, false otherwise
 */
static inline bool is_valid_phase(double phase) {
    return isfinite(phase) && (phase >= -4*MM_PI) && (phase <= 4*MM_PI);
}

#endif // PHASE_UTILS_H