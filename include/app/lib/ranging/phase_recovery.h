#ifndef PHASE_RECOVERY_H
#define PHASE_RECOVERY_H

#include <stdint.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/lib/ranging/mm_constants.h>

/**
 * @file phase_recovery.h
 * @brief Phase recovery functions for mm-accurate UWB ranging
 * 
 * Implements coarse and fine phase recovery from UWB measurements,
 * CFO correction, and dual-channel phase processing based on the
 * nils-nicklaus Python implementation.
 */

/**
 * @brief Pure computation functions for MM phase recovery
 * 
 * These functions implement the correct bidirectional phase combination
 * as described in the Python reference implementation.
 */

/**
 * @brief Modulo operation that matches Python's % operator behavior
 * 
 * Unlike C's fmod(), this always returns positive values for positive modulus.
 * This is critical for phase wrapping to match Python reference implementation.
 * 
 * @param a Dividend
 * @param b Divisor (modulus)
 * @return Result in range [0, b)
 */
double _mod(double a, double b);

/**
 * @brief Wrap angle to [-π, π] range
 * 
 * Wraps an angle to the range [-π, π], matching the MATLAB wrapToPi function.
 * Used for phase differences in alternative ambiguity resolution.
 * 
 * @param angle Input angle in radians
 * @return Wrapped angle in range [-π, π]
 */
double wrap_to_pi(double angle);

/**
 * @brief Compute coarse phase from poll and response phases
 * 
 * Implements coarse phase recovery by adding the phase measurements
 * from both directions: poll_phase (at responder) + resp_phase (at initiator).
 * This is the correct implementation matching the Python reference.
 * 
 * @param phase_poll Phase measured at responder when receiving poll message
 * @param phase_resp Phase measured at initiator when receiving response message  
 * @return Combined coarse phase in radians [0, 2π]
 */
double compute_coarse_phase_mm(double phase_poll, double phase_resp);

/**
 * @brief Fine phase computation result with all components
 */
struct fine_phase_result {
    double coarse_phase;        // Input coarse phase
    double residual_phase_raw;  // Raw residual phase (final - post_final)
    double residual_phase_smoothed; // Filtered residual phase (if filtering enabled)
    double fine_phase;          // Final computed fine phase
};

/**
 * @brief Compute fine phase using final and post-final measurements
 * 
 * Refines the coarse phase using the residual phase from final and post-final
 * messages as described in the Python reference implementation.
 * 
 * @param coarse_phase Previously computed coarse phase
 * @param phase_final Phase measured at responder when receiving final message
 * @param phase_post_final Phase measured at responder when receiving post-final message
 * @param initiator_id Initiator node ID for time series (optional, use 0 if no filtering)
 * @param responder_id Responder node ID for time series (optional, use 0 if no filtering)
 * @param channel UWB channel number for time series identification
 * @return Fine-tuned phase in radians [0, 2π]
 */
double compute_fine_phase_mm(double coarse_phase, double phase_final, double phase_post_final,
                            uint16_t initiator_id, uint16_t responder_id, uint8_t channel);

/**
 * @brief Compute fine phase with detailed result components
 * 
 * Extended version that returns all phase components: coarse, residual (raw and smoothed),
 * and final fine phase. This provides more detailed information for analysis and debugging.
 * 
 * @param coarse_phase Previously computed coarse phase
 * @param phase_final Phase measured at responder when receiving final message
 * @param phase_post_final Phase measured at responder when receiving post-final message
 * @param initiator_id Initiator node ID for time series (optional, use 0 if no filtering)
 * @param responder_id Responder node ID for time series (optional, use 0 if no filtering)
 * @param channel UWB channel number for time series identification
 * @param result Pointer to structure that will be filled with detailed results
 * @return 0 on success, negative error code on failure
 */
int compute_fine_phase_mm_detailed(double coarse_phase, double phase_final, double phase_post_final,
                                  uint16_t initiator_id, uint16_t responder_id, uint8_t channel,
                                  struct fine_phase_result *result);

/**
 * @brief Dual-channel phase measurement result
 */
struct dual_channel_phases {
    uint16_t initiator_id;
    uint16_t responder_id;
    
    // Channel 5 phases
    double coarse_phase_ch5;
    double fine_phase_ch5;
    double residual_phase_raw_ch5;  // Raw residual phase for channel 5
    
    // Channel 3 phases  
    double coarse_phase_ch3;
    double fine_phase_ch3;
    double residual_phase_raw_ch3;  // Raw residual phase for channel 3
    
    // CFO (Carrier Frequency Offset)
    double cfo_ch5;            // CFO from channel 5 (ppm)
    double cfo_ch3;            // CFO from channel 3 (ppm)
};


/**
 * @brief Correct phase measurement for CFO (Carrier Frequency Offset)
 * 
 * Removes phase errors introduced by carrier frequency offset between
 * transmitter and receiver clocks.
 * 
 * @param raw_phase Raw phase measurement in radians
 * @param cfo_ppm Carrier frequency offset in parts per million
 * @param time_of_flight Time of flight for the measurement (seconds)
 * @return CFO-corrected phase in radians [0, 2π]
 */
double correct_cfo_phase(double raw_phase, double cfo_ppm, double time_of_flight);


#endif // PHASE_RECOVERY_H