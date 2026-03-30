#ifndef PHASE_FILTER_H
#define PHASE_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include <app/lib/ranging/mm_constants.h>

/**
 * @file phase_filter.h
 * @brief Phase-aware windowed filtering for mm-accurate ranging
 * 
 * Provides abstraction layer for filtering phase measurements with
 * proper handling of 2π wrapping. Supports different filter types
 * (moving average, Savitzky-Golay) through a common interface.
 */

// Forward declarations
struct phase_filter;
struct phase_filter_manager;

/**
 * @brief Filter operations interface
 */
typedef struct phase_filter_ops {
    const char *name;
    
    /**
     * @brief Initialize filter context
     * @param context Filter-specific context
     * @param window_size Size of the sliding window
     * @return 0 on success, negative error code on failure
     */
    int (*init)(void *context, size_t window_size);
    
    /**
     * @brief Process a new sample through the filter
     * @param context Filter-specific context
     * @param new_sample New measurement value
     * @param is_phase True if sample is a phase value (needs unwrapping)
     * @return Filtered output value
     */
    double (*process)(void *context, double new_sample, bool is_phase);
    
    /**
     * @brief Reset filter state
     * @param context Filter-specific context
     */
    void (*reset)(void *context);
    
    /**
     * @brief Get current filter confidence/validity
     * @param context Filter-specific context
     * @return Confidence level (0-1), or negative if not applicable
     */
    double (*get_confidence)(void *context);
} phase_filter_ops_t;

/**
 * @brief Phase filter instance
 */
typedef struct phase_filter {
    const phase_filter_ops_t *ops;     // Filter operations
    void *context;                     // Filter-specific context
    
    // Node pair identification
    uint16_t initiator_id;
    uint16_t responder_id;
    uint32_t node_pair_hash;           // Hash of the node pair
    
    // Filter state
    size_t window_size;
    size_t sample_count;               // Number of samples processed
    bool is_initialized;
    
    // Statistics
    double last_output;
    uint32_t reset_count;
} phase_filter_t;

/**
 * @brief Filter manager for multiple node pairs
 */
typedef struct phase_filter_manager {
    phase_filter_t filters[MM_MAX_NODE_PAIRS];
    int active_filters;
    
    // Default configuration
    const phase_filter_ops_t *default_ops;
    size_t default_window_size;
    
    // Memory management
    uint8_t context_pool[MM_MAX_NODE_PAIRS * CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE]; // Pool for filter contexts
    size_t context_size;
    bool context_allocated[MM_MAX_NODE_PAIRS];
} phase_filter_manager_t;

// Global filter manager instance
extern phase_filter_manager_t g_filter_manager;

/**
 * @brief Initialize the global filter manager
 * @param ops Default filter operations to use
 * @param window_size Default window size
 * @param context_size Size of context needed per filter
 * @return 0 on success, negative error code on failure
 */
int phase_filter_manager_init(const phase_filter_ops_t *ops, 
                              size_t window_size, 
                              size_t context_size);

/**
 * @brief Get or create a filter for a node pair
 * @param manager Filter manager instance
 * @param initiator_id Ranging initiator node ID
 * @param responder_id Ranging responder node ID
 * @return Pointer to filter instance, or NULL on error
 */
phase_filter_t *get_or_create_filter(phase_filter_manager_t *manager,
                                      uint16_t initiator_id, 
                                      uint16_t responder_id);

/**
 * @brief Reset all filters in the manager
 * @param manager Filter manager instance
 */
void phase_filter_manager_reset(phase_filter_manager_t *manager);

/**
 * @brief Get statistics for all active filters
 * @param manager Filter manager instance
 * @param active_count Output: number of active filters
 * @param total_samples Output: total samples processed
 */
void phase_filter_manager_get_stats(phase_filter_manager_t *manager,
                                     int *active_count,
                                     uint32_t *total_samples);

/**
 * @brief Hash function for node pair identification
 * @param initiator_id Initiator node ID
 * @param responder_id Responder node ID
 * @return Hash value for the node pair
 */
static inline uint32_t node_pair_hash(uint16_t initiator_id, uint16_t responder_id) {
    // Simple hash combining both IDs
    return ((uint32_t)initiator_id << 16) | responder_id;
}

/**
 * @brief Unwrap phase array to handle 2π discontinuities
 * @param wrapped Array of wrapped phase values
 * @param unwrapped Output array for unwrapped values
 * @param length Length of both arrays
 */
void phase_unwrap(const double *wrapped, double *unwrapped, size_t length);

/**
 * @brief Wrap phase value to [0, 2π] range
 * @param phase Input phase value
 * @return Wrapped phase in [0, 2π]
 */
static inline double phase_wrap(double phase) {
    return WRAP_PHASE(phase);
}

// Available filter implementations
extern const phase_filter_ops_t moving_average_ops;
extern const phase_filter_ops_t savgol_ops;  // Future implementation

#endif // PHASE_FILTER_H