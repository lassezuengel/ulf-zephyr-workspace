#ifndef MEASUREMENT_FILTER_H
#define MEASUREMENT_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include <app/lib/ranging/mm_constants.h>

/**
 * @file measurement_filter.h
 * @brief Generic measurement filtering framework for ranging applications
 * 
 * Provides abstraction layer for filtering any type of measurement value
 * (phases, TOF, distances, CFO, etc.) with proper neighbor pair tracking.
 * Supports different filter types through a common interface.
 */

// Forward declarations
struct filter;
struct filter_manager;

/**
 * @brief Filter type enumeration for explicit filter selection
 * 
 * Allows users to explicitly choose which filter implementation to use
 * for each measurement type, keeping the filter manager generic.
 */
typedef enum {
    FILTER_TYPE_MOVING_AVG = 0,      // Simple moving average filter
    FILTER_TYPE_SAVITZKY_GOLAY,      // Savitzky-Golay filter
    FILTER_TYPE_MAX                  // Number of filter types
} filter_type_t;

/**
 * @brief Generic filter operations interface
 */
typedef struct filter_ops {
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
     * @return Filtered output value
     */
    double (*process)(void *context, double new_sample);
    
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
} filter_ops_t;

/**
 * @brief Generic filter instance for neighbor pair measurements
 */
typedef struct filter {
    const filter_ops_t *ops;           // Filter operations
    void *context;                     // Filter-specific context
    
    // Neighbor pair identification (maintains ranging table semantics)
    uint16_t initiator_id;
    uint16_t responder_id;
    uint8_t filter_index;              // Generic index for measurement type
    uint32_t filter_hash;              // Hash of the neighbor pair + filter index
    
    // Filter state
    size_t window_size;
    size_t sample_count;               // Number of samples processed
    bool is_initialized;
    
    // Statistics
    double last_output;
    uint32_t reset_count;
} filter_t;

/**
 * @brief Filter manager for multiple neighbor pairs and measurement types
 */
typedef struct filter_manager {
    filter_t filters[MM_MAX_NODE_PAIRS * 8];  // Up to 8 different filters per node pair
    int active_filters;
    
    // Default configuration
    const filter_ops_t *default_ops;
    size_t default_window_size;
    
    // Memory management
    uint8_t context_pool[(MM_MAX_NODE_PAIRS * 8) * CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE];
    size_t context_size;
    bool context_allocated[MM_MAX_NODE_PAIRS * 8];
} filter_manager_t;

// Global filter manager instance
extern filter_manager_t g_filter_manager;

/**
 * @brief Initialize the global filter manager
 * @param ops Default filter operations to use
 * @param window_size Default window size
 * @param context_size Size of context needed per filter
 * @return 0 on success, negative error code on failure
 */
int filter_manager_init(const filter_ops_t *ops, 
                        size_t window_size, 
                        size_t context_size);

/**
 * @brief Get or create a filter for a neighbor pair and measurement type
 * @param manager Filter manager instance
 * @param initiator_id Ranging initiator node ID
 * @param responder_id Ranging responder node ID
 * @param filter_index Index identifying the measurement type (0,1,2,...)
 * @param filter_type Type of filter to create (moving avg, Savitzky-Golay, etc.)
 * @param window_size Window size for the filter
 * @return Pointer to filter instance, or NULL on error
 */
filter_t *get_or_create_filter(filter_manager_t *manager,
                               uint16_t initiator_id, 
                               uint16_t responder_id,
                               uint8_t filter_index,
                               filter_type_t filter_type,
                               size_t window_size);

/**
 * @brief Reset all filters in the manager
 * @param manager Filter manager instance
 */
void filter_manager_reset(filter_manager_t *manager);

/**
 * @brief Get statistics for all active filters
 * @param manager Filter manager instance
 * @param active_count Output: number of active filters
 * @param total_samples Output: total samples processed
 */
void filter_manager_get_stats(filter_manager_t *manager,
                              int *active_count,
                              uint32_t *total_samples);

/**
 * @brief Hash function for neighbor pair + filter type identification
 * @param initiator_id Initiator node ID
 * @param responder_id Responder node ID
 * @param filter_index Filter type index
 * @return Hash value for the neighbor pair and filter type
 */
static inline uint32_t neighbor_filter_hash(uint16_t initiator_id, 
                                           uint16_t responder_id, 
                                           uint8_t filter_index) {
    // Hash combining both IDs and filter index
    return ((uint32_t)initiator_id << 16) | ((uint32_t)responder_id << 8) | filter_index;
}

// Available generic filter implementations
extern const filter_ops_t moving_average_ops;
extern const filter_ops_t savgol_ops;  // Future implementation

// Common filter window sizes (use existing constants from mm_constants.h)
#ifndef MM_FILTER_WINDOW_SIZE_SMALL
#define MM_FILTER_WINDOW_SIZE_SMALL   5
#endif
#ifndef MM_FILTER_WINDOW_SIZE_LARGE  
#define MM_FILTER_WINDOW_SIZE_LARGE   20
#endif

#endif // MEASUREMENT_FILTER_H