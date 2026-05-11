#include <app/lib/ranging/savitzky_golay_filter.h>
#include <app/lib/ranging/mm_constants.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

#include "iterative/savgolFilter.h"  // External library header

LOG_MODULE_REGISTER(savitzky_golay_filter, CONFIG_SAVITZKY_GOLAY_FILTER_LOG_LEVEL);

/**
 * @brief Savitzky-Golay filter context (generic numeric)
 * Stores samples in a circular buffer and applies SG on full windows.
 */
typedef struct {
    MqsRawDataPoint_t *buffer;          // Circular buffer for input samples
    MqsRawDataPoint_t *ordered_buffer;  // Temporary buffer for chronological ordering
    MqsRawDataPoint_t *filtered_buffer; // Buffer for filtered output
    size_t window_size;                 // Size of the filter window
    size_t head;                        // Current head position in circular buffer
    size_t count;                       // Number of samples currently in buffer
    uint8_t half_window_size;           // Half window size for SG filter
    uint8_t poly_order;                 // Polynomial order for SG filter
    bool initialized;                   // Whether context is properly initialized
    double last_output;                 // Last filtered value for immediate return
} savitzky_golay_context_t;

// No phase-specific unwrap/wrap logic here; this wrapper is generic numeric.

static int savitzky_golay_init(void *context, size_t window_size)
{
    if (!context || window_size == 0) {
        LOG_ERR("Invalid parameters: context=%p, window_size=%zu", context, window_size);
        return -EINVAL;
    }
    
    // Ensure odd window size
    if (window_size % 2 == 0) {
        window_size += 1;
        LOG_WRN("Savitzky-Golay requires odd window size, adjusted to %zu", window_size);
    }
    
    // Check maximum window size limitation
    if (window_size > MAX_WINDOW) {
        LOG_ERR("Window size %zu exceeds maximum %d", window_size, MAX_WINDOW);
        return -EINVAL;
    }
    
    savitzky_golay_context_t *ctx = (savitzky_golay_context_t *)context;
    
    // Calculate memory requirements for 3 arrays of MqsRawDataPoint_t
    size_t point_size = sizeof(MqsRawDataPoint_t);
    size_t total_buffer_size = window_size * (point_size * 3);
    size_t available_memory = CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE - sizeof(savitzky_golay_context_t);
    
    if (total_buffer_size > available_memory) {
        size_t max_window = available_memory / (3 * point_size);
        LOG_ERR("Window size too large: %zu (max ~%zu for %d byte context)", 
               window_size, max_window, CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE);
        return -ENOMEM;
    }
    
    // Place all buffers after the context structure in memory
    uint8_t *memory = (uint8_t *)context;
    uint8_t *buffer_start = memory + sizeof(savitzky_golay_context_t);
    
    size_t buffer_size = window_size * point_size;
    
    ctx->buffer = (MqsRawDataPoint_t *)buffer_start;
    ctx->ordered_buffer = (MqsRawDataPoint_t *)(buffer_start + buffer_size);
    ctx->filtered_buffer = (MqsRawDataPoint_t *)(buffer_start + 2 * buffer_size);
    
    ctx->window_size = window_size;
    ctx->half_window_size = (window_size - 1) / 2;
    ctx->poly_order = CONFIG_SAVITZKY_GOLAY_POLY_ORDER;
    ctx->head = 0;
    ctx->count = 0;
    ctx->initialized = true;
    ctx->last_output = 0.0;
    
    // Clear all buffers
    memset(ctx->buffer, 0, buffer_size);
    memset(ctx->ordered_buffer, 0, buffer_size);
    memset(ctx->filtered_buffer, 0, buffer_size);
    
    // Validate polynomial order against effective window size: poly < window_size
    if (ctx->poly_order >= (uint8_t)ctx->window_size) {
        LOG_ERR("Savitzky-Golay poly_order (%u) must be < window_size (%zu)",
                ctx->poly_order, ctx->window_size);
        return -EINVAL;
    }

    LOG_DBG("Savitzky-Golay filter initialized: window=%zu, half_window=%d, poly_order=%u", 
           window_size, ctx->half_window_size, ctx->poly_order);
    
    return 0;
}

static double savitzky_golay_process(void *context, double new_sample)
{
    if (!context) {
        LOG_ERR("Context is NULL");
        return new_sample;  // Pass-through on error
    }
    
    savitzky_golay_context_t *ctx = (savitzky_golay_context_t *)context;
    
    if (!ctx->initialized) {
        LOG_ERR("Filter not initialized");
        return new_sample;  // Pass-through on error
    }
    
    // Add new sample to circular buffer
    ctx->buffer[ctx->head].phaseAngle = new_sample;
    ctx->head = (ctx->head + 1) % ctx->window_size;
    
    if (ctx->count < ctx->window_size) {
        ctx->count++;
        // Not enough samples yet - return unfiltered for now
        ctx->last_output = new_sample;
        return new_sample;
    }
    
    // We have a full window - apply Savitzky-Golay filter
    // Arrange buffer in chronological order
    for (size_t i = 0; i < ctx->window_size; i++) {
        size_t idx = (ctx->head + i) % ctx->window_size;
        ctx->ordered_buffer[i] = ctx->buffer[idx];
    }
    
    // Apply Savitzky-Golay filter on numeric samples
    // Use centered alignment (target_point = 0) to match SciPy/reference behavior
    int ret = mes_savgolFilter(ctx->ordered_buffer, ctx->window_size, ctx->half_window_size,
                               ctx->filtered_buffer, ctx->poly_order, 0, 0);
    
    if (ret != 0) {
        LOG_WRN("Savitzky-Golay filter failed: %d, using pass-through", ret);
        return ctx->last_output;
    }

    // Return the filtered numeric value at the center of the window
    double result = ctx->filtered_buffer[ctx->half_window_size].phaseAngle;
    ctx->last_output = result;
    
    return result;
}

static void savitzky_golay_reset(void *context)
{
    if (!context) return;
    
    savitzky_golay_context_t *ctx = (savitzky_golay_context_t *)context;
    
    if (!ctx->initialized) return;
    
    ctx->head = 0;
    ctx->count = 0;
    ctx->last_output = 0.0;
    
    // Clear all buffers
    size_t buffer_size = ctx->window_size * sizeof(MqsRawDataPoint_t);
    memset(ctx->buffer, 0, buffer_size);
    memset(ctx->ordered_buffer, 0, buffer_size);
    memset(ctx->filtered_buffer, 0, buffer_size);
    
    LOG_DBG("Savitzky-Golay filter reset");
}

const filter_ops_t savitzky_golay_filter_ops = {
    .name = "Savitzky-Golay",
    .init = savitzky_golay_init,
    .process = savitzky_golay_process,
    .reset = savitzky_golay_reset,
};
