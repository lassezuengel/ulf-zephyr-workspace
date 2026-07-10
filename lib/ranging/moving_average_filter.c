#include <app/lib/ranging/phase_filter.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(moving_average_filter, CONFIG_MOVING_AVERAGE_FILTER_LOG_LEVEL);

/**
 * @brief Moving average filter context
 */
typedef struct {
    double *buffer;          // Circular buffer for samples
    double *unwrapped_buffer; // Buffer for unwrapped phase values
    size_t window_size;      // Size of the moving window
    size_t head;             // Current head position in circular buffer
    size_t count;            // Number of samples currently in buffer
    double sum;              // Current sum for efficient average calculation
    double last_unwrapped;   // Last unwrapped phase value
    bool initialized;        // Whether context is properly initialized
} moving_average_context_t;

static int moving_average_init(void *context, size_t window_size)
{
    if (!context || window_size == 0) {
        LOG_ERR("Invalid parameters: context=%p, window_size=%zu", context, window_size);
        return -EINVAL;
    }

    moving_average_context_t *ctx = (moving_average_context_t *)context;

    // Allocate buffers within the context memory
    // We have CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE bytes total per context, need space for two double arrays
    size_t buffer_size = window_size * sizeof(double);
    size_t available_memory = CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE - sizeof(moving_average_context_t);
    if (buffer_size * 2 > available_memory) {
        size_t max_window = available_memory / (2 * sizeof(double));
        LOG_ERR("Window size too large: %zu (max ~%zu for double buffers with %d byte context)",
               window_size, max_window, CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE);
        return -ENOMEM;
    }

    // Place buffers after the context structure
    uint8_t *memory = (uint8_t *)context;
    ctx->buffer = (double *)(memory + sizeof(moving_average_context_t));
    ctx->unwrapped_buffer = (double *)(memory + sizeof(moving_average_context_t) + buffer_size);

    ctx->window_size = window_size;
    ctx->head = 0;
    ctx->count = 0;
    ctx->sum = 0.0;
    ctx->last_unwrapped = 0.0;
    ctx->initialized = true;

    // Clear buffers
    memset(ctx->buffer, 0, buffer_size);
    memset(ctx->unwrapped_buffer, 0, buffer_size);

    LOG_DBG("Moving average filter initialized: window_size=%zu", window_size);
    return 0;
}

static double moving_average_process(void *context, double new_sample, bool is_phase)
{
    if (!context) {
        LOG_ERR("Context is NULL");
        return new_sample;  // Pass through
    }

    moving_average_context_t *ctx = (moving_average_context_t *)context;

    if (!ctx->initialized) {
        LOG_ERR("Context not initialized");
        return new_sample;
    }

    double sample_to_average = new_sample;

    // Handle phase unwrapping if this is a phase measurement
    if (is_phase) {
        if (ctx->count == 0) {
            // First sample - initialize unwrapped value
            ctx->last_unwrapped = new_sample;
            sample_to_average = new_sample;
        } else {
            // Unwrap the new phase sample
            double diff = new_sample - WRAP_PHASE(ctx->last_unwrapped);

            // Handle 2π discontinuities
            if (diff > MM_PI) {
                diff -= 2 * MM_PI;
            } else if (diff < -MM_PI) {
                diff += 2 * MM_PI;
            }

            ctx->last_unwrapped = ctx->last_unwrapped + diff;
            sample_to_average = ctx->last_unwrapped;
        }

        // Store unwrapped value
        ctx->unwrapped_buffer[ctx->head] = sample_to_average;
    }

    // Remove old sample from sum if buffer is full
    if (ctx->count == ctx->window_size) {
        ctx->sum -= ctx->buffer[ctx->head];
    }

    // Add new sample
    ctx->buffer[ctx->head] = sample_to_average;
    ctx->sum += sample_to_average;

    // Update indices and count
    ctx->head = (ctx->head + 1) % ctx->window_size;
    if (ctx->count < ctx->window_size) {
        ctx->count++;
    }

    // Calculate average
    double average = ctx->sum / ctx->count;

    // For phase measurements, wrap the result back to [0, 2π]
    if (is_phase) {
        average = WRAP_PHASE(average);
    }

    LOG_DBG("Moving average: sample=%.6f, avg=%.6f, count=%zu, is_phase=%d",
           new_sample, average, ctx->count, is_phase);

    return average;
}

static void moving_average_reset(void *context)
{
    if (!context) {
        LOG_ERR("Context is NULL");
        return;
    }

    moving_average_context_t *ctx = (moving_average_context_t *)context;

    if (!ctx->initialized) {
        LOG_WRN("Context not initialized, nothing to reset");
        return;
    }

    ctx->head = 0;
    ctx->count = 0;
    ctx->sum = 0.0;
    ctx->last_unwrapped = 0.0;

    // Clear buffers
    memset(ctx->buffer, 0, ctx->window_size * sizeof(double));
    memset(ctx->unwrapped_buffer, 0, ctx->window_size * sizeof(double));

    LOG_DBG("Moving average filter reset");
}

// Export the moving average filter operations
const phase_filter_ops_t moving_average_ops = {
    .name = "moving_average",
    .init = moving_average_init,
    .process = moving_average_process,
    .reset = moving_average_reset,
};
