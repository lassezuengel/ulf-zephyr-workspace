#include <app/lib/ranging/measurement_filter.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(generic_moving_average, CONFIG_MOVING_AVERAGE_FILTER_LOG_LEVEL);

/**
 * @brief Generic moving average filter context (no phase-specific logic)
 */
typedef struct {
    double *buffer;          // Circular buffer for samples
    size_t window_size;      // Size of the moving window
    size_t head;             // Current head position in circular buffer
    size_t count;            // Number of samples currently in buffer
    double sum;              // Current sum for efficient average calculation
    bool initialized;        // Whether context is properly initialized
} generic_moving_average_context_t;

static int generic_moving_average_init(void *context, size_t window_size)
{
    if (!context || window_size == 0) {
        LOG_ERR("Invalid parameters: context=%p, window_size=%zu", context, window_size);
        return -EINVAL;
    }

    generic_moving_average_context_t *ctx = (generic_moving_average_context_t *)context;

    // Allocate buffer within the context memory
    size_t buffer_size = window_size * sizeof(double);
    size_t available_memory = CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE - sizeof(generic_moving_average_context_t);
    if (buffer_size > available_memory) {
        size_t max_window = available_memory / sizeof(double);
        LOG_ERR("Window size too large: %zu (max ~%zu for %d byte context)",
               window_size, max_window, CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE);
        return -ENOMEM;
    }

    // Place buffer after the context structure
    uint8_t *memory = (uint8_t *)context;
    ctx->buffer = (double *)(memory + sizeof(generic_moving_average_context_t));

    ctx->window_size = window_size;
    ctx->head = 0;
    ctx->count = 0;
    ctx->sum = 0.0;
    ctx->initialized = true;

    // Clear buffer
    memset(ctx->buffer, 0, buffer_size);

    LOG_DBG("Generic moving average filter initialized: window_size=%zu", window_size);
    return 0;
}

static double generic_moving_average_process(void *context, double new_sample)
{
    if (!context) {
        LOG_ERR("Context is NULL");
        return new_sample;  // Pass through
    }

    generic_moving_average_context_t *ctx = (generic_moving_average_context_t *)context;

    if (!ctx->initialized) {
        LOG_ERR("Context not initialized");
        return new_sample;
    }

    // Remove old sample from sum if buffer is full
    if (ctx->count == ctx->window_size) {
        ctx->sum -= ctx->buffer[ctx->head];
    }

    // Add new sample
    ctx->buffer[ctx->head] = new_sample;
    ctx->sum += new_sample;

    // Update indices and count
    ctx->head = (ctx->head + 1) % ctx->window_size;
    if (ctx->count < ctx->window_size) {
        ctx->count++;
    }

    // Calculate average
    double average = ctx->sum / ctx->count;

    LOG_DBG("Generic moving average: sample=%.6f, avg=%.6f, count=%zu",
           new_sample, average, ctx->count);

    return average;
}

static void generic_moving_average_reset(void *context)
{
    if (!context) {
        LOG_ERR("Context is NULL");
        return;
    }

    generic_moving_average_context_t *ctx = (generic_moving_average_context_t *)context;

    if (!ctx->initialized) {
        LOG_WRN("Context not initialized, nothing to reset");
        return;
    }

    ctx->head = 0;
    ctx->count = 0;
    ctx->sum = 0.0;

    // Clear buffer
    memset(ctx->buffer, 0, ctx->window_size * sizeof(double));

    LOG_DBG("Generic moving average filter reset");
}

// Export the generic moving average filter operations
const filter_ops_t moving_average_ops = {
    .name = "generic_moving_average",
    .init = generic_moving_average_init,
    .process = generic_moving_average_process,
    .reset = generic_moving_average_reset,
};
