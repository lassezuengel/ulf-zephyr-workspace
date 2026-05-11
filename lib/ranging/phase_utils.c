#include <app/lib/ranging/phase_utils.h>
#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(phase_utils, CONFIG_LOG_DEFAULT_LEVEL);

/**
 * @brief Context for tracking phase unwrapping state per filter
 */
typedef struct {
    double last_unwrapped_phase;
    bool initialized;
} phase_unwrap_state_t;

// Static storage for phase unwrapping states (keyed by filter pointer)
static phase_unwrap_state_t phase_states[MM_MAX_NODE_PAIRS * 8];
static bool phase_states_initialized = false;

void phase_unwrap(const double *wrapped, double *unwrapped, size_t length)
{
    if (!wrapped || !unwrapped || length == 0) {
        LOG_ERR("Invalid parameters for phase_unwrap");
        return;
    }

    unwrapped[0] = wrapped[0];

    for (size_t i = 1; i < length; i++) {
        double diff = wrapped[i] - wrapped[i-1];

        // Unwrap phase differences > π
        if (diff > MM_PI) {
            diff -= 2 * MM_PI;
        } else if (diff < -MM_PI) {
            diff += 2 * MM_PI;
        }

        unwrapped[i] = unwrapped[i-1] + diff;
    }
}

static phase_unwrap_state_t* get_phase_state(filter_t *filter)
{
    if (!filter) {
        return NULL;
    }

    if (!phase_states_initialized) {
        memset(phase_states, 0, sizeof(phase_states));
        phase_states_initialized = true;
    }

    // Use filter hash as index (simple approach, could have collisions)
    // For production, would need a proper hash table
    uint32_t index = filter->filter_hash % (MM_MAX_NODE_PAIRS * 8);
    return &phase_states[index];
}

double filter_wrapped_phase(double phase_value, filter_t *filter)
{
    if (!filter || !is_valid_phase(phase_value)) {
        LOG_ERR("Invalid parameters: filter=%p, phase=%.6f", filter, phase_value);
        return phase_value;  // Pass through
    }

    phase_unwrap_state_t *state = get_phase_state(filter);
    if (!state) {
        LOG_ERR("Could not get phase unwrapping state");
        return phase_value;
    }

    double unwrapped_phase;

    if (!state->initialized) {
        // First sample - initialize unwrapping state
        state->last_unwrapped_phase = phase_value;
        state->initialized = true;
        unwrapped_phase = phase_value;

        LOG_DBG("Initialized phase unwrapping for filter %u-%u idx=%u: phase=%.6f",
                filter->initiator_id, filter->responder_id, filter->filter_index, phase_value);
    } else {
        // Unwrap the new phase sample relative to the last unwrapped value
        double wrapped_last = phase_wrap(state->last_unwrapped_phase);
        double diff = phase_value - wrapped_last;

        // Handle 2π discontinuities
        if (diff > MM_PI) {
            diff -= 2 * MM_PI;
        } else if (diff < -MM_PI) {
            diff += 2 * MM_PI;
        }

        unwrapped_phase = state->last_unwrapped_phase + diff;
        state->last_unwrapped_phase = unwrapped_phase;
    }

    // Apply generic filtering to unwrapped phase
    double filtered_unwrapped = filter->ops->process(filter->context, unwrapped_phase);

    // Wrap result back to [0, 2π] range
    double filtered_wrapped = phase_wrap(filtered_unwrapped);

    // Update the unwrapped state to the filtered value for continuity
    state->last_unwrapped_phase = filtered_unwrapped;

    LOG_DBG("Phase filtering: raw=%.6f, unwrapped=%.6f, filtered_unwrapped=%.6f, wrapped=%.6f",
            phase_value, unwrapped_phase, filtered_unwrapped, filtered_wrapped);

    return filtered_wrapped;
}
