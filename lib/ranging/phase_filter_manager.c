#include <app/lib/ranging/phase_filter.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(phase_filter_manager, CONFIG_PHASE_FILTER_MANAGER_LOG_LEVEL);

// Global filter manager instance
phase_filter_manager_t g_filter_manager = {0};

int phase_filter_manager_init(const phase_filter_ops_t *ops,
                              size_t window_size,
                              size_t context_size)
{
    if (!ops || window_size == 0 || context_size == 0 ||
        context_size > CONFIG_MM_RANGING_FILTER_CONTEXT_SIZE) {
        LOG_ERR("Invalid parameters: ops=%p, window_size=%zu, context_size=%zu",
                ops, window_size, context_size);
        return -EINVAL;
    }

    // Reset manager state
    memset(&g_filter_manager, 0, sizeof(g_filter_manager));

    // Set configuration
    g_filter_manager.default_ops = ops;
    g_filter_manager.default_window_size = window_size;
    g_filter_manager.context_size = context_size;
    g_filter_manager.active_filters = 0;

    LOG_INF("Phase filter manager initialized: ops=%s, window_size=%zu, context_size=%zu",
            ops->name, window_size, context_size);

    return 0;
}

phase_filter_t *get_or_create_filter(phase_filter_manager_t *manager,
                                      uint16_t initiator_id,
                                      uint16_t responder_id)
{
    if (!manager) {
        LOG_ERR("Manager is NULL");
        return NULL;
    }

    uint32_t hash = node_pair_hash(initiator_id, responder_id);

    // First, try to find existing filter
    for (int i = 0; i < MM_MAX_NODE_PAIRS; i++) {
        if (manager->filters[i].is_initialized &&
            manager->filters[i].node_pair_hash == hash) {
            LOG_DBG("Found existing filter for pair %u-%u at index %d",
                   initiator_id, responder_id, i);
            return &manager->filters[i];
        }
    }

    // Find first available slot
    int free_index = -1;
    for (int i = 0; i < MM_MAX_NODE_PAIRS; i++) {
        if (!manager->filters[i].is_initialized) {
            free_index = i;
            break;
        }
    }

    if (free_index == -1) {
        LOG_WRN("No free filter slots available, max=%d", MM_MAX_NODE_PAIRS);
        return NULL;
    }

    // Find free context memory
    int context_index = -1;
    for (int i = 0; i < MM_MAX_NODE_PAIRS; i++) {
        if (!manager->context_allocated[i]) {
            context_index = i;
            break;
        }
    }

    if (context_index == -1) {
        LOG_ERR("No free context memory available");
        return NULL;
    }

    // Initialize new filter
    phase_filter_t *filter = &manager->filters[free_index];
    memset(filter, 0, sizeof(phase_filter_t));

    filter->ops = manager->default_ops;
    filter->context = &manager->context_pool[context_index * manager->context_size];
    filter->initiator_id = initiator_id;
    filter->responder_id = responder_id;
    filter->node_pair_hash = hash;
    filter->window_size = manager->default_window_size;
    filter->sample_count = 0;
    filter->reset_count = 0;

    // Initialize the filter's context
    int ret = filter->ops->init(filter->context, filter->window_size);
    if (ret < 0) {
        LOG_ERR("Failed to initialize filter context: %d", ret);
        return NULL;
    }

    filter->is_initialized = true;
    manager->context_allocated[context_index] = true;
    manager->active_filters++;

    LOG_INF("Created new filter for pair %u-%u at index %d (active_filters=%d)",
           initiator_id, responder_id, free_index, manager->active_filters);

    return filter;
}

void phase_filter_manager_reset(phase_filter_manager_t *manager)
{
    if (!manager) {
        LOG_ERR("Manager is NULL");
        return;
    }

    LOG_INF("Resetting all filters (active_filters=%d)", manager->active_filters);

    for (int i = 0; i < MM_MAX_NODE_PAIRS; i++) {
        if (manager->filters[i].is_initialized) {
            if (manager->filters[i].ops && manager->filters[i].ops->reset) {
                manager->filters[i].ops->reset(manager->filters[i].context);
            }
            manager->filters[i].sample_count = 0;
            manager->filters[i].reset_count++;
        }
    }
}

void phase_filter_manager_get_stats(phase_filter_manager_t *manager,
                                     int *active_count,
                                     uint32_t *total_samples)
{
    if (!manager) {
        if (active_count) *active_count = 0;
        if (total_samples) *total_samples = 0;
        return;
    }

    int count = 0;
    uint32_t samples = 0;

    for (int i = 0; i < MM_MAX_NODE_PAIRS; i++) {
        if (manager->filters[i].is_initialized) {
            count++;
            samples += manager->filters[i].sample_count;
        }
    }

    if (active_count) *active_count = count;
    if (total_samples) *total_samples = samples;
}

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
