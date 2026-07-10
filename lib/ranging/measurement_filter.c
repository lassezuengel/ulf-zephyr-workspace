#include <app/lib/ranging/measurement_filter.h>
#include <string.h>
#include <zephyr/logging/log.h>

// External filter implementations
extern const filter_ops_t moving_average_ops;

#ifdef CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY
#include <app/lib/ranging/savitzky_golay_filter.h>
#endif

LOG_MODULE_REGISTER(measurement_filter, CONFIG_PHASE_FILTER_MANAGER_LOG_LEVEL);

// Global filter manager instance
filter_manager_t g_filter_manager = {0};

/**
 * @brief Get filter operations for a specific filter type
 * @param filter_type The type of filter requested
 * @return Pointer to filter operations, or NULL if type not supported
 */
static const filter_ops_t *get_filter_ops_by_type(filter_type_t filter_type)
{
    switch (filter_type) {
        case FILTER_TYPE_MOVING_AVG:
            return &moving_average_ops;

        case FILTER_TYPE_SAVITZKY_GOLAY:
#ifdef CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY
            return &savitzky_golay_filter_ops;
#else
            LOG_WRN("Savitzky-Golay filter requested but not enabled, falling back to moving average");
            return &moving_average_ops;
#endif

        default:
            LOG_ERR("Unknown filter type: %d", filter_type);
            return NULL;
    }
}


int filter_manager_init(const filter_ops_t *ops,
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

    LOG_INF("Generic filter manager initialized: ops=%s, window_size=%zu, context_size=%zu",
            ops->name, window_size, context_size);

    return 0;
}

filter_t *get_or_create_filter(filter_manager_t *manager,
                               uint16_t initiator_id,
                               uint16_t responder_id,
                               uint8_t filter_index,
                               filter_type_t filter_type,
                               size_t window_size)
{
    if (!manager) {
        LOG_ERR("Manager is NULL");
        return NULL;
    }

    uint32_t hash = neighbor_filter_hash(initiator_id, responder_id, filter_index);

    // First, try to find existing filter
    int max_filters = (MM_MAX_NODE_PAIRS * 8);
    for (int i = 0; i < max_filters; i++) {
        if (manager->filters[i].is_initialized &&
            manager->filters[i].filter_hash == hash) {
            LOG_DBG("Found existing filter for pair %u-%u idx=%u at index %d",
                   initiator_id, responder_id, filter_index, i);
            return &manager->filters[i];
        }
    }

    // Find first available slot
    int free_index = -1;
    for (int i = 0; i < max_filters; i++) {
        if (!manager->filters[i].is_initialized) {
            free_index = i;
            break;
        }
    }

    if (free_index == -1) {
        LOG_WRN("No free filter slots available, max=%d", max_filters);
        return NULL;
    }

    // Find free context memory
    int context_index = -1;
    for (int i = 0; i < max_filters; i++) {
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
    filter_t *filter = &manager->filters[free_index];
    memset(filter, 0, sizeof(filter_t));

    // Get filter operations for the requested filter type
    filter->ops = get_filter_ops_by_type(filter_type);
    if (!filter->ops) {
        LOG_ERR("Failed to get filter operations for type %d", filter_type);
        return NULL;
    }

    filter->context = &manager->context_pool[context_index * manager->context_size];
    filter->initiator_id = initiator_id;
    filter->responder_id = responder_id;
    filter->filter_index = filter_index;
    filter->filter_hash = hash;
    filter->window_size = window_size;
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

    LOG_INF("Created new filter for pair %u-%u idx=%u type=%s at index %d (active_filters=%d)",
           initiator_id, responder_id, filter_index, filter->ops->name, free_index, manager->active_filters);

    return filter;
}

void filter_manager_reset(filter_manager_t *manager)
{
    if (!manager) {
        LOG_ERR("Manager is NULL");
        return;
    }

    LOG_INF("Resetting all filters (active_filters=%d)", manager->active_filters);

    int max_filters = (MM_MAX_NODE_PAIRS * 8);
    for (int i = 0; i < max_filters; i++) {
        if (manager->filters[i].is_initialized) {
            if (manager->filters[i].ops && manager->filters[i].ops->reset) {
                manager->filters[i].ops->reset(manager->filters[i].context);
            }
            manager->filters[i].sample_count = 0;
            manager->filters[i].reset_count++;
        }
    }
}

void filter_manager_get_stats(filter_manager_t *manager,
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

    int max_filters = (MM_MAX_NODE_PAIRS * 8);
    for (int i = 0; i < max_filters; i++) {
        if (manager->filters[i].is_initialized) {
            count++;
            samples += manager->filters[i].sample_count;
        }
    }

    if (active_count) *active_count = count;
    if (total_samples) *total_samples = samples;
}
