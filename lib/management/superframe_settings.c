/**
 * @file superframe_settings.c
 * @brief Superframe configuration settings implementation
 */

#include <app/lib/management/superframe_settings.h>
#include <app/lib/management/network_settings.h>
#include <app/lib/scheduling/upper/block_type_registry.h>
#include <app/lib/timesync/time_synchronization.h>
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_SWARM_RANGING)
#include <app/lib/blocks/swarm_ranging.h>
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_MULOC)
#include <app/lib/blocks/muloc.h>
#endif
#include <app/lib/scheduling/upper/block_scheduler.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(superframe_settings, CONFIG_LOG_DEFAULT_LEVEL);

/* Current superframe configuration */
static struct superframe_config_ble current_config;
static struct superframe_config_ble default_config;
static bool defaults_set = false;
static bool initialized = false;

/* Registered callbacks per block type */
struct callback_entry {
    block_result_cb_t cb;
    void *user_data;
};
static struct callback_entry callbacks[BLOCK_TYPE_MAX];

/* Callback wrapper structures - need persistent storage for runtime configs */
struct callback_wrapper {
    block_result_cb_t cb;
    void *user_data;
};
static struct callback_wrapper cb_wrappers[SUPERFRAME_MAX_BLOCKS];

/**
 * Union of all possible runtime config types for static allocation.
 * This avoids the need for k_malloc which may not be available.
 */
union runtime_config_storage {
    struct glossy_block_config glossy;
    struct mtm_block_config mtm;
    struct mm_block_config mm;
    struct mm_reference_block_config mm_reference;
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_LS_POSITION)
    struct ls_position_block_config ls_position;
#endif
    struct time_sync_check_config time_sync_check;
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_SWARM_RANGING)
    struct swarm_ranging_block_config swarm_ranging;
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_MULOC)
    struct muloc_block_config muloc;
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
    struct butler_block_config butler;
#endif
};

/* Static storage for runtime configs - one per possible block */
static union runtime_config_storage runtime_configs[SUPERFRAME_MAX_BLOCKS];

/* Mutex for thread safety */
static K_MUTEX_DEFINE(settings_mutex);

/* ========================================================================== */
/* NVS Settings Handler                                                        */
/* ========================================================================== */

/* Flag to track if config was loaded from NVS */
static bool loaded_from_nvs = false;

static int settings_set(const char *name, size_t len,
                        settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "config", &next) && !next) {
        if (len != sizeof(current_config)) {
            LOG_WRN("NVS config size mismatch: got %zu, expected %zu",
                    len, sizeof(current_config));
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &current_config, sizeof(current_config));
        if (rc < 0) {
            return rc;
        }
        loaded_from_nvs = true;
        LOG_INF("Loaded superframe config from NVS: %u blocks",
                current_config.block_count);
        return 0;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(superframe, "superframe", NULL, settings_set, NULL, NULL);

/* ========================================================================== */
/* Public API                                                                  */
/* ========================================================================== */

void superframe_settings_set_defaults(const struct superframe_config_ble *config)
{
    k_mutex_lock(&settings_mutex, K_FOREVER);

    memcpy(&default_config, config, sizeof(default_config));
    defaults_set = true;

    /* Only set as current if NVS didn't already load a config.
     * This preserves user-configured superframes across reboots. */
    if (!loaded_from_nvs && !initialized) {
        memcpy(&current_config, config, sizeof(current_config));
        LOG_INF("Superframe defaults set as current: %u blocks", config->block_count);
    } else if (loaded_from_nvs) {
        LOG_INF("Superframe defaults set (NVS config preserved): %u blocks in NVS",
                current_config.block_count);
    } else {
        LOG_INF("Superframe defaults set: %u blocks", config->block_count);
    }

    k_mutex_unlock(&settings_mutex);
}

int superframe_settings_init(void)
{
    k_mutex_lock(&settings_mutex, K_FOREVER);

    if (!defaults_set) {
        LOG_WRN("No defaults set - using empty config");
        memset(&current_config, 0, sizeof(current_config));
        current_config.block_count = 1;
        current_config.blocks[0].type = BLOCK_TYPE_GLOSSY;
    }

    /* Settings subsystem will call our settings_set handler
     * if there's data in NVS, overwriting current_config */

    initialized = true;

    LOG_INF("Superframe settings initialized: %u blocks", current_config.block_count);

    k_mutex_unlock(&settings_mutex);
    return 0;
}

int superframe_settings_get(struct superframe_config_ble *config)
{
    if (!config) {
        return -EINVAL;
    }

    k_mutex_lock(&settings_mutex, K_FOREVER);
    memcpy(config, &current_config, sizeof(*config));
    k_mutex_unlock(&settings_mutex);

    return 0;
}

int superframe_settings_set(const struct superframe_config_ble *config)
{
    if (!config) {
        return -EINVAL;
    }

    if (config->block_count == 0 || config->block_count > SUPERFRAME_MAX_BLOCKS) {
        LOG_ERR("Invalid block count: %u", config->block_count);
        return -EINVAL;
    }

    k_mutex_lock(&settings_mutex, K_FOREVER);
    memcpy(&current_config, config, sizeof(current_config));
    k_mutex_unlock(&settings_mutex);

    LOG_INF("Superframe config updated: %u blocks", config->block_count);

    return 0;
}

uint8_t superframe_settings_get_block_count(void)
{
    uint8_t count;

    k_mutex_lock(&settings_mutex, K_FOREVER);
    count = current_config.block_count;
    k_mutex_unlock(&settings_mutex);

    return count;
}

int superframe_settings_set_block_count(uint8_t count)
{
    if (count == 0 || count > SUPERFRAME_MAX_BLOCKS) {
        LOG_ERR("Invalid block count: %u", count);
        return -EINVAL;
    }

    k_mutex_lock(&settings_mutex, K_FOREVER);

    /* If increasing, zero out new blocks */
    if (count > current_config.block_count) {
        for (uint8_t i = current_config.block_count; i < count; i++) {
            memset(&current_config.blocks[i], 0, sizeof(current_config.blocks[i]));
            current_config.blocks[i].type = BLOCK_TYPE_NONE;
        }
    }

    current_config.block_count = count;

    k_mutex_unlock(&settings_mutex);

    LOG_INF("Block count set to %u", count);
    return 0;
}

int superframe_settings_get_block(uint8_t index, struct block_config_ble *config)
{
    if (!config) {
        return -EINVAL;
    }

    k_mutex_lock(&settings_mutex, K_FOREVER);

    if (index >= current_config.block_count) {
        k_mutex_unlock(&settings_mutex);
        LOG_ERR("Block index %u out of range (count=%u)", index, current_config.block_count);
        return -EINVAL;
    }

    memcpy(config, &current_config.blocks[index], sizeof(*config));

    k_mutex_unlock(&settings_mutex);
    return 0;
}

int superframe_settings_set_block(uint8_t index, const struct block_config_ble *config)
{
    if (!config) {
        return -EINVAL;
    }

    k_mutex_lock(&settings_mutex, K_FOREVER);

    if (index >= current_config.block_count) {
        k_mutex_unlock(&settings_mutex);
        LOG_ERR("Block index %u out of range (count=%u)", index, current_config.block_count);
        return -EINVAL;
    }

    /* Validate block config */
    int err = block_type_validate_config(config);
    if (err) {
        k_mutex_unlock(&settings_mutex);
        return err;
    }

    memcpy(&current_config.blocks[index], config, sizeof(current_config.blocks[index]));

    k_mutex_unlock(&settings_mutex);

    LOG_INF("Block %u updated: type=%s", index, block_type_get_name(config->type));
    return 0;
}

int superframe_settings_validate(void)
{
    k_mutex_lock(&settings_mutex, K_FOREVER);

    if (current_config.block_count == 0 ||
        current_config.block_count > SUPERFRAME_MAX_BLOCKS) {
        k_mutex_unlock(&settings_mutex);
        LOG_ERR("Invalid block count: %u", current_config.block_count);
        return -EINVAL;
    }

    bool has_glossy = false;

    for (uint8_t i = 0; i < current_config.block_count; i++) {
        int err = block_type_validate_config(&current_config.blocks[i]);
        if (err) {
            k_mutex_unlock(&settings_mutex);
            LOG_ERR("Block %u validation failed: %d", i, err);
            return err;
        }

        if (current_config.blocks[i].type == BLOCK_TYPE_GLOSSY ||
            current_config.blocks[i].type == BLOCK_TYPE_BUTLER) {
            has_glossy = true;
        }
    }

    if (!has_glossy) {
        LOG_WRN("Superframe has no time sync block - enable free-running mode to run without time sync");
    }

    k_mutex_unlock(&settings_mutex);
    return 0;
}

int superframe_settings_register_callback(block_type_t type,
                                          block_result_cb_t cb,
                                          void *user_data)
{
    if (type >= BLOCK_TYPE_MAX) {
        return -EINVAL;
    }

    k_mutex_lock(&settings_mutex, K_FOREVER);
    callbacks[type].cb = cb;
    callbacks[type].user_data = user_data;
    k_mutex_unlock(&settings_mutex);

    LOG_DBG("Registered callback for block type %s", block_type_get_name(type));
    return 0;
}

int superframe_settings_build(struct superframe **sframe)
{
    if (!sframe) {
        return -EINVAL;
    }

    k_mutex_lock(&settings_mutex, K_FOREVER);

    /* Validate first */
    int err = superframe_settings_validate();
    if (err) {
        k_mutex_unlock(&settings_mutex);
        return err;
    }

    /* Allocate superframe */
    err = superframe_alloc(current_config.block_count, sframe);
    if (err) {
        k_mutex_unlock(&settings_mutex);
        LOG_ERR("Failed to allocate superframe: %d", err);
        return err;
    }

    struct superframe *sf = *sframe;

    /* Build each block */
    for (uint8_t i = 0; i < current_config.block_count; i++) {
        const struct block_config_ble *ble_cfg = &current_config.blocks[i];

        if (ble_cfg->type == BLOCK_TYPE_NONE) {
            /* Empty slot - skip */
            sf->blocks[i].block_handler = NULL;
            sf->blocks[i].config = NULL;
            sf->blocks[i].duration_ms = 0;
            sf->blocks[i].type = BLOCK_TYPE_NONE;
            continue;
        }

        const struct block_type_info *info = block_type_get_info(ble_cfg->type);
        if (!info) {
            /* Block type not compiled in - skip slot (idle during this block).
             * This allows a homogenous superframe to define blocks that only
             * some nodes support (e.g. PF position on mobile nodes only). */
            LOG_WRN("Block type %u not available, skipping slot %u", ble_cfg->type, i);
            sf->blocks[i].block_handler = NULL;
            sf->blocks[i].config = NULL;
            sf->blocks[i].duration_ms = 0;
            sf->blocks[i].type = BLOCK_TYPE_NONE;
            continue;
        }

        /* Use static storage for runtime config */
        void *runtime_cfg = &runtime_configs[i];

        /* Setup callback wrapper */
        cb_wrappers[i].cb = callbacks[ble_cfg->type].cb;
        cb_wrappers[i].user_data = callbacks[ble_cfg->type].user_data;

        /* Convert BLE config to runtime config (if block type has one) */
        if (info->build_runtime) {
            err = info->build_runtime(ble_cfg,
                                      cb_wrappers[i].cb,
                                      &cb_wrappers[i],
                                      runtime_cfg);
            if (err) {
                LOG_ERR("Failed to build runtime config for block %u: %d", i, err);
                superframe_free(*sframe);
                *sframe = NULL;
                k_mutex_unlock(&settings_mutex);
                return err;
            }
        }

        sf->blocks[i].block_handler = info->handler;
        sf->blocks[i].config = runtime_cfg;
        sf->blocks[i].type = ble_cfg->type;

        /* Use per-block duration if set, otherwise global default */
        if (ble_cfg->slot_duration_ms > 0) {
            sf->blocks[i].duration_ms = ble_cfg->slot_duration_ms;
        } else {
            sf->blocks[i].duration_ms = network_get_scheduler_slot_duration_ms();
        }

        LOG_DBG("Block %u: type=%s, duration=%u ms",
                i, info->name, sf->blocks[i].duration_ms);
    }

    /* Auto-detect sync mode and scheduling mode from first block type */
    if (sf->block_count > 0) {
        if (sf->blocks[0].type == BLOCK_TYPE_BUTLER) {
            time_sync_set_mode(TIME_SYNC_MODE_OFFSET);
            sf->scheduling_mode = SUPERFRAME_MODE_BEACON_RELATIVE;
            LOG_INF("Auto-detected OFFSET sync + BEACON_RELATIVE scheduling (Butler block 0)");
        } else if (sf->blocks[0].type == BLOCK_TYPE_GLOSSY) {
            time_sync_set_mode(TIME_SYNC_MODE_SKEW);
            sf->scheduling_mode = SUPERFRAME_MODE_MODULO;
            LOG_INF("Auto-detected SKEW sync + MODULO scheduling (Glossy block 0)");
        } else {
            sf->scheduling_mode = SUPERFRAME_MODE_MODULO;
        }
    }

    k_mutex_unlock(&settings_mutex);

    LOG_INF("Built superframe: %u blocks", sf->block_count);
    return 0;
}

int superframe_settings_apply(void)
{
    int err = superframe_settings_validate();
    if (err) {
        return err;
    }

    err = superframe_settings_save();
    if (err) {
        LOG_WRN("Failed to save config to NVS: %d", err);
    }

    /* Rebuild the running superframe (pause -> rebuild -> resume) */
    err = block_scheduler_rebuild_superframe();
    if (err) {
        LOG_ERR("Superframe rebuild failed: %d", err);
        return err;
    }

    LOG_INF("Superframe settings applied and rebuilt");
    return 0;
}

int superframe_settings_save(void)
{
    k_mutex_lock(&settings_mutex, K_FOREVER);

    int err = settings_save_one("superframe/config",
                                &current_config,
                                sizeof(current_config));

    k_mutex_unlock(&settings_mutex);

    if (err) {
        LOG_ERR("Failed to save superframe config: %d", err);
        return err;
    }

    LOG_INF("Superframe config saved to NVS");
    return 0;
}

void superframe_settings_print(void)
{
    k_mutex_lock(&settings_mutex, K_FOREVER);

    LOG_INF("Superframe Configuration:");
    LOG_INF("  Block count: %u", current_config.block_count);

    for (uint8_t i = 0; i < current_config.block_count; i++) {
        const struct block_config_ble *block = &current_config.blocks[i];
        const char *type_name = block_type_get_name(block->type);

        LOG_INF("  Block %u: type=%s, slot_duration=%u ms",
                i, type_name, block->slot_duration_ms);

        switch (block->type) {
        case BLOCK_TYPE_GLOSSY:
            LOG_INF("    max_depth=%u, tx_delay=%u us, guard=%u us, channel=%u",
                    block->config.glossy.max_depth,
                    block->config.glossy.transmission_delay_us,
                    block->config.glossy.guard_period_us,
                    block->config.glossy.channel);
            break;

        case BLOCK_TYPE_MTM:
            LOG_INF("    schedule=%u, slots_per_phase=%u, phases=%u",
                    block->config.mtm.schedule_type,
                    block->config.mtm.slots_per_phase,
                    block->config.mtm.phases);
            break;

        case BLOCK_TYPE_MM:
            LOG_INF("    schedule=%u, slots_per_phase=%u, phases=%u",
                    block->config.mm.schedule_type,
                    block->config.mm.slots_per_phase,
                    block->config.mm.phases);
            LOG_INF("    initiator=0x%04x, responder=0x%04x",
                    block->config.mm.initiator_addr,
                    block->config.mm.responder_addr);
            break;

        case BLOCK_TYPE_MM_REFERENCE:
            LOG_INF("    initiator=0x%04x, responder=0x%04x, ch=%u",
                    block->config.mm_reference.initiator_addr,
                    block->config.mm_reference.responder_addr,
                    block->config.mm_reference.channel);
            break;

        case BLOCK_TYPE_LS_POSITION:
            LOG_INF("    min_anchors=%u, max_age=%u ms",
                    block->config.ls_position.min_anchors,
                    block->config.ls_position.max_age_ms);
            break;

        default:
            break;
        }
    }

    k_mutex_unlock(&settings_mutex);
}
