/**
 * @file block_type_registry.c
 * @brief Block type registry implementation
 */

#include <app/lib/scheduling/upper/block_type_registry.h>
#include <app/lib/blocks/blocks.h>
#include <app/lib/blocks/mtm.h>
#include <app/lib/blocks/glossy.h>
#include <app/lib/blocks/announcement.h>
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_SWARM_RANGING)
#include <app/lib/blocks/swarm_ranging.h>
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_MULOC)
#include <app/lib/blocks/muloc.h>
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
#include <app/lib/blocks/cir_read.h>
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
#include <app/lib/blocks/butler.h>
#endif
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(block_type_registry, CONFIG_LOG_DEFAULT_LEVEL);

/* Forward declarations of converter functions */
static int glossy_build_runtime(const struct block_config_ble *ble,
                                block_result_cb_t cb, void *cb_data,
                                void *out);
static int glossy_build_ble(const void *runtime, struct block_config_ble *out);

static int mtm_build_runtime(const struct block_config_ble *ble,
                             block_result_cb_t cb, void *cb_data,
                             void *out);
static int mtm_build_ble(const void *runtime, struct block_config_ble *out);

static int mm_reference_build_runtime(const struct block_config_ble *ble,
                                      block_result_cb_t cb, void *cb_data,
                                      void *out);
static int mm_reference_build_ble(const void *runtime, struct block_config_ble *out);

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_LS_POSITION)
static int ls_position_build_runtime(const struct block_config_ble *ble,
                                     block_result_cb_t cb, void *cb_data,
                                     void *out);
static int ls_position_build_ble(const void *runtime, struct block_config_ble *out);
#endif

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_PF_POSITION)
static int pf_position_build_runtime(const struct block_config_ble *ble,
                                     block_result_cb_t cb, void *cb_data,
                                     void *out);
static int pf_position_build_ble(const void *runtime, struct block_config_ble *out);
#endif

static int time_sync_check_build_runtime(const struct block_config_ble *ble,
                                         block_result_cb_t cb, void *cb_data,
                                         void *out);
static int time_sync_check_build_ble(const void *runtime, struct block_config_ble *out);

static int announcement_build_runtime(const struct block_config_ble *ble,
                                      block_result_cb_t cb, void *cb_data,
                                      void *out);
static int announcement_build_ble(const void *runtime, struct block_config_ble *out);

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_SWARM_RANGING)
static int swarm_ranging_build_runtime(const struct block_config_ble *ble,
                                       block_result_cb_t cb, void *cb_data,
                                       void *out);
static int swarm_ranging_build_ble(const void *runtime, struct block_config_ble *out);
#endif

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_MULOC)
static int muloc_build_runtime(const struct block_config_ble *ble,
                               block_result_cb_t cb, void *cb_data,
                               void *out);
static int muloc_build_ble(const void *runtime, struct block_config_ble *out);
#endif

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
static int cir_read_build_runtime(const struct block_config_ble *ble,
                                  block_result_cb_t cb, void *cb_data,
                                  void *out);
static int cir_read_build_ble(const void *runtime, struct block_config_ble *out);
#endif

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
static int butler_build_runtime(const struct block_config_ble *ble,
                                block_result_cb_t cb, void *cb_data,
                                void *out);
static int butler_build_ble(const void *runtime, struct block_config_ble *out);
#endif

/* Block type registry */
static const struct block_type_info block_registry[] = {
    {
        .type = BLOCK_TYPE_GLOSSY,
        .name = "glossy",
        .handler = glossy_block_handler,
        .runtime_config_size = sizeof(struct glossy_block_config),
        .build_runtime = glossy_build_runtime,
        .build_ble = glossy_build_ble,
        .needs_sync = false,
    },
    {
        .type = BLOCK_TYPE_MTM,
        .name = "mtm",
        .handler = mtm_block_handler,
        .runtime_config_size = sizeof(struct mtm_block_config),
        .build_runtime = mtm_build_runtime,
        .build_ble = mtm_build_ble,
        .needs_sync = true,
    },
    {
        .type = BLOCK_TYPE_MM_REFERENCE,
        .name = "mm_reference",
        .handler = mm_reference_block_handler,
        .runtime_config_size = sizeof(struct mm_reference_block_config),
        .build_runtime = mm_reference_build_runtime,
        .build_ble = mm_reference_build_ble,
        .needs_sync = true,
    },
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_LS_POSITION)
    {
        .type = BLOCK_TYPE_LS_POSITION,
        .name = "ls_position",
        .handler = ls_position_block_handler,
        .runtime_config_size = sizeof(struct ls_position_block_config),
        .build_runtime = ls_position_build_runtime,
        .build_ble = ls_position_build_ble,
        .needs_sync = false,
    },
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_PF_POSITION)
    {
        .type = BLOCK_TYPE_PF_POSITION,
        .name = "pf_position",
        .handler = pf_position_block_handler,
        .runtime_config_size = sizeof(struct pf_position_block_config),
        .build_runtime = pf_position_build_runtime,
        .build_ble = pf_position_build_ble,
        .needs_sync = false,
    },
#endif
    {
        .type = BLOCK_TYPE_TIME_SYNC_CHECK,
        .name = "time_sync_check",
        .handler = time_sync_check_block_handler,
        .runtime_config_size = sizeof(struct time_sync_check_config),
        .build_runtime = time_sync_check_build_runtime,
        .build_ble = time_sync_check_build_ble,
        .needs_sync = true,
    },
    {
        .type = BLOCK_TYPE_ANNOUNCEMENT,
        .name = "announcement",
        .handler = announcement_block_handler,
        .runtime_config_size = sizeof(struct announcement_block_config),
        .build_runtime = announcement_build_runtime,
        .build_ble = announcement_build_ble,
        .needs_sync = true,
    },
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_SWARM_RANGING)
    {
        .type = BLOCK_TYPE_SWARM_RANGING,
        .name = "swarm_ranging",
        .handler = swarm_ranging_block_handler,
        .runtime_config_size = sizeof(struct swarm_ranging_block_config),
        .build_runtime = swarm_ranging_build_runtime,
        .build_ble = swarm_ranging_build_ble,
        .needs_sync = false,
    },
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_MULOC)
    {
        .type = BLOCK_TYPE_MULOC,
        .name = "muloc",
        .handler = muloc_block_handler,
        .runtime_config_size = sizeof(struct muloc_block_config),
        .build_runtime = muloc_build_runtime,
        .build_ble = muloc_build_ble,
        .needs_sync = true,
    },
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
    {
        .type = BLOCK_TYPE_CIR_READ,
        .name = "cir_read",
        .handler = cir_read_block_handler,
        .runtime_config_size = sizeof(struct cir_read_block_config),
        .build_runtime = cir_read_build_runtime,
        .build_ble = cir_read_build_ble,
        .needs_sync = false,
    },
#endif
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
    {
        .type = BLOCK_TYPE_BUTLER,
        .name = "butler",
        .handler = butler_block_handler,
        .runtime_config_size = sizeof(struct butler_block_config),
        .build_runtime = butler_build_runtime,
        .build_ble = butler_build_ble,
        .needs_sync = false,
    },
#endif
    {
        .type = BLOCK_TYPE_IDLE,
        .name = "idle",
        .handler = idle_block_handler,
        .runtime_config_size = 0,
        .build_runtime = NULL,
        .build_ble = NULL,
        .needs_sync = false,
    },
};

const struct block_type_info *block_type_get_info(block_type_t type)
{
    for (size_t i = 0; i < ARRAY_SIZE(block_registry); i++) {
        if (block_registry[i].type == type) {
            return &block_registry[i];
        }
    }
    return NULL;
}

const char *block_type_get_name(block_type_t type)
{
    const struct block_type_info *info = block_type_get_info(type);
    return info ? info->name : "unknown";
}

int block_type_validate_config(const struct block_config_ble *config)
{
    if (config->type == BLOCK_TYPE_NONE) {
        return 0; /* Empty slot is valid */
    }

    if (config->type >= BLOCK_TYPE_MAX) {
        LOG_ERR("Invalid block type: %u", config->type);
        return -EINVAL;
    }

    const struct block_type_info *info = block_type_get_info(config->type);
    if (!info) {
        LOG_ERR("Unsupported block type: %u", config->type);
        return -ENOTSUP;
    }

    /* Type-specific validation */
    switch (config->type) {
    case BLOCK_TYPE_GLOSSY:
        if (config->config.glossy.max_depth == 0) {
            LOG_ERR("Glossy max_depth must be > 0");
            return -EINVAL;
        }
        break;

    case BLOCK_TYPE_MTM:
        if (config->config.mtm.schedule_type > BLE_SCHEDULE_CONTENTION) {
            LOG_ERR("Invalid MTM schedule_type: %u", config->config.mtm.schedule_type);
            return -EINVAL;
        }
        if (config->config.mtm.slots_per_phase == 0 || config->config.mtm.phases == 0) {
            LOG_ERR("MTM slots_per_phase and phases must be > 0");
            return -EINVAL;
        }
        break;

    case BLOCK_TYPE_MM:
        if (config->config.mm.schedule_type > BLE_SCHEDULE_CONTENTION) {
            LOG_ERR("Invalid MM schedule_type: %u", config->config.mm.schedule_type);
            return -EINVAL;
        }
        if (config->config.mm.slots_per_phase == 0 || config->config.mm.phases == 0) {
            LOG_ERR("MM slots_per_phase and phases must be > 0");
            return -EINVAL;
        }
        break;

    case BLOCK_TYPE_MM_REFERENCE:
        /* MM reference has reasonable defaults, minimal validation */
        break;

    case BLOCK_TYPE_LS_POSITION:
        if (config->config.ls_position.min_anchors < 3) {
            LOG_WRN("LS position min_anchors < 3 may not produce valid results");
        }
        break;

    case BLOCK_TYPE_PF_POSITION:
        if (config->config.pf_position.min_anchors < 3) {
            LOG_WRN("PF position min_anchors < 3 may not produce valid results");
        }
        if (config->config.pf_position.particle_count == 0) {
            LOG_ERR("PF position particle_count must be > 0");
            return -EINVAL;
        }
        break;

    case BLOCK_TYPE_TIME_SYNC_CHECK:
        /* No specific validation needed */
        break;

    case BLOCK_TYPE_ANNOUNCEMENT:
        if (config->config.announcement.max_depth == 0) {
            LOG_ERR("Announcement max_depth must be > 0");
            return -EINVAL;
        }
        if (config->config.announcement.announce_probability_pct > 100) {
            LOG_ERR("Announcement probability must be 0-100");
            return -EINVAL;
        }
        break;

    case BLOCK_TYPE_SWARM_RANGING:
        /* No specific validation needed */
        break;

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
    case BLOCK_TYPE_CIR_READ:
        if (config->config.cir_read.mode > 1) {
            LOG_ERR("CIR read: mode must be 0 (sender) or 1 (receiver)");
            return -EINVAL;
        }
        if (config->config.cir_read.mode == CIR_MODE_RECEIVER) {
            if (!config->config.cir_read.only_first_path &&
                config->config.cir_read.from_index >= config->config.cir_read.to_index) {
                LOG_ERR("CIR read: from_index must be < to_index");
                return -EINVAL;
            }
            if (config->config.cir_read.to_index > CIR_READ_MAX_SAMPLES) {
                LOG_ERR("CIR read: to_index exceeds max samples (%d)",
                        CIR_READ_MAX_SAMPLES);
                return -EINVAL;
            }
        }
        break;
#endif

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_MULOC)
    case BLOCK_TYPE_MULOC:
        if (config->config.muloc.anchor_count < 2 ||
            config->config.muloc.anchor_count > MULOC_MAX_ANCHORS) {
            LOG_ERR("MULoc anchor_count must be 2-%d", MULOC_MAX_ANCHORS);
            return -EINVAL;
        }
        if (config->config.muloc.anchor_id != 0xFF &&
            config->config.muloc.anchor_id >= config->config.muloc.anchor_count) {
            LOG_ERR("MULoc anchor_id must be < anchor_count or 0xFF");
            return -EINVAL;
        }
        if (config->config.muloc.num_rounds == 0 ||
            config->config.muloc.num_rounds > MULOC_MAX_ROUNDS) {
            LOG_ERR("MULoc num_rounds must be 1-%d", MULOC_MAX_ROUNDS);
            return -EINVAL;
        }
        break;
#endif

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
    case BLOCK_TYPE_BUTLER:
        if (config->config.butler.max_subslots == 0) {
            LOG_ERR("Butler max_subslots must be > 0");
            return -EINVAL;
        }
        if (config->config.butler.p_tx_pct > 100) {
            LOG_ERR("Butler p_tx_pct must be 0-100");
            return -EINVAL;
        }
        break;
#endif

    default:
        break;
    }

    return 0;
}

/* ========================================================================== */
/* Converter implementations                                                   */
/* ========================================================================== */

static int glossy_build_runtime(const struct block_config_ble *ble,
                                block_result_cb_t cb, void *cb_data,
                                void *out)
{
    struct glossy_block_config *cfg = out;

    cfg->max_depth = ble->config.glossy.max_depth;
    cfg->transmission_delay_us = ble->config.glossy.transmission_delay_us;
    cfg->guard_period_us = ble->config.glossy.guard_period_us;
    cfg->channel = ble->config.glossy.channel;

    /* Glossy block has no result callback */
    (void)cb;
    (void)cb_data;

    return 0;
}

static int glossy_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct glossy_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_GLOSSY;
    out->config.glossy.max_depth = cfg->max_depth;
    out->config.glossy.transmission_delay_us = cfg->transmission_delay_us;
    out->config.glossy.guard_period_us = cfg->guard_period_us;
    out->config.glossy.channel = cfg->channel;

    return 0;
}

/* Wrapper to convert generic callback to mtm_cb_t signature */
struct mtm_cb_wrapper {
    block_result_cb_t cb;
    void *user_data;
};

static void mtm_cb_adapter(mtm_status_t status, const struct mtm_block_result *result, void *user_data)
{
    struct mtm_cb_wrapper *wrapper = user_data;
    /* Only call the callback on success - error means no valid result */
    if (status != MTM_STATUS_SUCCESS || result == NULL) {
        return;
    }
    if (wrapper && wrapper->cb) {
        wrapper->cb(BLOCK_TYPE_MTM, (void *)result, wrapper->user_data);
    }
}

static int mtm_build_runtime(const struct block_config_ble *ble,
                             block_result_cb_t cb, void *cb_data,
                             void *out)
{
    struct mtm_block_config *cfg = out;

    cfg->schedule_type = (schedule_type_t)ble->config.mtm.schedule_type;
    cfg->ranging_round_slots_per_phase = ble->config.mtm.slots_per_phase;
    cfg->ranging_round_phases = ble->config.mtm.phases;
    cfg->slot_padding_us = ble->config.mtm.slot_padding_us;

    /* Store callback - the superframe_settings module manages wrapper lifetime */
    if (cb) {
        /* Note: The caller must ensure cb_data points to a persistent wrapper struct */
        cfg->mtm_cb = mtm_cb_adapter;
        cfg->cb_user_data = cb_data;
    } else {
        cfg->mtm_cb = NULL;
        cfg->cb_user_data = NULL;
    }

    return 0;
}

static int mtm_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct mtm_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_MTM;
    out->config.mtm.schedule_type = (uint8_t)cfg->schedule_type;
    out->config.mtm.slots_per_phase = cfg->ranging_round_slots_per_phase;
    out->config.mtm.phases = cfg->ranging_round_phases;
    out->config.mtm.slot_padding_us = cfg->slot_padding_us;

    return 0;
}

/* MM Reference block callback wrapper */
static void mm_reference_cb_adapter(mtm_status_t status,
                                    const struct mm_reference_block_result *result,
                                    void *user_data)
{
    struct mtm_cb_wrapper *wrapper = user_data;
    /* Only call the callback on success - error means no valid result */
    if (status != MTM_STATUS_SUCCESS || result == NULL) {
        return;
    }
    if (wrapper && wrapper->cb) {
        wrapper->cb(BLOCK_TYPE_MM_REFERENCE, (void *)result, wrapper->user_data);
    }
}

static int mm_reference_build_runtime(const struct block_config_ble *ble,
                                      block_result_cb_t cb, void *cb_data,
                                      void *out)
{
    struct mm_reference_block_config *cfg = out;

    cfg->respond_interval_us = ble->config.mm_reference.respond_interval_us;
    cfg->guard_period_us = ble->config.mm_reference.guard_period_us;
    cfg->timeout_us = ble->config.mm_reference.timeout_us;
    cfg->initiator_addr = ble->config.mm_reference.initiator_addr;
    cfg->responder_addr = ble->config.mm_reference.responder_addr;
    cfg->channel = ble->config.mm_reference.channel;

    if (cb) {
        cfg->mm_ref_cb = mm_reference_cb_adapter;
        cfg->cb_user_data = cb_data;
    } else {
        cfg->mm_ref_cb = NULL;
        cfg->cb_user_data = NULL;
    }

    return 0;
}

static int mm_reference_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct mm_reference_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_MM_REFERENCE;
    out->config.mm_reference.respond_interval_us = cfg->respond_interval_us;
    out->config.mm_reference.guard_period_us = cfg->guard_period_us;
    out->config.mm_reference.timeout_us = cfg->timeout_us;
    out->config.mm_reference.initiator_addr = cfg->initiator_addr;
    out->config.mm_reference.responder_addr = cfg->responder_addr;
    out->config.mm_reference.channel = cfg->channel;

    return 0;
}

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_LS_POSITION)
/* LS Position callback wrapper */
static void ls_position_cb_adapter(const struct vec3d_f *position, float residual, void *user_data)
{
    struct mtm_cb_wrapper *wrapper = user_data;
    if (wrapper && wrapper->cb) {
        /* Pack position and residual into a result struct */
        struct {
            const struct vec3d_f *position;
            float residual;
        } result = { position, residual };
        wrapper->cb(BLOCK_TYPE_LS_POSITION, (void *)&result, wrapper->user_data);
    }
}

static int ls_position_build_runtime(const struct block_config_ble *ble,
                                     block_result_cb_t cb, void *cb_data,
                                     void *out)
{
    struct ls_position_block_config *cfg = out;

    cfg->min_anchors = ble->config.ls_position.min_anchors;
    cfg->max_age_ms = ble->config.ls_position.max_age_ms;

    /* Parse flags - default to constrain Z positive if flags byte is 0
     * (backwards compatible: old configs with flags=0 get the safe default) */
    uint8_t flags = ble->config.ls_position.flags;
    if (flags == 0) {
        /* Default: enable Z constraint */
        cfg->constrain_z_positive = true;
    } else {
        cfg->constrain_z_positive = (flags & LS_POSITION_FLAG_CONSTRAIN_Z_POSITIVE) != 0;
    }

    if (cb) {
        cfg->position_cb = ls_position_cb_adapter;
        cfg->cb_user_data = cb_data;
    } else {
        cfg->position_cb = NULL;
        cfg->cb_user_data = NULL;
    }

    return 0;
}

static int ls_position_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct ls_position_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_LS_POSITION;
    out->config.ls_position.min_anchors = cfg->min_anchors;
    out->config.ls_position.max_age_ms = cfg->max_age_ms;
    out->config.ls_position.flags = cfg->constrain_z_positive ?
        LS_POSITION_FLAG_CONSTRAIN_Z_POSITIVE : 0;

    return 0;
}
#endif /* CONFIG_SYNCHROFLY_BLOCK_LS_POSITION */

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_PF_POSITION)
/* PF Position callback wrapper */
static void pf_position_cb_adapter(const struct vec3d_f *position, float variance, void *user_data)
{
    struct mtm_cb_wrapper *wrapper = user_data;
    if (wrapper && wrapper->cb) {
        /* Pack position and variance into a result struct */
        struct {
            const struct vec3d_f *position;
            float variance;
        } result = { position, variance };
        wrapper->cb(BLOCK_TYPE_PF_POSITION, (void *)&result, wrapper->user_data);
    }
}

static int pf_position_build_runtime(const struct block_config_ble *ble,
                                     block_result_cb_t cb, void *cb_data,
                                     void *out)
{
    struct pf_position_block_config *cfg = out;

    cfg->min_anchors = ble->config.pf_position.min_anchors;
    cfg->max_age_ms = ble->config.pf_position.max_age_ms;
    cfg->particle_count = ble->config.pf_position.particle_count;
    cfg->measurement_variance = ble->config.pf_position.measurement_variance_x10 / 10.0f;
    cfg->process_noise_std = ble->config.pf_position.process_noise_std_x100 / 100.0f;
    cfg->send_particles = ble->config.pf_position.send_particles != 0;

    if (cb) {
        cfg->position_cb = pf_position_cb_adapter;
        cfg->cb_user_data = cb_data;
    } else {
        cfg->position_cb = NULL;
        cfg->cb_user_data = NULL;
    }

    return 0;
}

static int pf_position_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct pf_position_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_PF_POSITION;
    out->config.pf_position.min_anchors = cfg->min_anchors;
    out->config.pf_position.max_age_ms = cfg->max_age_ms;
    out->config.pf_position.particle_count = cfg->particle_count;
    out->config.pf_position.measurement_variance_x10 = (uint8_t)(cfg->measurement_variance * 10.0f);
    out->config.pf_position.process_noise_std_x100 = (uint8_t)(cfg->process_noise_std * 100.0f);
    out->config.pf_position.send_particles = cfg->send_particles ? 1 : 0;

    return 0;
}
#endif /* CONFIG_SYNCHROFLY_BLOCK_PF_POSITION */

static int time_sync_check_build_runtime(const struct block_config_ble *ble,
                                         block_result_cb_t cb, void *cb_data,
                                         void *out)
{
    struct time_sync_check_config *cfg = out;

    cfg->isNetworkRoot = ble->config.time_sync_check.is_network_root != 0;

    /* Time sync check has no result callback */
    (void)cb;
    (void)cb_data;

    return 0;
}

static int time_sync_check_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct time_sync_check_config *cfg = runtime;

    out->type = BLOCK_TYPE_TIME_SYNC_CHECK;
    out->config.time_sync_check.is_network_root = cfg->isNetworkRoot ? 1 : 0;

    return 0;
}

static int announcement_build_runtime(const struct block_config_ble *ble,
                                      block_result_cb_t cb, void *cb_data,
                                      void *out)
{
    struct announcement_block_config *cfg = out;

    cfg->max_depth = ble->config.announcement.max_depth;
    cfg->transmission_delay_us = ble->config.announcement.transmission_delay_us;
    cfg->guard_period_us = ble->config.announcement.guard_period_us;
    cfg->channel = ble->config.announcement.channel;
    cfg->announce_probability_pct = ble->config.announcement.announce_probability_pct;

    /* Announcement block has no result callback */
    (void)cb;
    (void)cb_data;

    return 0;
}

static int announcement_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct announcement_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_ANNOUNCEMENT;
    out->config.announcement.max_depth = cfg->max_depth;
    out->config.announcement.transmission_delay_us = cfg->transmission_delay_us;
    out->config.announcement.guard_period_us = cfg->guard_period_us;
    out->config.announcement.channel = cfg->channel;
    out->config.announcement.announce_probability_pct = cfg->announce_probability_pct;

    return 0;
}

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_SWARM_RANGING)
static int swarm_ranging_build_runtime(const struct block_config_ble *ble,
                                       block_result_cb_t cb, void *cb_data,
                                       void *out)
{
    struct swarm_ranging_block_config *cfg = out;

    cfg->channel = ble->config.swarm_ranging.channel;
    cfg->period_ms = ble->config.swarm_ranging.period_ms;
    cfg->filter_enabled = ble->config.swarm_ranging.filter_enabled;
    cfg->bus_boarding_enabled = ble->config.swarm_ranging.bus_boarding_enabled;

    /* Swarm ranging block has no result callback */
    (void)cb;
    (void)cb_data;

    return 0;
}

static int swarm_ranging_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct swarm_ranging_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_SWARM_RANGING;
    out->config.swarm_ranging.channel = cfg->channel;
    out->config.swarm_ranging.filter_enabled = cfg->filter_enabled ? 1 : 0;
    out->config.swarm_ranging.period_ms = cfg->period_ms;
    out->config.swarm_ranging.bus_boarding_enabled = cfg->bus_boarding_enabled ? 1 : 0;

    return 0;
}
#endif /* CONFIG_SYNCHROFLY_BLOCK_SWARM_RANGING */

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_MULOC)
/* MULoc callback adapter */
static void muloc_cb_adapter(muloc_status_t status,
                             const struct muloc_block_result *result,
                             void *user_data)
{
    struct mtm_cb_wrapper *wrapper = user_data;
    if (status != MULOC_STATUS_SUCCESS || result == NULL) {
        return;
    }
    if (wrapper && wrapper->cb) {
        wrapper->cb(BLOCK_TYPE_MULOC, (void *)result, wrapper->user_data);
    }
}

static int muloc_build_runtime(const struct block_config_ble *ble,
                               block_result_cb_t cb, void *cb_data,
                               void *out)
{
    struct muloc_block_config *cfg = out;

    cfg->anchor_count = ble->config.muloc.anchor_count;
    cfg->anchor_id = ble->config.muloc.anchor_id;
    cfg->num_rounds = ble->config.muloc.num_rounds;
    cfg->delay_time_us = ble->config.muloc.delay_time_us;
    cfg->start_channel = ble->config.muloc.start_channel;
    cfg->hop_channel = ble->config.muloc.hop_channel;

    if (cb) {
        cfg->muloc_cb = muloc_cb_adapter;
        cfg->cb_user_data = cb_data;
    } else {
        cfg->muloc_cb = NULL;
        cfg->cb_user_data = NULL;
    }

    return 0;
}

static int muloc_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct muloc_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_MULOC;
    out->config.muloc.anchor_count = cfg->anchor_count;
    out->config.muloc.anchor_id = cfg->anchor_id;
    out->config.muloc.num_rounds = cfg->num_rounds;
    out->config.muloc.delay_time_us = cfg->delay_time_us;
    out->config.muloc.start_channel = cfg->start_channel;
    out->config.muloc.hop_channel = cfg->hop_channel;
    out->config.muloc.reserved = 0;

    return 0;
}
#endif /* CONFIG_SYNCHROFLY_BLOCK_MULOC */

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
/* CIR Read callback adapter */
static void cir_read_cb_adapter(cir_read_status_t status,
                                const struct cir_read_block_result *result,
                                void *user_data)
{
    struct mtm_cb_wrapper *wrapper = user_data;
    if (status != CIR_READ_STATUS_SUCCESS || result == NULL) {
        return;
    }
    if (wrapper && wrapper->cb) {
        wrapper->cb(BLOCK_TYPE_CIR_READ, (void *)result, wrapper->user_data);
    }
}

static int cir_read_build_runtime(const struct block_config_ble *ble,
                                  block_result_cb_t cb, void *cb_data,
                                  void *out)
{
    struct cir_read_block_config *cfg = out;

    cfg->mode = ble->config.cir_read.mode;
    cfg->channel = ble->config.cir_read.channel;
    cfg->tx_delay_dtu = ble->config.cir_read.tx_delay_dtu;
    cfg->from_index = ble->config.cir_read.from_index;
    cfg->to_index = ble->config.cir_read.to_index;
    cfg->only_first_path = ble->config.cir_read.only_first_path;

    if (cb) {
        cfg->cir_cb = cir_read_cb_adapter;
        cfg->cb_user_data = cb_data;
    } else {
        cfg->cir_cb = NULL;
        cfg->cb_user_data = NULL;
    }

    return 0;
}

static int cir_read_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct cir_read_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_CIR_READ;
    out->config.cir_read.mode = cfg->mode;
    out->config.cir_read.channel = cfg->channel;
    out->config.cir_read.tx_delay_dtu = cfg->tx_delay_dtu;
    out->config.cir_read.from_index = cfg->from_index;
    out->config.cir_read.to_index = cfg->to_index;
    out->config.cir_read.only_first_path = cfg->only_first_path;
    memset(out->config.cir_read._reserved, 0,
           sizeof(out->config.cir_read._reserved));

    return 0;
}
#endif /* CONFIG_SYNCHROFLY_BLOCK_CIR_READ */

/* ========================================================================== */
/* Butler converters                                                           */
/* ========================================================================== */
#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_BUTLER)

static int butler_build_runtime(const struct block_config_ble *ble,
                                block_result_cb_t cb, void *cb_data,
                                void *out)
{
    struct butler_block_config *cfg = out;

    cfg->max_subslots = ble->config.butler.max_subslots;
    cfg->subslot_duration_us = ble->config.butler.subslot_duration_us;
    cfg->guard_period_us = ble->config.butler.guard_period_us;
    cfg->p_tx_pct = ble->config.butler.p_tx_pct;
    cfg->channel = ble->config.butler.channel;

    /* Butler block has no result callback */
    (void)cb;
    (void)cb_data;

    return 0;
}

static int butler_build_ble(const void *runtime, struct block_config_ble *out)
{
    const struct butler_block_config *cfg = runtime;

    out->type = BLOCK_TYPE_BUTLER;
    out->config.butler.max_subslots = cfg->max_subslots;
    out->config.butler.subslot_duration_us = cfg->subslot_duration_us;
    out->config.butler.guard_period_us = cfg->guard_period_us;
    out->config.butler.p_tx_pct = cfg->p_tx_pct;
    out->config.butler.channel = cfg->channel;

    return 0;
}

#endif /* CONFIG_SYNCHROFLY_BLOCK_BUTLER */
