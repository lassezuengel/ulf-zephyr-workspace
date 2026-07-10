#include <app/lib/management/network_settings.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include <app/lib/blocks/mtm.h>

LOG_MODULE_REGISTER(network_settings, CONFIG_LOG_DEFAULT_LEVEL);

struct network_settings {
    schedule_type_t schedule_type;
    uint32_t scheduler_slot_duration_ms;
    uint8_t ranging_round_slots_per_phase;
    uint8_t ranging_round_phases;
    uint16_t glossy_guard_us;
    uint16_t superframe_slots;
    uint16_t glossy_max_depth;
    uint16_t glossy_transmission_delay_us;
    uint16_t slot_padding_us;
};

static struct network_settings settings = {
    .schedule_type = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_SCHEDULE_TYPE,
    .ranging_round_slots_per_phase = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_RANGING_ROUND_SLOTS_PER_PHASE,
    .ranging_round_phases = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_RANGING_ROUND_PHASES,
    .scheduler_slot_duration_ms = CONFIG_DEFAULT_ROUND_DURATION,
    .glossy_guard_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US,
    .superframe_slots = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_SUPERFRAME_SLOTS,
    .glossy_max_depth = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH,
    .glossy_transmission_delay_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US,
    .slot_padding_us = CONFIG_SYNCHROFLY_SCHEDULING_SLOT_PADDING_US
};

static int settings_set(const char *name, size_t len,
    settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    int rc;

    if (settings_name_steq(name, "schedule_type", &next) && !next) {
        if (len != sizeof(settings.schedule_type)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.schedule_type,
                   sizeof(settings.schedule_type));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "ranging_round_slots_per_phase", &next) && !next) {
        if (len != sizeof(settings.ranging_round_slots_per_phase)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.ranging_round_slots_per_phase,
                   sizeof(settings.ranging_round_slots_per_phase));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "ranging_round_phases", &next) && !next) {
        if (len != sizeof(settings.ranging_round_phases)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.ranging_round_phases,
            sizeof(settings.ranging_round_phases));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "scheduler_slot_duration_ms", &next) && !next) {
        if (len != sizeof(settings.scheduler_slot_duration_ms)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.scheduler_slot_duration_ms,
            sizeof(settings.scheduler_slot_duration_ms));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "glossy_guard_us", &next) && !next) {
        if (len != sizeof(settings.glossy_guard_us)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.glossy_guard_us,
            sizeof(settings.glossy_guard_us));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "superframe_slots", &next) && !next) {
        if (len != sizeof(settings.superframe_slots)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.superframe_slots,
            sizeof(settings.superframe_slots));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "glossy_max_depth", &next) && !next) {
        if (len != sizeof(settings.glossy_max_depth)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.glossy_max_depth,
            sizeof(settings.glossy_max_depth));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "glossy_transmission_delay_us", &next) && !next) {
        if (len != sizeof(settings.glossy_transmission_delay_us)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.glossy_transmission_delay_us,
            sizeof(settings.glossy_transmission_delay_us));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "slot_padding_us", &next) && !next) {
        if (len != sizeof(settings.slot_padding_us)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.slot_padding_us,
            sizeof(settings.slot_padding_us));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    return -ENOENT;
}

/*---- Getters -----*/
schedule_type_t network_get_schedule_type(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return CONFIG_SYNCHROFLY_NETWORK_DEFAULT_SCHEDULE_TYPE;
#else
    return settings.schedule_type;
#endif
}

uint8_t network_get_ranging_round_slots_per_phase(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return CONFIG_SYNCHROFLY_NETWORK_DEFAULT_RANGING_ROUND_SLOTS_PER_PHASE;
#else
    return settings.ranging_round_slots_per_phase;
#endif
}

uint32_t network_get_scheduler_slot_duration_ms(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return CONFIG_DEFAULT_ROUND_DURATION;
#else
    return settings.scheduler_slot_duration_ms;
#endif
}

uint8_t network_get_ranging_round_phases(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return CONFIG_SYNCHROFLY_NETWORK_DEFAULT_RANGING_ROUND_PHASES;
#else
    return settings.ranging_round_phases;
#endif
}

uint16_t network_get_glossy_guard_us(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US;
#else
    return settings.glossy_guard_us;
#endif
}

uint16_t network_get_superframe_slots(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return CONFIG_SYNCHROFLY_NETWORK_DEFAULT_SUPERFRAME_SLOTS;
#else
    return settings.superframe_slots;
#endif
}

uint16_t network_get_glossy_max_depth(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH;
#else
    return settings.glossy_max_depth;
#endif
}

uint16_t network_get_glossy_transmission_delay_us(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US;
#else
    return settings.glossy_transmission_delay_us;
#endif
}

uint16_t network_get_slot_padding_us(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return CONFIG_SYNCHROFLY_SCHEDULING_SLOT_PADDING_US;
#else
    return settings.slot_padding_us;
#endif
}

/*---- Setters ----*/
void network_set_schedule_type(schedule_type_t schedule_type) {
    settings.schedule_type = schedule_type;
    settings_save_one("network/schedule_type", &settings.schedule_type,
                    sizeof(settings.schedule_type));
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    LOG_INF("Saved to NVS but not active (COMPILE_TIME mode uses CONFIG values)");
#else
    LOG_DBG("Applied and saved to NVS");
#endif
}

void network_set_ranging_round_slots_per_phase(uint8_t slots) {
    settings.ranging_round_slots_per_phase = slots;
    settings_save_one("network/ranging_round_slots_per_phase", &settings.ranging_round_slots_per_phase,
                    sizeof(settings.ranging_round_slots_per_phase));
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    LOG_INF("Saved to NVS but not active (COMPILE_TIME mode uses CONFIG values)");
#else
    LOG_DBG("Applied and saved to NVS");
#endif
}

void network_set_scheduler_slot_duration_ms(uint32_t duration) {
    settings.scheduler_slot_duration_ms = duration;
    settings_save_one("network/scheduler_slot_duration_ms", &settings.scheduler_slot_duration_ms,
                    sizeof(settings.scheduler_slot_duration_ms));
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    LOG_INF("Saved to NVS but not active (COMPILE_TIME mode uses CONFIG values)");
#else
    LOG_DBG("Applied and saved to NVS");
#endif
}

void network_set_ranging_round_phases(uint8_t phases) {
    settings.ranging_round_phases = phases;
    settings_save_one("network/ranging_round_phases", &settings.ranging_round_phases,
        sizeof(settings.ranging_round_phases));
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    LOG_INF("Saved to NVS but not active (COMPILE_TIME mode uses CONFIG values)");
#else
    LOG_DBG("Applied and saved to NVS");
#endif
}

void network_set_glossy_guard_us(uint16_t guards_us) {
    settings.glossy_guard_us = guards_us;
    settings_save_one("network/glossy_guard_us", &settings.glossy_guard_us,
        sizeof(settings.glossy_guard_us));
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    LOG_INF("Saved to NVS but not active (COMPILE_TIME mode uses CONFIG values)");
#else
    LOG_DBG("Applied and saved to NVS");
#endif
}

void network_set_superframe_slots(uint16_t slots) {
    settings.superframe_slots = slots;
    settings_save_one("network/superframe_slots", &settings.superframe_slots,
        sizeof(settings.superframe_slots));
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    LOG_INF("Saved to NVS but not active (COMPILE_TIME mode uses CONFIG values)");
#else
    LOG_DBG("Applied and saved to NVS");
#endif
}

void network_set_glossy_max_depth(uint16_t max_depth) {
    settings.glossy_max_depth = max_depth;
    settings_save_one("network/glossy_max_depth", &settings.glossy_max_depth,
        sizeof(settings.glossy_max_depth));
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    LOG_INF("Saved to NVS but not active (COMPILE_TIME mode uses CONFIG values)");
#else
    LOG_DBG("Applied and saved to NVS");
#endif
}

void network_set_glossy_transmission_delay_us(uint16_t delay_us) {
    settings.glossy_transmission_delay_us = delay_us;
    settings_save_one("network/glossy_transmission_delay_us", &settings.glossy_transmission_delay_us,
        sizeof(settings.glossy_transmission_delay_us));
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    LOG_INF("Saved to NVS but not active (COMPILE_TIME mode uses CONFIG values)");
#else
    LOG_DBG("Applied and saved to NVS");
#endif
}

void network_set_slot_padding_us(uint16_t padding_us) {
    settings.slot_padding_us = padding_us;
    settings_save_one("network/slot_padding_us", &settings.slot_padding_us,
        sizeof(settings.slot_padding_us));
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    LOG_INF("Saved to NVS but not active (COMPILE_TIME mode uses CONFIG values)");
#else
    LOG_DBG("Applied and saved to NVS");
#endif
}

const char* network_get_settings_source(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    return "compile-time";
#else
    return "dynamic";
#endif
}

void network_save_config_defaults_to_nvs(void) {
    LOG_INF("Saving CONFIG defaults to NVS...");

    network_set_schedule_type(CONFIG_SYNCHROFLY_NETWORK_DEFAULT_SCHEDULE_TYPE);
    network_set_ranging_round_slots_per_phase(CONFIG_SYNCHROFLY_NETWORK_DEFAULT_RANGING_ROUND_SLOTS_PER_PHASE);
    network_set_ranging_round_phases(CONFIG_SYNCHROFLY_NETWORK_DEFAULT_RANGING_ROUND_PHASES);
    network_set_scheduler_slot_duration_ms(CONFIG_DEFAULT_ROUND_DURATION);
    network_set_glossy_guard_us(CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US);
    network_set_superframe_slots(CONFIG_SYNCHROFLY_NETWORK_DEFAULT_SUPERFRAME_SLOTS);
    network_set_glossy_max_depth(CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH);
    network_set_glossy_transmission_delay_us(CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US);
    network_set_slot_padding_us(CONFIG_SYNCHROFLY_SCHEDULING_SLOT_PADDING_US);

    LOG_INF("CONFIG defaults saved to NVS");
}

static void network_validate_settings(void) {
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    // In COMPILE_TIME mode, warn about NVS/CONFIG mismatches
    if (settings.glossy_transmission_delay_us != CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US) {
        LOG_WRN("NVS override detected (ignored in COMPILE_TIME mode):");
        LOG_WRN("  glossy_transmission_delay_us: NVS=%u, CONFIG=%u (using CONFIG)",
                settings.glossy_transmission_delay_us,
                CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US);
    }
    if (settings.glossy_max_depth != CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH) {
        LOG_WRN("  glossy_max_depth: NVS=%u, CONFIG=%u (using CONFIG)",
                settings.glossy_max_depth,
                CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH);
    }
    if (settings.glossy_guard_us != CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US) {
        LOG_WRN("  glossy_guard_us: NVS=%u, CONFIG=%u (using CONFIG)",
                settings.glossy_guard_us,
                CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US);
    }
#endif
}

void network_print_settings(void) {
    const char *schedule_type_str;
    switch (network_get_schedule_type()) {
        case SCHEDULE_BASIC:
            schedule_type_str = "basic";
            break;
        case SCHEDULE_HASHED:
            schedule_type_str = "hashed";
            break;
        case SCHEDULE_CONTENTION:
            schedule_type_str = "contention";
            break;
        default:
            schedule_type_str = "unknown";
            break;
    }

    LOG_WRN("Network Settings (source: %s):", network_get_settings_source());
    LOG_WRN("  Schedule Type: %s", schedule_type_str);
    LOG_WRN("  Slot Duration: %u ms", network_get_scheduler_slot_duration_ms());
    LOG_WRN("  Slot Padding: %u us", network_get_slot_padding_us());
    LOG_WRN("  Superframe Slots: %u", network_get_superframe_slots());
    LOG_WRN("  Ranging Slots/Phase: %u", network_get_ranging_round_slots_per_phase());
    LOG_WRN("  Ranging Phases: %u", network_get_ranging_round_phases());
    LOG_WRN("  Glossy TX Delay: %u us", network_get_glossy_transmission_delay_us());
    LOG_WRN("  Glossy Guard: %u us", network_get_glossy_guard_us());
    LOG_WRN("  Glossy Max Depth: %u", network_get_glossy_max_depth());

    // In COMPILE_TIME mode, also show NVS values if they differ
#if IS_ENABLED(CONFIG_SYNCHROFLY_NETWORK_SETTINGS_SOURCE_COMPILE_TIME)
    if (settings.glossy_transmission_delay_us != CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US) {
        LOG_WRN("  (NVS contains: %u us, ignored)", settings.glossy_transmission_delay_us);
    }
#endif

    network_validate_settings();
}

SETTINGS_STATIC_HANDLER_DEFINE(network, "network", NULL, settings_set, NULL, NULL);
