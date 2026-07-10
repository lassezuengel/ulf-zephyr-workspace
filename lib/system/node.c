#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include <hal/nrf_ficr.h>

#include <app/lib/system/node.h>
#include <zephyr/settings/settings.h>
#include <string.h>
#include <errno.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(node, CONFIG_LOG_DEFAULT_LEVEL);

static node_mode_t node_mode = NODE_MODE_ACTIVE;

/* Node position storage (absolute coordinates in meters) */
static struct node_config_position node_pos = {
    .x = 0.0,
    .y = 0.0,
    .z = 0.0
};
static bool node_pos_valid = false;

/* Network root status (Glossy flood initiator) */
static bool node_is_root = false;
static bool node_is_root_initialized = false;

/* Free-running mode (scheduler runs without time sync) */
static bool node_free_running = false;

/* Time sync mode (0=SKEW, 1=OFFSET) */
static uint8_t node_sync_mode = 0;

/* Position estimation mode */
static node_position_mode_t node_position_mode = NODE_POSITION_MODE_STATIC;

/* Callback for position change notifications */
static node_position_changed_cb_t position_changed_callback = NULL;

/* Mutex to protect position access */
K_MUTEX_DEFINE(pos_mutex);


static int settings_set(const char *name, size_t len,
                           settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "node_mode", &next) && !next) {
        if (len != sizeof(node_mode)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &node_mode,
                   sizeof(node_mode));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    if (settings_name_steq(name, "position", &next) && !next) {
        if (len != sizeof(node_pos)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &node_pos, sizeof(node_pos));
        if (rc < 0) {
            return rc;
        }
        node_pos_valid = true;
        LOG_INF("Loaded position from settings: (%.3f, %.3f, %.3f)",
                node_pos.x, node_pos.y, node_pos.z);
        return 0;
    }

    if (settings_name_steq(name, "is_root", &next) && !next) {
        if (len != sizeof(node_is_root)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &node_is_root, sizeof(node_is_root));
        if (rc < 0) {
            return rc;
        }
        node_is_root_initialized = true;
        LOG_INF("Loaded is_root from settings: %s", node_is_root ? "true" : "false");
        return 0;
    }

    if (settings_name_steq(name, "free_running", &next) && !next) {
        if (len != sizeof(node_free_running)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &node_free_running, sizeof(node_free_running));
        if (rc < 0) {
            return rc;
        }
        LOG_INF("Loaded free_running from settings: %s", node_free_running ? "true" : "false");
        return 0;
    }

    if (settings_name_steq(name, "sync_mode", &next) && !next) {
        if (len != sizeof(node_sync_mode)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &node_sync_mode, sizeof(node_sync_mode));
        if (rc < 0) {
            return rc;
        }
        LOG_INF("Loaded sync_mode from settings: %u", node_sync_mode);
        return 0;
    }

    if (settings_name_steq(name, "pos_mode", &next) && !next) {
        if (len != sizeof(node_position_mode)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &node_position_mode, sizeof(node_position_mode));
        if (rc < 0) {
            return rc;
        }
        LOG_INF("Loaded position mode from settings: %d", node_position_mode);
        return 0;
    }

    return -ENOENT;
}

/* static int cmd_set_node_id(const struct shell *sh, size_t argc, char **argv) */
/* { */
/*     if (argc != 2) { */
/*         shell_error(sh, "Usage: config node_id <id>"); */
/*         shell_help(sh); */
/*         return -EINVAL; */
/*     } */

/*     int id = atoi(argv[1]); */
/*     if (id < 0 || id > DECA_SHORT_ADDR_MAX) { */
/*         shell_error(sh, "Invalid node ID. Must be between 0 and %d", DECA_SHORT_ADDR_MAX); */
/*         return -EINVAL; */
/*     } */

/*     deca_short_addr_t new_node_id = (deca_short_addr_t)id; */
/*     node_id = new_node_id; */
/*     shell_print(sh, "Node ranging ID set to: %d", new_node_id); */

/*     // Save the setting */
/*     settings_save_one("node/node_id", &new_node_id, sizeof(new_node_id)); */

/*     return 0; */
/* } */


#if SYNCHROFLY_NODE_ID_FROM_SETTINGS
static deca_short_addr_t settings_node_ranging_id = DECA_NO_ADDRESS;

static int node_id_set(const char *name, size_t len,
                           settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "node_id", &next) && !next) {
        if (len != sizeof(settings_node_ranging_id)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings_node_ranging_id,
                   sizeof(settings_node_ranging_id));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    return -ENOENT;
}

static deca_short_addr_t id_from_settings() {
    if (node_settings.node_ranging_id != 0) {
        node_ranging_id = node_settings.node_ranging_id;
        LOG_WRN("Using node ranging ID from settings: %d", node_ranging_id);
    } else {
        /* Otherwise use the default approach */
        int16_t signed_node_id = get_node_number(node_id);
        node_ranging_id = signed_node_id;

        if (signed_node_id < 0) {
            LOG_ERR("Failed to get node ranging ID: %d", signed_node_id);
            // disable node
            set_node_mode(NODE_MODE_DISABLED);
            node_ranging_id = DECA_NO_ADDRESS;
        } else {
            /* Save to settings for future use */
            node_settings.node_ranging_id = node_ranging_id;
            ret = settings_save_one("node/ranging_id", &node_settings.node_ranging_id,
                sizeof(node_settings.node_ranging_id));
            if (ret) {
                LOG_WRN("Failed to save node ranging ID: %d", ret);
            }
        }
    }
}
#endif


/**
 * @brief Derive node ID using the IoT-LAB iotlab_uid() formula.
 *
 * Reads FICR DEVICEID[1] directly and applies the same byte extraction as
 * RIOT's cpuid_get() + IoT-LAB's iotlab_uid() default formula:
 *   (id[CPUID_LEN-4] | (id[CPUID_LEN-2] << 7)) << 8 | id[CPUID_LEN-3]
 *
 * On nRF52 (CPUID_LEN=8), this uses bytes 4,5,6 of the raw FICR DEVICEID,
 * which are the low 3 bytes of DEVICEID[1] in native little-endian order.
 */
static deca_short_addr_t id_from_iotlab_uid(void) {
    uint32_t did1 = nrf_ficr_deviceid_get(NRF_FICR, 1);
    uint8_t b0 = (did1 >> 0) & 0xFF;   /* RIOT cpuid byte [4] */
    uint8_t b1 = (did1 >> 8) & 0xFF;   /* RIOT cpuid byte [5] */
    uint8_t b2 = (did1 >> 16) & 0xFF;  /* RIOT cpuid byte [6] */
    return (deca_short_addr_t)((b0 | (b2 << 7)) << 8 | b1);
}

deca_short_addr_t get_node_addr(void) {
    return id_from_iotlab_uid();
}

node_mode_t get_node_mode() {
    return node_mode;
}

void set_node_mode(node_mode_t mode) {
    node_mode = mode;
}

int get_node_config_position(struct node_config_position *pos)
{
    if (!pos) {
        return -EINVAL;
    }

    k_mutex_lock(&pos_mutex, K_FOREVER);

    if (!node_pos_valid) {
        k_mutex_unlock(&pos_mutex);
        return -ENOENT;
    }

    memcpy(pos, &node_pos, sizeof(struct node_config_position));
    k_mutex_unlock(&pos_mutex);

    return 0;
}

int set_node_config_position(const struct node_config_position *pos)
{
    int ret;

    if (!pos) {
        return -EINVAL;
    }

    k_mutex_lock(&pos_mutex, K_FOREVER);

    memcpy(&node_pos, pos, sizeof(struct node_config_position));
    node_pos_valid = true;

    /* Persist to settings */
    ret = settings_save_one("node/position", &node_pos, sizeof(node_pos));
    if (ret) {
        LOG_ERR("Failed to save position (err %d)", ret);
        k_mutex_unlock(&pos_mutex);
        return ret;
    }

    LOG_INF("Position set to (%.3f, %.3f, %.3f)", pos->x, pos->y, pos->z);

    /* Notify callback (outside mutex to avoid potential deadlocks) */
    node_position_changed_cb_t cb = position_changed_callback;
    k_mutex_unlock(&pos_mutex);

    if (cb) {
        cb((float)pos->x, (float)pos->y, (float)pos->z);
    }

    return 0;
}

void set_node_position_volatile(const struct node_config_position *pos)
{
    if (!pos) {
        return;
    }

    k_mutex_lock(&pos_mutex, K_FOREVER);
    memcpy(&node_pos, pos, sizeof(struct node_config_position));
    node_pos_valid = true;

    /* Notify callback (outside mutex to avoid potential deadlocks) */
    node_position_changed_cb_t cb = position_changed_callback;
    k_mutex_unlock(&pos_mutex);

    if (cb) {
        cb((float)pos->x, (float)pos->y, (float)pos->z);
    }
}

bool get_node_is_root(void)
{
#ifdef CONFIG_SYNCHROFLY_ROOT_NODE_FROM_SETTINGS
    /* In settings mode, default to non-root if NVS has no value.
     * Root must be explicitly set via BLE or shell. */
    return node_is_root;
#else
    // In static mode, always determine root status from CONFIG at runtime
    return (get_node_addr() == CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID);
#endif
}

int set_node_is_root(bool is_root)
{
    int ret;

    node_is_root = is_root;
    node_is_root_initialized = true;  // Mark as initialized to prevent re-init from static config

    /* Persist to settings */
    ret = settings_save_one("node/is_root", &node_is_root, sizeof(node_is_root));
    if (ret) {
        LOG_ERR("Failed to save is_root flag (err %d)", ret);
        return ret;
    }

    LOG_INF("Network root mode %s", is_root ? "enabled" : "disabled");

    return 0;
}

bool get_node_free_running(void)
{
    return node_free_running;
}

int set_node_free_running(bool enable)
{
    int ret;

    node_free_running = enable;

    /* Persist to settings */
    ret = settings_save_one("node/free_running", &node_free_running, sizeof(node_free_running));
    if (ret) {
        LOG_ERR("Failed to save free_running flag (err %d)", ret);
        return ret;
    }

    LOG_INF("Free-running mode %s", enable ? "enabled" : "disabled");

    return 0;
}

uint8_t get_node_sync_mode(void)
{
    return node_sync_mode;
}

int set_node_sync_mode(uint8_t mode)
{
    int ret;

    node_sync_mode = mode;

    ret = settings_save_one("node/sync_mode", &node_sync_mode, sizeof(node_sync_mode));
    if (ret) {
        LOG_ERR("Failed to save sync_mode (err %d)", ret);
        return ret;
    }

    LOG_INF("Sync mode set to %u (%s)", mode, mode == 1 ? "OFFSET" : "SKEW");

    return 0;
}

node_position_mode_t get_node_position_mode(void)
{
    return node_position_mode;
}

int set_node_position_mode(node_position_mode_t mode)
{
    int ret;

    if (mode > NODE_POSITION_MODE_BELIEF_PROPAGATION) {
        return -EINVAL;
    }

    node_position_mode = mode;

    /* Persist to settings */
    ret = settings_save_one("node/pos_mode", &node_position_mode, sizeof(node_position_mode));
    if (ret) {
        LOG_ERR("Failed to save position mode (err %d)", ret);
        return ret;
    }

    LOG_INF("Position mode set to %d", mode);

    return 0;
}

void node_print_settings(void)
{
    const char *mode_str;
    switch (node_mode) {
        case NODE_MODE_ACTIVE:
            mode_str = "active";
            break;
        case NODE_MODE_DISABLED:
            mode_str = "disabled";
            break;
        default:
            mode_str = "unknown";
            break;
    }

    const char *pos_mode_str;
    switch (node_position_mode) {
        case NODE_POSITION_MODE_STATIC:
            pos_mode_str = "static (anchor)";
            break;
        case NODE_POSITION_MODE_LEAST_SQUARES:
            pos_mode_str = "least_squares (mobile)";
            break;
        case NODE_POSITION_MODE_BELIEF_PROPAGATION:
            pos_mode_str = "belief_propagation (reserved)";
            break;
        default:
            pos_mode_str = "unknown";
            break;
    }

    const char *root_source;
#ifdef CONFIG_SYNCHROFLY_ROOT_NODE_FROM_SETTINGS
    root_source = "settings";
#else
    root_source = "static_config";
#endif

    bool is_root = get_node_is_root();

    LOG_WRN("Node Configuration:");
    LOG_WRN("  Node Address: 0x%04hx", get_node_addr());
    LOG_WRN("  Mode: %s", mode_str);
    LOG_WRN("  Position Mode: %s", pos_mode_str);
    LOG_WRN("  Is Root: %s", is_root ? "yes" : "no");
    LOG_WRN("  Root Source: %s", root_source);
    LOG_WRN("  Free Running: %s", node_free_running ? "yes" : "no");

    k_mutex_lock(&pos_mutex, K_FOREVER);
    if (node_pos_valid) {
        LOG_WRN("  Position: x=%.3f y=%.3f z=%.3f",
                (double)node_pos.x,
                (double)node_pos.y,
                (double)node_pos.z);
    } else {
        LOG_WRN("  Position: not configured");
    }
    k_mutex_unlock(&pos_mutex);
}


void node_register_position_callback(node_position_changed_cb_t cb)
{
    k_mutex_lock(&pos_mutex, K_FOREVER);
    position_changed_callback = cb;
    k_mutex_unlock(&pos_mutex);
}

/* settings */
SETTINGS_STATIC_HANDLER_DEFINE(node, "node", NULL, settings_set, NULL, NULL);

#if SYNCHROFLY_NODE_ID_FROM_SETTINGS
SETTINGS_STATIC_HANDLER_DEFINE(node, "node_id", NULL, node_id_set, NULL, NULL);
#endif
