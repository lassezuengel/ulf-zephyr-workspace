#include <zephyr/settings/settings.h>

#include <app/lib/system/hw.h>
#include <app/lib/system/node.h>
#include <app/lib/timesync/time_synchronization.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/lib/blocks/glossy.h>
#include <app/lib/blocks/blocks.h>
#include <app/lib/node_table/node_table.h>

#define TESTING_RANDOM_FAIL_GLOSSY 0
#define TESTING_RANDOM_FAIL_GLOSSY_PROBABILITY 0.33

/* Internal Configuration settings struct */
struct glossy_settings {
    bool with_status_message;
};

static struct glossy_settings settings = {
    .with_status_message = true
};

static int settings_set(const char *name, size_t len,
                             settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "status_msg", &next) && !next) {
        if (len != sizeof(settings.with_status_message)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &settings.with_status_message,
                   sizeof(settings.with_status_message));
        if (rc < 0) {
            return rc;
        }
        return 0;
    }

    return -ENOENT;
}

// will see how we do this.. probably in network_settings using two sets of settings, one which is setup and one which is applied right now
/* void capture_current_network_settings(struct network_config_information *config) { */
/*     if (!config) { */
/*         return; */
/*     } */

/*     config->scheduler_slot_duration_ms = network_scheduler_settings.scheduler_slot_duration_ms; */
/*     config->schedule_type = ranging_settings.schedule_type; */
/*     config->ranging_round_slots_per_phase = ranging_settings.ranging_round_slots_per_phase; */
/*     config->ranging_round_phases = ranging_settings.ranging_round_phases; */
/*     config->with_measurement_exchange_round = ranging_settings.with_measurement_exchange_round; */
/*     config->rangings_per_glossy = glossy_settings.rangings_per_glossy; */
/*     config->ranging_worker_guards_us = ranging_settings.ranging_worker_guards_us; */
/* } */


// Function to apply network configuration from glossy payload
/* void commit_network_config(const struct network_config_information *config) { */
/*     if (!config) { */
/*         return; */
/*     } */

/*     // Update local settings from network configuration */
/*     LOG_WRN("Applying network config: slot_dur=%u ms, sched=%d, ranging_round_slots=%u, ranging_round_phases=%u, msg_exchange=%d\n", */
/*             config->scheduler_slot_duration_ms, */
/*         config->schedule_type, */
/*         config->ranging_round_slots_per_phase, */
/*         config->ranging_round_phases, */
/*         config->with_measurement_exchange_round); */

/*     // prepare network configuration to share */
/*     network_scheduler_settings.scheduler_slot_duration_ms = config->scheduler_slot_duration_ms; */
/*     ranging_settings.schedule_type = config->schedule_type; */
/*     ranging_settings.ranging_round_slots_per_phase = config->ranging_round_slots_per_phase; */
/*     ranging_settings.ranging_round_phases = config->ranging_round_phases; */
/*     ranging_settings.with_measurement_exchange_round = config->with_measurement_exchange_round; */
/*     ranging_settings.ranging_worker_guards_us = config->ranging_worker_guards_us; */
/*     glossy_settings.rangings_per_glossy = config->rangings_per_glossy; */
/* } */

void glossy_set_status_message(bool enabled) {
    settings.with_status_message = enabled;

    // Save the setting
    settings_save_one("glossy/status_msg", &settings.with_status_message,
                    sizeof(settings.with_status_message));
}

void glossy_block_handler(uint64_t event_time, void *user_data)
{
    struct deca_glossy_result sync_result;
    /* static uint8_t glossy_payload[sizeof(struct network_config_information)]; */

    struct glossy_block_config *config = (struct glossy_block_config*) user_data;

    // Ensure radio is on the configured channel for Glossy if provided
    if (config && config->channel) {
        const uwb_driver_t *uwb = uwb_driver_get(ieee802154_dev);
        if (uwb && uwb->set_channel) {
            uwb->set_channel(ieee802154_dev, config->channel);
        }
    }

    struct ieee802154_radio_api *__attribute__((unused)) radio_api = (struct ieee802154_radio_api *) ieee802154_dev->api;

    // Initialize payload if this node is the root
    /* if (network_config_commit_mode == COMMIT_GLOSSY && */
    /*     node_id == CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID) { */
    /*     // Copy to payload buffer */
    /*     memcpy(glossy_payload, &net_config, sizeof(net_config)); */
    /* } */

    struct deca_glossy_configuration deca_glossy_conf = {
        .node_addr = get_node_addr(),
        .isRoot = get_node_is_root(),  // Use function to respect both static and settings-based config
        .guard_period_us = config->guard_period_us,
        .max_depth = config->max_depth,
        /* .payload_size = node_id == CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID ? sizeof(net_config) : 0, */
        /* .payload = node_id == CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID ? glossy_payload : NULL, */
        .transmission_delay_us = config->transmission_delay_us,
    };

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif
    int ret = deca_glossy_time_synchronization(ieee802154_dev, &deca_glossy_conf, &sync_result);
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    // normally glossy will not fail, but we still would like to test for this condition
#if TESTING_RANDOM_FAIL_GLOSSY
    if (node_id != CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID &&
        (sys_rand32_get() % 100) < (100*TESTING_RANDOM_FAIL_GLOSSY_PROBABILITY)) {
        ret = -1;
    }
#endif
    if (ret >= 0) {
        time_sync_update(event_time, sync_result);

        // Update node table with hop count to root (root_node_id extracted from glossy frame)
        if (!get_node_is_root() && sync_result.dist_to_root > 0) {
            node_table_update_hop_count(sync_result.root_node_id, sync_result.dist_to_root, event_time);
            node_table_set_flags(sync_result.root_node_id, NODE_TABLE_FLAG_ROOT);
            node_table_notify_changed();
        }

        // If not root and we received a config payload, apply it
        /* if (network_config_commit_mode == COMMIT_GLOSSY) { */
        /*     if(sync_result.payload_size == sizeof(struct network_config_information) && sync_result.payload != NULL) { */
        /*         commit_network_config((const struct network_config_information *)sync_result.payload); */
        /*     } */
        /* } */

        /* time_sync_set_slot_duration(network_scheduler_settings.scheduler_slot_duration_ms); */
    }

    if (settings.with_status_message) {
        if (ret >= 0) {
            if (sync_result.measured_constant_delay_us >= 0) {
                printk("{\"event\": \"glossy\", \"node_id\": \"0x%04hx\", \"hops\": %d, \"rtc\": ""%lld, \"payload_size\": %d, \"measured_delay_us\": %d}\n",
                    get_node_addr(), sync_result.dist_to_root, event_time, sync_result.payload_size, sync_result.measured_constant_delay_us);
            } else {
                printk("{\"event\": \"glossy\", \"node_id\": \"0x%04hx\", \"hops\": %d, \"rtc\": ""%lld, \"payload_size\": %d, \"measured_delay_us\": \"no\"}\n",
                    get_node_addr(), sync_result.dist_to_root, event_time, sync_result.payload_size);
            }
        } else {
            const char *error_name = "UNKNOWN";
            switch (-ret) {
                case 5:   error_name = "EIO"; break;
                case 16:  error_name = "EBUSY"; break;
                case 19:  error_name = "ENODEV"; break;
                case 22:  error_name = "EINVAL"; break;
                case 116: error_name = "ETIMEDOUT"; break;  // Zephyr minimal libc uses 116
                default:  error_name = "UNKNOWN"; break;
            }
            printk("{\"event\": \"glossy\", \"node_id\": \"0x%04hx\", \"error\": %d, \"error_name\": \"%s\", \"rtc\": "
                   "%lld}\n",
                get_node_addr(), ret, error_name, event_time);
        }
    }
}


// contains configuration values which ought to be shared between nodes

// TODO
/* struct network_config_information { */
/*     uint32_t scheduler_slot_duration_ms; */
/*     // for now we will globally only use one schedule type */
/*     schedule_type_t schedule_type; */
/*     uint8_t ranging_round_slots_per_phase; */
/*     uint8_t ranging_round_phases; */
/*     uint16_t ranging_worker_guards_us; */
/*     bool with_measurement_exchange_round; // TODO */

/*     // glossy */
/*     uint16_t rangings_per_glossy; */
/* } __attribute__((packed)); */

/* Register settings handlers */
SETTINGS_STATIC_HANDLER_DEFINE(glossy, "glossy", NULL, settings_set, NULL, NULL);
