#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <app/lib/system/hw.h>
#include <app/lib/system/node.h>
#include <app/lib/blocks/blocks.h>
#include <app/lib/blocks/announcement.h>
#include <app/lib/node_table/node_table.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>

LOG_MODULE_REGISTER(block_announcement, CONFIG_LOG_DEFAULT_LEVEL);

void announcement_block_handler(uint64_t event_time, void *user_data)
{
	struct announcement_block_config *config = (struct announcement_block_config *)user_data;
	if (!config) {
		LOG_ERR("Announcement block called with NULL config");
		return;
	}

	/* Decide probabilistically whether this node initiates the announcement */
	bool is_initiator = false;
	if (config->announce_probability_pct >= 100) {
		is_initiator = true;
	} else if (config->announce_probability_pct > 0) {
		uint32_t roll = sys_rand32_get() % 100;
		is_initiator = (roll < config->announce_probability_pct);
	}

	/* Set channel if configured */
	if (config->channel) {
		const uwb_driver_t *uwb = uwb_driver_get(ieee802154_dev);
		if (uwb && uwb->set_channel) {
			uwb->set_channel(ieee802154_dev, config->channel);
		}
	}

	/* Build announcement payload if we are initiating */
	struct announcement_payload ann_payload;
	uint8_t *payload_ptr = NULL;
	size_t payload_size = 0;

	if (is_initiator) {
		ann_payload.node_addr = get_node_addr();
		ann_payload.mode = (uint8_t)get_node_mode();
		ann_payload.position_mode = (uint8_t)get_node_position_mode();
		ann_payload.flags = 0;
		if (get_node_is_root()) {
			ann_payload.flags |= NODE_TABLE_FLAG_ROOT;
		}
		if (node_is_anchor()) {
			ann_payload.flags |= NODE_TABLE_FLAG_ANCHOR;
		}

		/* Get configured position */
		struct node_config_position pos;
		if (get_node_config_position(&pos) == 0) {
			ann_payload.pos_x = (float)pos.x;
			ann_payload.pos_y = (float)pos.y;
			ann_payload.pos_z = (float)pos.z;
		} else {
			ann_payload.pos_x = 0.0f;
			ann_payload.pos_y = 0.0f;
			ann_payload.pos_z = 0.0f;
		}

		payload_ptr = (uint8_t *)&ann_payload;
		payload_size = sizeof(ann_payload);
	}

	struct uwb_flood_config flood_conf = {
		.node_addr = get_node_addr(),
		.is_initiator = is_initiator,
		.guard_period_us = config->guard_period_us,
		.max_depth = config->max_depth,
		.transmission_delay_us = config->transmission_delay_us,
		.payload = payload_ptr,
		.payload_size = payload_size,
		.frame_id = UWB_ANNOUNCEMENT_FRAME_ID,
	};

	struct uwb_flood_result flood_result;

#if CONFIG_RANGING_RADIO_SLEEP
	struct ieee802154_radio_api *radio_api = (struct ieee802154_radio_api *)ieee802154_dev->api;
	radio_api->start(ieee802154_dev);
#endif

	int ret = uwb_glossy_flood(ieee802154_dev, &flood_conf, &flood_result);

#if CONFIG_RANGING_RADIO_SLEEP
	radio_api->stop(ieee802154_dev);
#endif

	if (ret >= 0 && flood_result.payload_size == sizeof(struct announcement_payload) &&
	    flood_result.payload != NULL) {
		/* Parse received announcement */
		const struct announcement_payload *rx_ann =
			(const struct announcement_payload *)flood_result.payload;

		/* Update node table with announced node's info */
		uint16_t announced_addr = rx_ann->node_addr;

		node_table_update_hop_count(announced_addr, flood_result.hop_count, event_time);

		if (rx_ann->flags & NODE_TABLE_FLAG_ROOT) {
			node_table_set_flags(announced_addr, NODE_TABLE_FLAG_ROOT);
		}
		if (rx_ann->flags & NODE_TABLE_FLAG_ANCHOR) {
			node_table_update_position(announced_addr,
				rx_ann->pos_x, rx_ann->pos_y, rx_ann->pos_z, event_time);
		}
		node_table_notify_changed();

		printk("{\"event\": \"announcement\", \"node_id\": \"0x%04hx\", "
		       "\"announced\": \"0x%04hx\", \"hops\": %d, "
		       "\"pos\": [%.3f, %.3f, %.3f], \"mode\": %d, \"flags\": %d, "
		       "\"initiator\": %s, \"rtc\": %lld}\n",
			get_node_addr(), announced_addr, flood_result.hop_count,
			(double)rx_ann->pos_x, (double)rx_ann->pos_y, (double)rx_ann->pos_z,
			rx_ann->mode, rx_ann->flags,
			is_initiator ? "true" : "false", event_time);
	} else if (ret >= 0 && is_initiator) {
		/* Initiator with no received payload (we sent it) */
		printk("{\"event\": \"announcement\", \"node_id\": \"0x%04hx\", "
		       "\"announced\": \"0x%04hx\", \"hops\": 0, "
		       "\"initiator\": true, \"rtc\": %lld}\n",
			get_node_addr(), get_node_addr(), event_time);
	} else if (ret == -ETIMEDOUT) {
		LOG_DBG("Announcement: no flood received (timeout)");
	} else if (ret < 0) {
		LOG_WRN("Announcement flood failed: %d", ret);
	}
}
