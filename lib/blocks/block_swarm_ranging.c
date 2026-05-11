/*
 * Copyright (c) 2025 SynchroFly
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/lib/system/hw.h>
#include <app/lib/blocks/swarm_ranging.h>
#include <app/lib/system/radio_arbiter.h>
#include <app/lib/swarm_ranging/swarm_ranging_core.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>

LOG_MODULE_REGISTER(block_swarm_ranging, CONFIG_LOG_DEFAULT_LEVEL);

void swarm_ranging_block_handler(uint64_t rtc_event_time, void *user_data)
{
	struct swarm_ranging_block_config *config = (struct swarm_ranging_block_config *)user_data;

	/* Optionally restore channel for background protocol.
	 * Must acquire device lock to avoid SPI contention with the
	 * swarm UWB RX/TX threads. */
	if (config && config->channel) {
		const uwb_driver_t *uwb = uwb_driver_get(ieee802154_dev);
		if (uwb && uwb->set_channel) {
			uwb->acquire_device(ieee802154_dev);
			uwb->set_channel(ieee802154_dev, config->channel);
			uwb->release_device(ieee802154_dev);
		}
	}

	/* Apply ranging period if configured */
	if (config && config->period_ms > 0) {
		swarm_ranging_set_period(config->period_ms);
	}

	/* Apply distance filter and bus boarding settings */
	if (config) {
		swarm_ranging_set_distance_filter(config->filter_enabled);
		swarm_ranging_set_bus_boarding(config->bus_boarding_enabled);
	}

	/* Release radio for background protocol */
	radio_arbiter_release();
}
