/*
 * Copyright (c) 2025 SynchroFly
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SYNCHROFLY_BLOCK_SWARM_RANGING_H
#define SYNCHROFLY_BLOCK_SWARM_RANGING_H

#include <stdint.h>

/**
 * @brief Runtime configuration for the swarm ranging block
 *
 * The swarm ranging block releases the radio for asynchronous
 * swarm ranging protocol use during its time slot.
 * It returns immediately -- the scheduler handles slot timing.
 */
struct swarm_ranging_block_config {
	uint8_t channel;           /**< UWB channel for background protocol (0 = keep current) */
	uint16_t period_ms;        /**< Sending interval in ms (0 = don't change) */
	bool filter_enabled;       /**< Distance outlier filter on/off */
	bool bus_boarding_enabled; /**< Bus boarding scheduling scheme on/off */
};

/**
 * @brief Swarm ranging block handler
 *
 * Optionally restores UWB channel, then releases the radio via the
 * radio arbiter. Returns immediately.
 */
void swarm_ranging_block_handler(uint64_t rtc_event_time, void *user_data);

#endif /* SYNCHROFLY_BLOCK_SWARM_RANGING_H */
