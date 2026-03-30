/*
 * Copyright (c) 2025 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SYNCHROFLY_STATUS_H
#define SYNCHROFLY_STATUS_H

#include <stdint.h>

/**
 * Abstract SynchroFly system status states.
 *
 * These represent the operational state of the UWB MAC and time synchronization
 * subsystem. Status indicators (LED, audio, display, etc.) can implement
 * visual/audible representations of these abstract states.
 */
enum synchrofly_status {
    /** Node is completely disabled */
    STATUS_DISABLED = 0,

    /** Node is warming up / attempting to join network (not yet synchronized) */
    STATUS_WARMING_UP = 1,

    /** Synchronized with network, passive mode (TDOA positioning only) */
    STATUS_SYNCED_PASSIVE = 2,

    /** Synchronized with network, active mode (full TWR ranging) */
    STATUS_SYNCED_ACTIVE = 3,

    /** Node is operating as network timing root (Glossy flood initiator) */
    STATUS_ROOT_MODE = 4,

    /** Degraded operation (time sync only, ranging disabled/failed) */
    STATUS_DEGRADED = 5,
};

/**
 * Get the current SynchroFly system status.
 *
 * @return Current system status
 */
enum synchrofly_status synchrofly_status_get(void);

/**
 * Manually set the SynchroFly system status.
 *
 * This is typically not needed as status is determined automatically
 * by querying system state. Use this for testing or manual overrides.
 *
 * @param status Status to set
 */
void synchrofly_status_set(enum synchrofly_status status);

/**
 * Trigger device identification LED pattern.
 *
 * Causes the status LED to blink rapidly for the specified duration,
 * overriding the normal status-driven pattern. After the duration expires,
 * the normal status pattern resumes automatically.
 *
 * @param duration_ms Duration of identification blink in milliseconds
 */
void synchrofly_status_identify(uint32_t duration_ms);

#endif /* SYNCHROFLY_STATUS_H */
