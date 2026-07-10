/*
 * Copyright (c) 2025 SynchroFly
 * SPDX-License-Identifier: Apache-2.0
 *
 * Radio Arbiter -- manages radio ownership between superframe blocks
 * and asynchronous background protocols.
 *
 * Superframe blocks implicitly claim the radio when they run.
 * A "baseline" block explicitly releases it for background use.
 * Background protocols check availability before radio access.
 *
 * When CONFIG_SYNCHROFLY_RADIO_ARBITER is disabled, all functions
 * are no-ops and is_available() always returns true.
 */

#ifndef SYNCHROFLY_RADIO_ARBITER_H
#define SYNCHROFLY_RADIO_ARBITER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#if IS_ENABLED(CONFIG_SYNCHROFLY_RADIO_ARBITER)

/** Initialize the radio arbiter. Radio starts as claimed (unavailable). */
int radio_arbiter_init(const struct device *uwb_dev);

/** Release radio for background protocol use. Called by baseline block. */
void radio_arbiter_release(void);

/** Claim radio for superframe use. Cancels any pending background wait. */
void radio_arbiter_claim(void);

/** Check if radio is available for background use. Non-blocking. */
bool radio_arbiter_is_available(void);

#else /* !CONFIG_SYNCHROFLY_RADIO_ARBITER */

static inline int radio_arbiter_init(const struct device *uwb_dev)
{
	ARG_UNUSED(uwb_dev);
	return 0;
}

static inline void radio_arbiter_release(void) {}
static inline void radio_arbiter_claim(void) {}

static inline bool radio_arbiter_is_available(void)
{
	return true;
}

#endif /* CONFIG_SYNCHROFLY_RADIO_ARBITER */

#ifdef __cplusplus
}
#endif

#endif /* SYNCHROFLY_RADIO_ARBITER_H */
