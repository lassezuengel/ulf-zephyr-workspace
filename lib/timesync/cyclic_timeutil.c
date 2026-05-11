/*
 * Copyright (c) 2023 Your Organization
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>

#include "cyclic_timeutil.h"

LOG_MODULE_REGISTER(cyclic_timeutil, CONFIG_CYCLIC_TIMEUTIL_LOG_LEVEL);

#define WITH_CORRECT_SKEW 1

/**
 * Handle wrap around when calculating delta between two timestamps
 *
 * @param newer the newer timestamp
 * @param older the older timestamp
 * @param wrap_value the value at which the clock wraps around
 *
 * @return the time difference accounting for possible wrap around
 */
static int64_t cyclic_time_delta(uint64_t newer, uint64_t older, uint64_t wrap_value)
{
    if (newer >= older || wrap_value == 0) {
        return newer - older;
    } else {
        // Handle wrap around
        return (wrap_value - older) + newer;
    }
}

int cyclic_timeutil_sync_state_update(struct cyclic_timeutil_sync_state *tsp,
                                     const struct cyclic_timeutil_sync_instant *inst)
{
    int rv = -EINVAL;

    // Check if this is the first update or if timestamps are newer
    if ((tsp->base.ref == 0) ||
        (cyclic_time_delta(inst->ref, tsp->base.ref, tsp->cfg->ref_wrap) > 0 &&
         cyclic_time_delta(inst->local, tsp->base.local, tsp->cfg->local_wrap) > 0)) {

        /* printk("Updating base: ref %llu, local %llu\n", inst->ref, inst->local); */

        if (tsp->base.ref == 0) {
            tsp->base = *inst;
            tsp->latest = (struct cyclic_timeutil_sync_instant){};
            tsp->skew = 1.0f;
            rv = 0;
        } else {
            tsp->latest = *inst;
            rv = 1;
        }
    }

    return rv;
}

int cyclic_timeutil_sync_state_set_skew(struct cyclic_timeutil_sync_state *tsp, double skew,
                                       const struct cyclic_timeutil_sync_instant *base)
{
    int rv = -EINVAL;

    if (skew > 0) {
        tsp->skew = skew;
        if (base != NULL) {
            tsp->base = *base;
            tsp->latest = (struct cyclic_timeutil_sync_instant){};
        }
        rv = 0;
    }

    return rv;
}

double cyclic_timeutil_sync_estimate_skew(const struct cyclic_timeutil_sync_state *tsp)
{
    double rv = 0;

    if ((tsp->base.ref != 0) && (tsp->latest.ref != 0)) {
        const struct cyclic_timeutil_sync_config *cfg = tsp->cfg;
        int64_t ref_delta = cyclic_time_delta(tsp->latest.ref, tsp->base.ref, cfg->ref_wrap);
        int64_t local_delta = cyclic_time_delta(tsp->latest.local, tsp->base.local, cfg->local_wrap);

        LOG_DBG("Skew calculation - ref_delta: %lld, local_delta: %lld", ref_delta, local_delta);
        LOG_DBG("Frequency config - ref_Hz: %llu, local_Hz: %llu", cfg->ref_Hz, cfg->local_Hz);

        if (local_delta > 0) {
            if(cfg->ref_Hz != cfg->local_Hz) {
                // Log the intermediate calculation that could overflow
                int64_t ref_delta_scaled = ref_delta * cfg->local_Hz;
                LOG_DBG("Intermediate calc - ref_delta * local_Hz = %lld", ref_delta_scaled);
                
                rv = (double)(ref_delta * cfg->local_Hz) / (double) local_delta / (double) cfg->ref_Hz;
                
                LOG_DBG("Final skew result: %f (%.0f ppm)", rv, (rv - 1.0) * 1000000.0);
            } else {
                rv = (double)ref_delta / (double)local_delta;
                LOG_DBG("Same frequency skew: %f", rv);
            }
        } else {
            LOG_WRN("Invalid local_delta: %lld (must be > 0)", local_delta);
        }
    } else {
        LOG_DBG("Skipping skew calculation - base.ref: %llu, latest.ref: %llu", 
                tsp->base.ref, tsp->latest.ref);
    }

    return rv;
}

int cyclic_timeutil_sync_ref_from_local(const struct cyclic_timeutil_sync_state *tsp,
                                       uint64_t local, uint64_t *refp)
{
    int rv = -EINVAL;

    if ((tsp->skew > 0) && (tsp->base.ref != 0) && (refp != NULL)) {
        const struct cyclic_timeutil_sync_config *cfg = tsp->cfg;
        int64_t local_delta = cyclic_time_delta(local, tsp->base.local, cfg->local_wrap);
        /* printk("local_delta: %lld\n", local_delta); */

#if WITH_CORRECT_SKEW
        if (tsp->skew != 1.0) {
            local_delta *= (double)tsp->skew;
        }
#endif

        /* printk("local_delta skew corrected: %lld\n", local_delta); */

        if(cfg->ref_Hz != cfg->local_Hz) {
            int64_t ref_delta = (local_delta * cfg->ref_Hz) / cfg->local_Hz;
            uint64_t ref_abs = (int64_t)tsp->base.ref + ref_delta;
            /* printk("ref_abs: %llu\n", ref_abs); */
            if(cfg->ref_wrap != 0) {
                *refp = ref_abs % cfg->ref_wrap;
            } else {
                *refp = ref_abs;
            }
        } else {
            *refp = (tsp->base.ref + local_delta) % cfg->ref_wrap;
        }

        // Handle wrap around in the reference clock
        rv = (int)(tsp->skew != 1.0);
    }

    return rv;
}

int cyclic_timeutil_sync_local_from_ref(const struct cyclic_timeutil_sync_state *tsp,
                                       uint64_t ref, uint64_t *localp)
{
    int rv = -EINVAL;

    if ((tsp->skew > 0) && (tsp->base.ref != 0) && (localp != NULL)) {
        const struct cyclic_timeutil_sync_config *cfg = tsp->cfg;
        int64_t ref_delta = cyclic_time_delta(ref, tsp->base.ref, cfg->ref_wrap);
        int64_t local_delta;

        if(cfg->ref_Hz != cfg->local_Hz) {
            local_delta = (ref_delta * cfg->local_Hz) / cfg->ref_Hz;
        } else {
            local_delta = ref_delta;
        }

#if WITH_CORRECT_SKEW
        /* printk("local_delta: %lld\n", local_delta); */
        if (tsp->skew != 1.0) {
            local_delta /= (double)tsp->skew;
        }
        /* printk("local_delta skew corrected: %lld with skew %f\n", local_delta, tsp->skew); */
#endif

        uint64_t local_abs = (int64_t) tsp->base.local + (int64_t) local_delta;

        // Handle wrap around in the local clock
        if(cfg->local_wrap != 0) {
            *localp = local_abs % cfg->local_wrap;
        } else {
            *localp = local_abs;
        }

        rv = (int)(tsp->skew != 1.0);
    }

    return rv;
}

int32_t cyclic_timeutil_sync_skew_to_ppb(double skew)
{
    int64_t ppb64 = (int64_t)((1.0 - (double)skew) * 1E9);
    int32_t ppb32 = (int32_t)ppb64;

    return (ppb64 == ppb32) ? ppb32 : INT32_MIN;
}
