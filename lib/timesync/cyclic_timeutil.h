#ifndef CYCLIC_TIMEUTIL_H
#define CYCLIC_TIMEUTIL_H

#include <stdint.h>

struct cyclic_timeutil_sync_config {
    /** Frequency of the reference clock in Hz */
    uint64_t ref_Hz;
    /** Frequency of the local clock in Hz */
    uint64_t local_Hz;
    /** Value at which the reference clock wraps around */
    uint64_t ref_wrap;
    /** Value at which the local clock wraps around */
    uint64_t local_wrap;
};

struct cyclic_timeutil_sync_instant {
    /** Reference time when state captured */
    uint64_t ref;
    /** Local time when state captured */
    uint64_t local;
};

struct cyclic_timeutil_sync_state {
    /** Pointer to configuration data */
    const struct cyclic_timeutil_sync_config *cfg;
    /** Base point for synchronization */
    struct cyclic_timeutil_sync_instant base;
    /** Latest point for synchronization */
    struct cyclic_timeutil_sync_instant latest;
    /** Estimated skew between clocks (reference/local) */
    double skew;
};

int cyclic_timeutil_sync_state_update(struct cyclic_timeutil_sync_state *tsp,
                                     const struct cyclic_timeutil_sync_instant *inst);

int cyclic_timeutil_sync_state_set_skew(struct cyclic_timeutil_sync_state *tsp,
                                       double skew,
                                       const struct cyclic_timeutil_sync_instant *base);

double cyclic_timeutil_sync_estimate_skew(const struct cyclic_timeutil_sync_state *tsp);

int cyclic_timeutil_sync_ref_from_local(const struct cyclic_timeutil_sync_state *tsp,
                                       uint64_t local, uint64_t *refp);

int cyclic_timeutil_sync_local_from_ref(const struct cyclic_timeutil_sync_state *tsp,
                                       uint64_t ref, uint64_t *localp);

int32_t cyclic_timeutil_sync_skew_to_ppb(double skew);

#endif /* CYCLIC_TIMEUTIL_H */
