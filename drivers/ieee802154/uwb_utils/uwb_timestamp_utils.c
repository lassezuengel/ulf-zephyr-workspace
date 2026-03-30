/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uwb_timestamp_utils.h"

uint64_t correct_overflow(uwb_ts_t end_ts, uwb_ts_t start_ts)
{
    if (end_ts < start_ts) {
        end_ts += 0xFFFFFFFFFF;
    }

    return end_ts - start_ts;
}

uwb_ts_t from_packed_uwb_ts(const uwb_packed_ts_t ts) 
{
    return (uwb_ts_t) ts[0] | 
           ((uwb_ts_t) ts[1] << 8) | 
           ((uwb_ts_t) ts[2] << 16) | 
           ((uwb_ts_t) ts[3] << 24) | 
           ((uwb_ts_t) ts[4] << 32);
}

void to_packed_uwb_ts(uwb_packed_ts_t ts, uwb_ts_t value) 
{
    ts[0] = value & 0xFF;
    ts[1] = (value >> 8) & 0xFF;
    ts[2] = (value >> 16) & 0xFF;
    ts[3] = (value >> 24) & 0xFF;
    ts[4] = (value >> 32) & 0xFF;
}