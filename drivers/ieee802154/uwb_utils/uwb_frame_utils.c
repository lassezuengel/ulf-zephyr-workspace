/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/drivers/ieee802154/uwb_frame_utils.h>

int deca_ranging_frame_get_tagged_timestamps(const struct deca_ranging_frame *frame,
                                           struct deca_tagged_timestamp **timestamps)
{
    if (!frame || !timestamps) {
        return -1;
    }

    *timestamps = (struct deca_tagged_timestamp *)(&frame->payload[frame->payload_size]);
    return frame->rx_ts_count;
}

int deca_ranging_frame_get_mm_tagged_timestamps(const struct deca_ranging_frame *frame,
                                              struct deca_tagged_mm_timestamp **timestamps)
{
    if (!frame || !timestamps) {
        return -1;
    }

    *timestamps = (struct deca_tagged_mm_timestamp *)(&frame->payload[frame->payload_size]);
    return frame->rx_ts_count;
}
