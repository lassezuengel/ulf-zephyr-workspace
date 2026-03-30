/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UWB_FRAME_UTILS_H
#define UWB_FRAME_UTILS_H

#include <stdint.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get tagged timestamps from a ranging frame
 *
 * @param frame Pointer to the ranging frame
 * @param timestamps Output pointer to timestamp array
 * @return Number of timestamps, or -1 on error
 */
int deca_ranging_frame_get_tagged_timestamps(const struct deca_ranging_frame *frame,
                                           struct deca_tagged_timestamp **timestamps);

/**
 * @brief Get MM tagged timestamps from a ranging frame
 *
 * @param frame Pointer to the ranging frame
 * @param timestamps Output pointer to MM timestamp array
 * @return Number of timestamps, or -1 on error
 */
int deca_ranging_frame_get_mm_tagged_timestamps(const struct deca_ranging_frame *frame,
                                              struct deca_tagged_mm_timestamp **timestamps);

#ifdef __cplusplus
}
#endif

#endif /* UWB_FRAME_UTILS_H */
