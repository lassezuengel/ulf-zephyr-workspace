/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UWB_TIMESTAMP_UTILS_H
#define UWB_TIMESTAMP_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UWB timestamp types - driver-agnostic */
typedef uint64_t uwb_ts_t;
typedef uint8_t uwb_packed_ts_t[5];

/* UWB timestamp constants */
#define UWB_TS_MASK (0xFFFFFFFFFF)

/* UWB frame IDs */
#define UWB_MTM_RANGING_FRAME_ID 0x01
#define UWB_MTM_GLOSSY_TX_ID 0x02

/* UWB timing configuration structure */
struct uwb_ranging_timing {
	uint32_t min_slot_length_us;
	uint32_t phy_activate_rx_delay_us;
	uint32_t phase_setup_delay_us;
	uint32_t round_setup_delay_us;
	uint32_t preamble_chunk_duration_us;
	uint16_t frame_timeout_period;
	uint16_t preamble_timeout;
};

/**
 * @brief Correct timestamp overflow for duration calculations
 *
 * @param end_ts End timestamp
 * @param start_ts Start timestamp
 * @return Corrected duration between timestamps
 */
uint64_t correct_overflow(uwb_ts_t end_ts, uwb_ts_t start_ts);

/**
 * @brief Convert packed timestamp to 64-bit timestamp
 *
 * @param ts Packed timestamp (5 bytes)
 * @return 64-bit timestamp value
 */
uwb_ts_t from_packed_uwb_ts(const uwb_packed_ts_t ts);

/**
 * @brief Convert 64-bit timestamp to packed timestamp
 *
 * @param ts Packed timestamp buffer (5 bytes)
 * @param value 64-bit timestamp value
 */
void to_packed_uwb_ts(uwb_packed_ts_t ts, uwb_ts_t value);

/* Legacy compatibility - map old DWT types to new UWB types */
typedef uwb_ts_t dwt_ts_t;
typedef uwb_packed_ts_t dwt_packed_ts_t;

#define from_packed_dwt_ts from_packed_uwb_ts
#define to_packed_dwt_ts to_packed_uwb_ts

/* Legacy constants */
#define DWT_TS_MASK UWB_TS_MASK
#define DWT_MTM_RANGIN_FRAME_ID UWB_MTM_RANGING_FRAME_ID
#define DWT_MTM_GLOSSY_TX_ID UWB_MTM_GLOSSY_TX_ID

#ifdef __cplusplus
}
#endif

#endif /* UWB_TIMESTAMP_UTILS_H */
