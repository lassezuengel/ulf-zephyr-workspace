#ifndef TS_SERIES_IDS_H
#define TS_SERIES_IDS_H

/**
 * @file ts_series_ids.h
 * @brief Well-known series_id values for ts_series_key_t.series_id
 *
 * Each ID range is 256 wide to allow per-channel offsets (+ channel number).
 */

#define TS_ID_TWR_TOF              0x0100
#define TS_ID_PHASE_BASE           0x0200  /* + channel number */
#define TS_ID_PHASE_UNWRAPPED_BASE 0x0300  /* + channel number */
#define TS_ID_D_PHASE_DIFF         0x0400
#define TS_ID_RESIDUAL_PHASE       0x0500  /* + channel number */
#define TS_ID_RTC_BASE             0x0600  /* + channel number (timestamp companion) */

#endif /* TS_SERIES_IDS_H */
