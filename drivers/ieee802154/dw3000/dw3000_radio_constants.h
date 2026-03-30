/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DW3000_RADIO_CONSTANTS_H_
#define DW3000_RADIO_CONSTANTS_H_

#include <stdint.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DW3000 Time Conversion Constants */
#define DW3000_TIME_UNITS_PER_US    63898    // 499.2 MHz * 128 = 63897.6 time units per us (rounded)
#define DW3000_TIMESTAMP_MASK       0xFFFFFFFFFFULL // 40-bit timestamp
#define DW3000_TIMESTAMP_RESOLUTION_PS  15.65f   // Time resolution in picoseconds

/* DW3000 CFO (Crystal Frequency Offset) Conversion */
#define DW3000_CFO_CONVERSION_FACTOR   -1.0e-6f  // DW3000-specific CFO conversion

/* DW3000 RSSI Constants for different PRF configurations */
#define DW3000_RX_SIG_PWR_A_CONST_PRF16  121.74f
#define DW3000_RX_SIG_PWR_A_CONST_PRF64  121.74f

/* DW3000 Register Constants */
#define DW3000_MAX_FRAME_LEN    127
#define DW3000_FCS_LEN          2
#define DW3000_CIR_SIZE         4064    // Channel Impulse Response buffer size

/* DW3000 Antenna Delay Constants (typical values) */
#define DW3000_DEFAULT_ANTENNA_DELAY_TX  16450   // Default TX antenna delay
#define DW3000_DEFAULT_ANTENNA_DELAY_RX  16450   // Default RX antenna delay

/* DW3000 Radio Constants Structure */
static const uwb_radio_constants_t dw3000_radio_constants = {
    .timestamp_mask = DW3000_TIMESTAMP_MASK,
    .cfo_conversion_factor = DW3000_CFO_CONVERSION_FACTOR,
    .timestamp_resolution_ps = DW3000_TIMESTAMP_RESOLUTION_PS,
    .rssi_constants = {
        .prf16 = DW3000_RX_SIG_PWR_A_CONST_PRF16,
        .prf64 = DW3000_RX_SIG_PWR_A_CONST_PRF64
    }
};

/* Function to get DW3000 radio constants */
static inline const uwb_radio_constants_t *dw3000_get_radio_constants(void)
{
    return &dw3000_radio_constants;
}

#ifdef __cplusplus
}
#endif

#endif /* DW3000_RADIO_CONSTANTS_H_ */