/*
 * A/B Testing runtime toggle API
 *
 * Generic key-value store for runtime comparison of firmware algorithm
 * variants without reflashing.  Values are exposed over BLE via the
 * A/B Testing GATT service and default to 0 (variant A / old behaviour).
 */

#ifndef AB_TESTING_H
#define AB_TESTING_H

#include <stdint.h>

/* Toggle indices -- add new experiments here */
#define AB_TEST_RCPHASE_DIRECTION  0  /* 0 = add (old), 1 = subtract (fix) */
#define AB_TEST_CIR_TAP_OFFSET    1  /* 0 = fp_index,   1 = fp_index + 1  */
#define AB_TEST_MAX_TOGGLES       8

/*
 * Return the current value for toggle @p index.
 * Returns 0 for out-of-range indices.
 */
uint8_t ab_testing_get_value(uint8_t index);

#endif /* AB_TESTING_H */
