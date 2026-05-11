/*
 * A/B Testing BLE Service
 *
 * Generic index/value GATT service for runtime algorithm variant
 * toggling during development.
 */

#ifndef AB_TESTING_SERVICE_H
#define AB_TESTING_SERVICE_H

/* Service UUID: "abtestin" in ASCII hex (61627465-7374-696e) */
#define AB_TESTING_SERVICE_UUID_VAL \
    BT_UUID_128_ENCODE(0x61627465, 0x7374, 0x696e, 0x0000, 0x000000000000ULL)

/* Index characteristic UUID (R/W uint8) */
#define AB_TESTING_INDEX_CHAR_UUID_VAL \
    BT_UUID_128_ENCODE(0x61627465, 0x7374, 0x696e, 0x0000, 0x000000000001ULL)

/* Value characteristic UUID (R/W uint8) */
#define AB_TESTING_VALUE_CHAR_UUID_VAL \
    BT_UUID_128_ENCODE(0x61627465, 0x7374, 0x696e, 0x0000, 0x000000000002ULL)

#endif /* AB_TESTING_SERVICE_H */
