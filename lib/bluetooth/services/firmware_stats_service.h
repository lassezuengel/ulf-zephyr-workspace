/*
 * Firmware Statistics BLE GATT Service
 *
 * Service UUID base: "fwstats\0" = 0x66777374-6174-7300
 */

#ifndef FIRMWARE_STATS_SERVICE_H
#define FIRMWARE_STATS_SERVICE_H

#include <zephyr/bluetooth/uuid.h>

/* Service UUID: 66777374-6174-7300-0000-000000000000 */
#define FIRMWARE_STATS_SERVICE_UUID_VAL \
	BT_UUID_128_ENCODE(0x66777374, 0x6174, 0x7300, 0x0000, 0x000000000000ULL)

/* Subscribe characteristic: write uint32 LE stat_id to start tracking */
#define FIRMWARE_STATS_SUBSCRIBE_UUID_VAL \
	BT_UUID_128_ENCODE(0x66777374, 0x6174, 0x7300, 0x0000, 0x000000000001ULL)

/* Unsubscribe characteristic: write uint32 LE stat_id to stop tracking */
#define FIRMWARE_STATS_UNSUBSCRIBE_UUID_VAL \
	BT_UUID_128_ENCODE(0x66777374, 0x6174, 0x7300, 0x0000, 0x000000000002ULL)

/* Select characteristic: read/write uint32 LE -- select stat for value read/notify */
#define FIRMWARE_STATS_SELECT_UUID_VAL \
	BT_UUID_128_ENCODE(0x66777374, 0x6174, 0x7300, 0x0000, 0x000000000003ULL)

/* Value characteristic: read/notify uint32 LE -- current value of selected stat */
#define FIRMWARE_STATS_VALUE_UUID_VAL \
	BT_UUID_128_ENCODE(0x66777374, 0x6174, 0x7300, 0x0000, 0x000000000004ULL)

/* Active count characteristic: read uint8 -- number of active slots */
#define FIRMWARE_STATS_ACTIVE_COUNT_UUID_VAL \
	BT_UUID_128_ENCODE(0x66777374, 0x6174, 0x7300, 0x0000, 0x000000000005ULL)

#endif /* FIRMWARE_STATS_SERVICE_H */
