/**
 * @file swarm_ranging_types.h
 * @brief Swarm Ranging Protocol Data Structures
 *
 * Port of SEU-NetSI swarm ranging data structures to Zephyr.
 * IMPORTANT: Wire format structures must remain byte-compatible with original.
 */

#ifndef SWARM_RANGING_TYPES_H
#define SWARM_RANGING_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <app/lib/swarm_ranging/platform_zephyr.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/* Ranging Constants */
#ifndef CONFIG_SWARM_RANGING_PERIOD_MS
#define RANGING_PERIOD 200  // default in 200ms
#else
#define RANGING_PERIOD CONFIG_SWARM_RANGING_PERIOD_MS
#endif

#define RANGING_PERIOD_MIN 50   // default 50ms
#define RANGING_PERIOD_MAX 500  // default 500ms

/* UWB Constants */
#define UWB_SPEED_OF_LIGHT 299702547
#define UWB_MAX_TIMESTAMP 0xFFFFFFFFFFULL  // 2^40 = 1099511627776
#define UWB_TX_ANT_DLY 16385
#define UWB_RX_ANT_DLY 16385

/* Message Size Constants */
#define UWB_FRAME_LEN_STD 127
#define UWB_FRAME_LEN_MAX UWB_FRAME_LEN_STD
#define UWB_PACKET_SIZE_MAX UWB_FRAME_LEN_MAX
#define UWB_PAYLOAD_SIZE_MAX (UWB_PACKET_SIZE_MAX - sizeof(UWB_Packet_Header_t))

#define RANGING_MESSAGE_SIZE_MAX UWB_PAYLOAD_SIZE_MAX
#define RANGING_MESSAGE_PAYLOAD_SIZE_MAX (RANGING_MESSAGE_SIZE_MAX - sizeof(Ranging_Message_Header_t))
#define RANGING_MAX_Tr_UNIT 3
#define RANGING_MAX_BODY_UNIT (RANGING_MESSAGE_PAYLOAD_SIZE_MAX / sizeof(Body_Unit_t))

/* Ranging Table Constants */
#ifndef CONFIG_SWARM_RANGING_TABLE_SIZE
#define RANGING_TABLE_SIZE_MAX 20  // default up to 20 one-hop neighbors
#else
#define RANGING_TABLE_SIZE_MAX CONFIG_SWARM_RANGING_TABLE_SIZE
#endif

#define RANGING_TABLE_HOLD_TIME (6 * RANGING_PERIOD_MAX)
#define Tr_Rr_BUFFER_POOL_SIZE 3
#define Tf_BUFFER_POOL_SIZE (2 * RANGING_PERIOD_MAX / RANGING_PERIOD_MIN)

/* Topology Sensing */
#ifndef CONFIG_SWARM_NEIGHBOR_ADDRESS_MAX
#define NEIGHBOR_ADDRESS_MAX 32
#else
#define NEIGHBOR_ADDRESS_MAX CONFIG_SWARM_NEIGHBOR_ADDRESS_MAX
#endif

#define NEIGHBOR_SET_HOLD_TIME (6 * RANGING_PERIOD_MAX)

/* Queue Constants */
#ifndef CONFIG_SWARM_RANGING_RX_QUEUE_SIZE
#define RANGING_RX_QUEUE_SIZE 5  // default
#else
#define RANGING_RX_QUEUE_SIZE CONFIG_SWARM_RANGING_RX_QUEUE_SIZE
#endif

#define RANGING_RX_QUEUE_ITEM_SIZE sizeof(Ranging_Message_With_Timestamp_t)

/* Address Constants */
#define UWB_DEST_ANY 65535
#define UWB_DEST_EMPTY 65534

typedef uint16_t UWB_Address_t;
typedef int16_t set_index_t;

/* ============================================================================
 * UWB Packet Types
 * ============================================================================ */

typedef enum {
    UWB_RANGING_MESSAGE = 0,
    UWB_FLOODING_MESSAGE = 1,
    UWB_DATA_MESSAGE = 2,
    UWB_AODV_MESSAGE = 3,
    UWB_OLSR_MESSAGE = 4,
    UWB_MESSAGE_TYPE_COUNT,
} UWB_MESSAGE_TYPE;

typedef struct {
    UWB_Address_t srcAddress;
    UWB_Address_t destAddress;
    UWB_MESSAGE_TYPE type: 6;
    uint16_t length: 10;
} __attribute__((packed)) UWB_Packet_Header_t;

typedef struct {
    UWB_Packet_Header_t header;
    uint8_t payload[UWB_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) UWB_Packet_t;

/* ============================================================================
 * Timestamp and Ranging Message Structures (WIRE FORMAT - DO NOT MODIFY)
 * ============================================================================ */

/**
 * Timestamp Tuple (10 bytes)
 * Contains 40-bit UWB timestamp and sequence number
 */
typedef struct {
    uint64_t timestamp : 40;  // UWB timestamp (40-bit)
    uint16_t seqNumber;       // Sequence number
} __attribute__((packed)) Timestamp_Tuple_t;

/**
 * Body Unit (13 bytes)
 * Contains neighbor info and timestamp for ranging message
 */
typedef struct {
    struct {
        uint8_t MPR: 1;        // MPR (Multi-Point Relay) flag for OLSR
        uint8_t RESERVED: 7;
    } flags;
    uint16_t address;          // Neighbor address
    Timestamp_Tuple_t timestamp;  // Timestamp tuple
} __attribute__((packed)) Body_Unit_t;

/**
 * Ranging Message Header
 * Must match SEU-NetSI format exactly
 */
typedef struct {
    uint16_t srcAddress;
    uint16_t msgSequence;
    Timestamp_Tuple_t lastTxTimestamps[RANGING_MAX_Tr_UNIT];
    int16_t velocity;  // cm/s
    uint16_t msgLength;
    uint16_t filter;   // 16-bit bloom filter
} __attribute__((packed)) Ranging_Message_Header_t;

/**
 * Ranging Message (full packet)
 */
typedef struct {
    Ranging_Message_Header_t header;
    Body_Unit_t bodyUnits[RANGING_MAX_BODY_UNIT];
} __attribute__((packed)) Ranging_Message_t;

/**
 * Ranging Message With RX Timestamp
 * Used in RX queue
 */
typedef struct {
    Ranging_Message_t rangingMessage;
    uint64_t rxTime;  // 40-bit timestamp
} __attribute__((packed)) Ranging_Message_With_Timestamp_t;

/* ============================================================================
 * Ranging Table Structures
 * ============================================================================ */

typedef struct {
    Timestamp_Tuple_t Tr;
    Timestamp_Tuple_t Rr;
} __attribute__((packed)) Ranging_Table_Tr_Rr_Candidate_t;

/**
 * Tr and Rr candidate buffer for each Ranging Table
 */
typedef struct {
    set_index_t latest;  // Index of latest valid (Tr,Rr) pair
    set_index_t cur;     // Index of current empty slot for next pair
    Ranging_Table_Tr_Rr_Candidate_t candidates[Tr_Rr_BUFFER_POOL_SIZE];
} Ranging_Table_Tr_Rr_Buffer_t;

/**
 * Ranging Table State Machine States
 */
typedef enum {
    RANGING_STATE_RESERVED,
    RANGING_STATE_S1,
    RANGING_STATE_S2,
    RANGING_STATE_S3,
    RANGING_STATE_S4,
    RANGING_STATE_S5,  // Temporary state for distance calculation
    RANGING_TABLE_STATE_COUNT
} RANGING_TABLE_STATE;

/**
 * Ranging Table Events
 */
typedef enum {
    RANGING_EVENT_TX_Tf,
    RANGING_EVENT_RX_NO_Rf,
    RANGING_EVENT_RX_Rf,
    RANGING_TABLE_EVENT_COUNT
} RANGING_TABLE_EVENT;

/**
 * Ranging Table (per neighbor)
 */
typedef struct {
    uint16_t neighborAddress;

    Timestamp_Tuple_t Rp;
    Timestamp_Tuple_t Tp;
    Ranging_Table_Tr_Rr_Buffer_t TrRrBuffer;
    Timestamp_Tuple_t Rf;
    Timestamp_Tuple_t Tf;
    Timestamp_Tuple_t Re;
    Timestamp_Tuple_t latestReceived;

    Time_t period;
    Time_t nextExpectedDeliveryTime;
    Time_t expirationTime;
    Time_t lastSendTime;
    int16_t distance;

    RANGING_TABLE_STATE state;
} Ranging_Table_t;

/**
 * Ranging Table Set
 */
typedef struct {
    int size;
    struct k_mutex mu;
    Ranging_Table_t tables[RANGING_TABLE_SIZE_MAX];
} Ranging_Table_Set_t;

/**
 * Event handler function pointer
 */
typedef void (*RangingTableEventHandler)(Ranging_Table_t *);

/* ============================================================================
 * Neighbor Set Structures
 * ============================================================================ */

/**
 * Neighbor Bit Set (uses 64-bit bitfield)
 */
typedef struct {
    uint64_t bits;
    uint8_t size;
} Neighbor_Bit_Set_t;

/**
 * Neighbor set hook function pointer
 */
typedef void (*neighborSetHook)(UWB_Address_t);

/**
 * Hook list node
 */
typedef struct Neighbor_Set_Hook {
    neighborSetHook hook;
    struct Neighbor_Set_Hook *next;
} Neighbor_Set_Hooks_t;

/**
 * Neighbor Entry (stores info for one neighbor)
 * Used in dense neighbor table (search-based, not direct indexed)
 */
typedef struct {
    UWB_Address_t address;
    bool active;
    Neighbor_Bit_Set_t twoHopReachSet;  // Bits represent indices of one-hop neighbors
    Time_t expirationTime;
} Neighbor_Entry_t;

/**
 * Neighbor Set (tracks 1-hop and 2-hop neighbors)
 * REFACTORED: Now uses dense entry table with search, not sparse arrays
 */
typedef struct {
    Neighbor_Entry_t neighbors[NEIGHBOR_ADDRESS_MAX + 1];  // Dense table, searched by address
    int count;  // Number of active entries
    struct k_mutex mu;
    Neighbor_Bit_Set_t oneHopIndices;  // Bits represent table indices (not addresses!)
    Neighbor_Bit_Set_t twoHopIndices;  // Bits represent table indices (not addresses!)
    Neighbor_Set_Hooks_t neighborNewHooks;
    Neighbor_Set_Hooks_t neighborExpirationHooks;
    Neighbor_Set_Hooks_t neighborTopologyChangeHooks;
} Neighbor_Set_t;

/* ============================================================================
 * Utility Macros
 * ============================================================================ */

/* MIN and MAX are provided by Zephyr's sys/util.h (included via platform_zephyr.h) */

#ifdef __cplusplus
}
#endif

#endif /* SWARM_RANGING_TYPES_H */
