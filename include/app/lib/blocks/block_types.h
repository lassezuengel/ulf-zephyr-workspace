/**
 * @file block_types.h
 * @brief Block type definitions and BLE-serializable configuration structures
 *
 * This header defines the block type enumeration and packed structures
 * suitable for BLE transmission and NVS storage of superframe configurations.
 */

#ifndef SYNCHROFLY_BLOCK_TYPES_H
#define SYNCHROFLY_BLOCK_TYPES_H

#include <stdint.h>
#include <zephyr/sys/util.h>

/**
 * @brief Block type enumeration
 *
 * These values are part of the BLE/NVS ABI - do not reorder, only append.
 */
typedef enum {
    BLOCK_TYPE_NONE = 0,            /**< Empty/disabled slot */
    BLOCK_TYPE_GLOSSY = 1,          /**< Time synchronization (glossy flooding) */
    BLOCK_TYPE_MTM = 2,             /**< Multi-to-multi ranging */
    BLOCK_TYPE_MM = 3,              /**< Millimeter accuracy ranging */
    BLOCK_TYPE_MM_REFERENCE = 4,    /**< Single-channel reference ranging */
    BLOCK_TYPE_LS_POSITION = 5,     /**< Least-squares positioning */
    BLOCK_TYPE_TIME_SYNC_CHECK = 6, /**< Time sync verification */
    BLOCK_TYPE_PF_POSITION = 7,     /**< Particle filter positioning */
    BLOCK_TYPE_ANNOUNCEMENT = 8,    /**< Node presence announcement (glossy flooding) */
    BLOCK_TYPE_SWARM_RANGING = 9,   /**< Swarm ranging protocol slot (radio released for async use) */
    BLOCK_TYPE_MULOC = 10,          /**< MULoc anchor overhearing protocol */
    BLOCK_TYPE_CIR_READ = 11,       /**< CIR (Channel Impulse Response) read */
    BLOCK_TYPE_BUTLER = 12,         /**< Butler distributed time synchronization */
    BLOCK_TYPE_IDLE = 13,           /**< Idle slot (no radio activity, placeholder for sleep) */
    BLOCK_TYPE_MAX
} block_type_t;

/**
 * @brief Schedule type for ranging blocks
 * Must match schedule_type_t in mtm.h
 */
typedef enum {
    BLE_SCHEDULE_BASIC = 0,
    BLE_SCHEDULE_HASHED = 1,
    BLE_SCHEDULE_CONTENTION = 2,
} ble_schedule_type_t;

/**
 * @brief Glossy block BLE configuration
 */
struct glossy_config_ble {
    uint16_t max_depth;
    uint16_t transmission_delay_us;
    uint16_t guard_period_us;
    uint8_t channel;
} __packed;

/**
 * @brief MTM (multi-to-multi) block BLE configuration
 */
struct mtm_config_ble {
    uint8_t schedule_type;      /**< ble_schedule_type_t */
    uint8_t slots_per_phase;
    uint8_t phases;
    uint16_t slot_padding_us;   /**< Per-slot padding in us (0 = use global network setting) */
} __packed;

/**
 * @brief MM (millimeter accuracy) block BLE configuration
 */
struct mm_config_ble {
    uint8_t schedule_type;      /**< ble_schedule_type_t */
    uint8_t slots_per_phase;
    uint8_t phases;
    uint16_t initiator_addr;
    uint16_t responder_addr;
} __packed;

/**
 * @brief MM Reference (single-channel) block BLE configuration
 *
 * Each block instance performs one TWR exchange on one channel.
 * Place multiple blocks in the superframe for multi-channel operation.
 */
struct mm_reference_config_ble {
    uint32_t respond_interval_us;
    uint32_t guard_period_us;
    uint16_t timeout_us;
    uint16_t initiator_addr;
    uint16_t responder_addr;
    uint8_t channel;
} __packed;

/**
 * @brief LS Position block BLE configuration
 */
struct ls_position_config_ble {
    uint8_t min_anchors;
    uint16_t max_age_ms;
    uint8_t flags;  /**< Bit 0: constrain Z >= 0 (default: enabled) */
} __packed;

/** LS Position flag: constrain Z coordinate to be non-negative */
#define LS_POSITION_FLAG_CONSTRAIN_Z_POSITIVE  0x01

/**
 * @brief PF Position (particle filter) block BLE configuration
 */
struct pf_position_config_ble {
    uint8_t min_anchors;
    uint16_t max_age_ms;
    uint16_t particle_count;
    uint8_t measurement_variance_x10;  /**< Variance * 10 (e.g., 5 = 0.5 m^2) */
    uint8_t process_noise_std_x100;    /**< Std dev * 100 (e.g., 10 = 0.1 m) */
    uint8_t send_particles;            /**< 0 = send mean only, 1 = send particles */
} __packed;

/**
 * @brief Time sync check block BLE configuration
 */
struct time_sync_check_config_ble {
    uint8_t is_network_root;
} __packed;

/**
 * @brief Announcement block BLE configuration
 *
 * Uses glossy-style flooding to announce node presence.
 * Each node announces with probability announce_probability_pct per iteration.
 */
struct announcement_config_ble {
    uint16_t max_depth;
    uint16_t transmission_delay_us;
    uint16_t guard_period_us;
    uint8_t channel;
    uint8_t announce_probability_pct; /**< Probability of announcing (0-100, 0=never, 100=always) */
} __packed;

/**
 * @brief Swarm ranging block BLE configuration
 */
struct swarm_ranging_config_ble {
    uint8_t channel;              /**< UWB channel for background protocol (0 = keep current) */
    uint8_t filter_enabled;       /**< Distance filter: 0=disabled, 1=enabled */
    uint16_t period_ms;           /**< Sending interval in ms (0 = keep current, valid: 50-500) */
    uint8_t bus_boarding_enabled; /**< Bus boarding scheduling: 0=disabled, 1=enabled */
} __packed;

/**
 * @brief MULoc (anchor overhearing) block BLE configuration
 */
struct muloc_config_ble {
    uint8_t anchor_count;        /**< Number of anchors (2-8) */
    uint8_t anchor_id;           /**< This node's anchor ID (0..N-1, 0xFF=listener) */
    uint8_t num_rounds;          /**< Rounds per execution (1-8, typically 2-4) */
    uint16_t delay_time_us;      /**< Inter-anchor TX delay in us (default 600) */
    uint8_t start_channel;       /**< Starting UWB channel (1 or 5) */
    uint8_t hop_channel;         /**< Frequency hop channel (3) */
    uint8_t reserved;            /**< Padding for alignment */
} __packed;

/**
 * @brief CIR Read block BLE configuration
 *
 * Each node is configured as sender (transmit probe) or receiver (listen and
 * read CIR accumulator). Timing is derived from glossy time synchronization.
 */
struct cir_read_config_ble {
    uint8_t  mode;              /**< 0=CIR_MODE_SENDER, 1=CIR_MODE_RECEIVER */
    uint8_t  channel;           /**< UWB channel (0 = keep current) */
    uint16_t tx_delay_dtu;      /**< Sender TX delay offset (short_ts units, ~4ns/step) */
    uint16_t from_index;        /**< Receiver: start sample index (or offset before FP) */
    uint16_t to_index;          /**< Receiver: end sample index (or offset after FP) */
    uint8_t  only_first_path;   /**< Receiver: if true, from/to relative to fp_index */
    uint8_t  _reserved[7];      /**< Padding to 16 bytes */
} __packed;

/**
 * @brief Butler (distributed time sync) block BLE configuration
 */
struct butler_config_ble {
    uint8_t max_subslots;
    uint16_t subslot_duration_us;
    uint16_t guard_period_us;
    uint8_t p_tx_pct;            /**< TX probability percentage (0-100) */
    uint8_t channel;
} __packed;

/**
 * @brief Maximum size of block-specific config in the union
 */
#define BLOCK_CONFIG_BLE_UNION_SIZE 16

/**
 * @brief BLE-serializable block configuration
 *
 * This structure is used for BLE transmission and NVS storage.
 * It contains a type discriminator and a union of block-specific configs.
 */
struct block_config_ble {
    uint8_t type;               /**< block_type_t */
    uint16_t slot_duration_ms;  /**< Per-block duration in ms (0 = use global default) */
    union {
        struct glossy_config_ble glossy;
        struct mtm_config_ble mtm;
        struct mm_config_ble mm;
        struct mm_reference_config_ble mm_reference;
        struct ls_position_config_ble ls_position;
        struct pf_position_config_ble pf_position;
        struct time_sync_check_config_ble time_sync_check;
        struct announcement_config_ble announcement;
        struct swarm_ranging_config_ble swarm_ranging;
        struct muloc_config_ble muloc;
        struct cir_read_config_ble cir_read;
        struct butler_config_ble butler;
        uint8_t raw[BLOCK_CONFIG_BLE_UNION_SIZE];
    } config;
} __packed;

/**
 * @brief Size of a single block config for BLE transmission
 */
#define BLOCK_CONFIG_BLE_SIZE (3 + BLOCK_CONFIG_BLE_UNION_SIZE)

/**
 * @brief Maximum number of blocks in a superframe
 */
#define SUPERFRAME_MAX_BLOCKS 16

/**
 * @brief BLE-serializable superframe configuration
 */
struct superframe_config_ble {
    uint8_t block_count;
    struct block_config_ble blocks[SUPERFRAME_MAX_BLOCKS];
} __packed;

/**
 * @brief Maximum size of superframe config for BLE transmission
 */
#define SUPERFRAME_CONFIG_BLE_MAX_SIZE (1 + SUPERFRAME_MAX_BLOCKS * BLOCK_CONFIG_BLE_SIZE)

#endif /* SYNCHROFLY_BLOCK_TYPES_H */
