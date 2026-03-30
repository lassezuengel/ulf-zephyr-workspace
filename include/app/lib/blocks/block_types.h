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
    BLOCK_TYPE_MM_REFERENCE = 4,    /**< Dual-channel reference ranging */
    BLOCK_TYPE_LS_POSITION = 5,     /**< Least-squares positioning */
    BLOCK_TYPE_TIME_SYNC_CHECK = 6, /**< Time sync verification */
    BLOCK_TYPE_PF_POSITION = 7,     /**< Particle filter positioning */
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
 * @brief MM Reference (dual-channel) block BLE configuration
 */
struct mm_reference_config_ble {
    uint32_t respond_interval_us;
    uint32_t guard_period_us;
    uint16_t timeout_us;
    uint16_t initiator_addr;
    uint16_t responder_addr;
    uint8_t channel_a;
    uint8_t channel_b;
} __packed;

/**
 * @brief LS Position block BLE configuration
 */
struct ls_position_config_ble {
    uint8_t min_anchors;
    uint16_t max_age_ms;
} __packed;

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
    uint8_t slot_duration_ms;   /**< Per-block duration (0 = use global default) */
    union {
        struct glossy_config_ble glossy;
        struct mtm_config_ble mtm;
        struct mm_config_ble mm;
        struct mm_reference_config_ble mm_reference;
        struct ls_position_config_ble ls_position;
        struct pf_position_config_ble pf_position;
        struct time_sync_check_config_ble time_sync_check;
        uint8_t raw[BLOCK_CONFIG_BLE_UNION_SIZE];
    } config;
} __packed;

/**
 * @brief Size of a single block config for BLE transmission
 */
#define BLOCK_CONFIG_BLE_SIZE (2 + BLOCK_CONFIG_BLE_UNION_SIZE)

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
