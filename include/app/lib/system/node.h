#ifndef NODE_ID_H
#define NODE_ID_H

#include <stdint.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/lib/localization/location.h>

typedef enum {
    /**
     * Device is completely turned off.
     */
    NODE_MODE_DISABLED = 0,

    /**
     * Device participates in TDOA (Time Difference of Arrival) operations.
     * Acts as a passive node receiving signals for positioning.
     */
    NODE_MODE_PASSIVE = 1,

    /**
     * Device actively engages in TWR (Two-Way Ranging) operations.
     * Initiates ranging measurements and exchanges messages with other nodes.
     */
    NODE_MODE_ACTIVE = 2,

    /**
     * Device participates in time synchronization but does not perform
     * any ranging operations.
     */
    NODE_MODE_RANGING_DISABLED = 3
} node_mode_t;

/**
 * @brief Position estimation mode for this node
 *
 * Determines how this node's position is obtained:
 * - STATIC: Position is manually configured (node acts as anchor)
 * - LEAST_SQUARES: Position estimated from ranging to anchor nodes
 * - BELIEF_PROPAGATION: Reserved for future cooperative localization
 * - PARTICLE_FILTER: Position estimated using particle filter
 */
typedef enum {
    /**
     * Node has a static, configured position (acts as anchor).
     * Position is set manually and does not change.
     */
    NODE_POSITION_MODE_STATIC = 0,

    /**
     * Node estimates its position using least squares optimization
     * from ranging measurements to anchor (static) nodes.
     */
    NODE_POSITION_MODE_LEAST_SQUARES = 1,

    /**
     * Node estimates position using belief propagation algorithm.
     * (Reserved for future implementation - currently does nothing)
     */
    NODE_POSITION_MODE_BELIEF_PROPAGATION = 2,

    /**
     * Node estimates its position using a particle filter (Sequential Monte Carlo).
     * More robust to non-Gaussian noise than least squares but uses more
     * computation and memory. Anchors in this mode use 1 particle.
     */
    NODE_POSITION_MODE_PARTICLE_FILTER = 3,
} node_position_mode_t;

/**
 * @brief Node configuration position structure (absolute coordinates in meters)
 *
 * Stores the configured absolute position of this node, as opposed to
 * estimated/measured positions. Uses double precision for compatibility
 * with external position configuration systems.
 */
struct node_config_position {
    double x;  /* X coordinate in meters */
    double y;  /* Y coordinate in meters */
    double z;  /* Z coordinate in meters */
};

node_mode_t get_node_mode();
void        set_node_mode(node_mode_t mode);
deca_short_addr_t get_node_addr();

/* returns rtc of last position update on success, otherwise on error -1 */
int64_t get_node_location(struct vec3d_f *pos);

/**
 * @brief Get whether this node is configured as the network root (Glossy flood initiator)
 *
 * @return true if this node should initiate Glossy floods, false otherwise
 */
bool get_node_is_root(void);

/**
 * @brief Set whether this node is the network root and persist to settings
 *
 * @param is_root true to make this node the Glossy flood initiator
 * @return 0 on success, negative errno on failure
 */
int set_node_is_root(bool is_root);

/**
 * @brief Get the stored node configuration position
 *
 * @param pos Pointer to position structure to fill
 * @return 0 on success, negative errno on failure
 */
int get_node_config_position(struct node_config_position *pos);

/**
 * @brief Set the node configuration position and persist to settings
 *
 * @param pos Pointer to position structure
 * @return 0 on success, negative errno on failure
 */
int set_node_config_position(const struct node_config_position *pos);

/**
 * @brief Set position without persisting to NVS
 *
 * Use this for frequently updated positions (e.g., from LS localization)
 * to avoid flash wear and latency from NVS writes.
 *
 * @param pos Pointer to position structure
 */
void set_node_position_volatile(const struct node_config_position *pos);

/**
 * @brief Print node settings to console in JSON format
 *
 * Prints node address, mode, root status, and position (if configured).
 * Output format is JSON for easy parsing and logging.
 */
void node_print_settings(void);

/**
 * @brief Get the current position estimation mode
 *
 * @return Current position mode
 */
node_position_mode_t get_node_position_mode(void);

/**
 * @brief Set the position estimation mode and persist to settings
 *
 * @param mode New position mode
 * @return 0 on success, negative errno on failure
 */
int set_node_position_mode(node_position_mode_t mode);

/**
 * @brief Check if this node acts as an anchor (has static position)
 *
 * Anchors have static, manually configured positions and broadcast
 * their position to mobile nodes for localization.
 *
 * @return true if node has static position mode (is an anchor)
 */
static inline bool node_is_anchor(void) {
    return get_node_position_mode() == NODE_POSITION_MODE_STATIC;
}

/**
 * @brief Callback type for position change notifications
 *
 * Called when the node's position is updated via set_node_position_volatile()
 * or set_node_config_position().
 *
 * @param x X coordinate in meters
 * @param y Y coordinate in meters
 * @param z Z coordinate in meters
 */
typedef void (*node_position_changed_cb_t)(float x, float y, float z);

/**
 * @brief Register a callback for position change notifications
 *
 * The callback will be invoked whenever the node's position is updated.
 * Only one callback can be registered at a time.
 *
 * @param cb Callback function, or NULL to unregister
 */
void node_register_position_callback(node_position_changed_cb_t cb);

#endif
