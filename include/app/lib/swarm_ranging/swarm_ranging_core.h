/**
 * @file swarm_ranging_core.h
 * @brief Swarm ranging core logic API
 */

#ifndef SWARM_RANGING_CORE_H
#define SWARM_RANGING_CORE_H

#include <app/lib/swarm_ranging/swarm_ranging_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * State Machine
 * ============================================================================ */

/**
 * Process ranging table event
 *
 * @param table Ranging table pointer
 * @param event Event type
 */
void rangingTableOnEvent(Ranging_Table_t *table, RANGING_TABLE_EVENT event);

/* ============================================================================
 * Distance Computation
 * ============================================================================ */

/**
 * Compute distance using Two-Way Ranging
 *
 * @param Tp Previous TX timestamp
 * @param Rp Previous RX timestamp
 * @param Tr Remote TX timestamp
 * @param Rr Remote RX (local) timestamp
 * @param Tf Final TX timestamp
 * @param Rf Final RX timestamp
 * @return Distance in cm, or -1 on error
 */
int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                       Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                       Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);

/* ============================================================================
 * Message Processing
 * ============================================================================ */

/**
 * Process received ranging message
 *
 * @param rangingMessageWithTimestamp Received message with RX timestamp
 * @param my_address Local UWB address
 */
void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp,
                          uint16_t my_address);

/**
 * Generate ranging message for transmission
 *
 * @param rangingMessage Output buffer for ranging message
 * @param my_address Local UWB address
 * @return Task delay in ms until next message should be sent
 */
Time_t generateRangingMessage(Ranging_Message_t *rangingMessage, uint16_t my_address);

/* ============================================================================
 * Initialization
 * ============================================================================ */

/**
 * Initialize ranging subsystem
 *
 * @param uwb_address Local UWB address
 * @return 0 on success, negative error code on failure
 */
int ranging_init(uint16_t uwb_address);

/**
 * @brief Deinitialize swarm ranging, freeing heap allocations.
 * Aborts threads, cancels eviction work, frees neighbor set and ranging table.
 * Safe to call if not initialized (no-op).
 */
void ranging_deinit(void);

/* ============================================================================
 * Runtime Configuration
 * ============================================================================ */

/**
 * Set base ranging period (clamped to RANGING_PERIOD_MIN..RANGING_PERIOD_MAX)
 */
void swarm_ranging_set_period(uint16_t period_ms);

/**
 * Get current base ranging period
 */
uint16_t swarm_ranging_get_period(void);

/**
 * Enable/disable distance outlier filter (rejects < 0 and > 1000 cm)
 */
void swarm_ranging_set_distance_filter(bool enabled);

/**
 * Get current distance filter state
 */
bool swarm_ranging_get_distance_filter(void);

/**
 * Enable/disable bus boarding scheduling scheme.
 * When enabled, neighbors are sorted by next delivery time (fairer).
 * When disabled, sorted by last send time (simple round-robin).
 */
void swarm_ranging_set_bus_boarding(bool enabled);

/**
 * Get current bus boarding state
 */
bool swarm_ranging_get_bus_boarding(void);

/**
 * Reset all swarm ranging state (tables, buffers, sequence numbers).
 * Threads keep running but start fresh. Call after superframe reconfiguration.
 */
void swarm_ranging_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_RANGING_CORE_H */
