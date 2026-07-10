/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TIMESYNC_BUTLER_H
#define TIMESYNC_BUTLER_H

#include <stdint.h>
#include <stdbool.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>

/**
 * @brief Butler on-wire frame structure
 */
struct __attribute__((__packed__)) butler_frame {
    uint8_t msg_id;
    uint8_t remaining_slots;
    uint16_t sender_node_id;
    uint16_t sigma_node_id;
    uint32_t sigma_rtc_at_tau;
    uwb_packed_ts_t sigma_dwt_at_tau;
    uwb_packed_ts_t sigma_tx_dwt;
    uint8_t hop_count;
};

/**
 * @brief Resync seed: Butler frame info captured during another block.
 * Set by the intercepting block (e.g., MTM), consumed by the next Butler call.
 */
struct butler_resync_seed {
    bool valid;
    uint16_t sigma_node_id;
    uint32_t sigma_rtc_at_tau;
    uint64_t sigma_dwt_at_tau;
    uint32_t local_rx_rtc;
    uint64_t local_rx_dwt;
    uint8_t remaining_slots;
};

void butler_set_resync_seed(const struct butler_resync_seed *seed);

/**
 * @brief Butler flood configuration
 */
struct butler_flood_config {
    deca_short_addr_t node_addr;
    uint8_t max_subslots;          /**< Total subslots per Butler round */
    uint16_t subslot_duration_us;  /**< Duration of each subslot */
    uint16_t guard_period_us;      /**< Guard period for RX timeout */
    uint8_t p_tx_pct;             /**< TX probability percentage (0-100) */
};

void butler_store_config(const struct butler_flood_config *conf);
const struct butler_flood_config *butler_get_last_config(void);

/**
 * @brief Butler flood result
 *
 * Contains clock pairs suitable for feeding into time_sync_update()
 * via a deca_glossy_result struct.
 */
struct butler_flood_result {
    uint16_t sigma_node_id;        /**< Who proposed the winning tau */
    bool converged;                /**< At least one sync frame received */
    struct deca_glossy_time_pair rtc_clock_pair;
    struct deca_glossy_time_pair deca_clock_pair;
    /* Post-convergence beacon for skew estimation.
     * SYNC frame carries sigma's original TX timestamp (preserved through relays).
     * Beacon is transmitted directly by sigma (hop_count=0).
     * Receiver corrects for hop delay: sync_local_rx includes hop_count * subslot
     * of relay delay that sigma_tx does not. */
    bool has_beacon;
    uint64_t sync_local_rx_dwt;    /**< Local DWT at SYNC frame RX */
    uint64_t sync_sigma_tx_dwt;    /**< Sigma's original TX timestamp (from frame, preserved through relays) */
    uint8_t  sync_hop_count;       /**< Hops from sigma to this node at SYNC */
    uint64_t beacon_dwt;           /**< Local DWT at beacon RX */
    uint64_t sigma_beacon_dwt;     /**< Sigma's DWT at beacon TX (from frame, hop_count=0) */
    uint32_t beacon_rtc;           /**< Local RTC at beacon RX */
    uint32_t sigma_beacon_rtc;     /**< Sigma's RTC at beacon (from frame) */
};

/**
 * @brief Execute a Butler distributed synchronization round
 *
 * Implements Algorithm 1 from Mager et al. "Butler: Increasing the
 * Availability of Low-Power Wireless Communication Protocols".
 *
 * All nodes run the same code path (no initiator/receiver distinction).
 * Each subslot, a node probabilistically decides to TX or RX.
 * Nodes converge on the earliest proposed reference time (tau).
 *
 * Extended from the paper: frames carry sigma's absolute RTC+DWT
 * timestamps at tau, enabling clock synchronization (not just temporal
 * alignment). These timestamps are relayed verbatim through the network.
 *
 * @param dev UWB device
 * @param conf Flood configuration
 * @param result Output: converged reference time and clock pairs
 * @return 0 on success, negative error code on failure
 */
int uwb_butler_flood(const struct device *dev,
                     struct butler_flood_config *conf,
                     struct butler_flood_result *result);

#endif /* TIMESYNC_BUTLER_H */
