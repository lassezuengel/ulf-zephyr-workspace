/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Least Squares Position Block
 *
 * This block runs after ranging blocks to:
 * 1. Process received position payloads from the MAC RX queue
 * 2. Update neighbor table with anchor positions
 * 3. Compute position using least squares (if mobile node)
 * 4. Queue position payload for next ranging round
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <app/lib/blocks/blocks.h>
#include <app/lib/system/node.h>
#include <app/lib/communication/mac_queue.h>
#include <app/lib/localization/position_payload.h>
#include <app/lib/localization/least_squares.h>
#include <app/lib/localization/location.h>
#include <app/lib/node_table/node_table.h>
#include <app/lib/timesync/time_synchronization.h>

#if IS_ENABLED(CONFIG_CRAZYFLIE)
#include <app/lib/crazyflie/connector.h>
#endif

LOG_MODULE_REGISTER(block_ls_position, LOG_LEVEL_WRN);

/* Maximum anchors to use for position estimation */
#define LS_MAX_ANCHORS 16

/* Default minimum anchors required for position estimation */
#define LS_DEFAULT_MIN_ANCHORS 3

/* Default maximum age for anchor measurements (ms) */
#define LS_DEFAULT_MAX_AGE_MS 2000

/* Convert node_entry to node_position for localization API */
static void entry_to_node_position(const struct node_entry *entry,
                                   struct node_position *pos)
{
    pos->addr = entry->node_id;
    pos->position.x = entry->pos_x;
    pos->position.y = entry->pos_y;
    pos->position.z = entry->pos_z;
}

/* Speed of light conversion factor for UWB time units to meters */
#define SPEED_OF_LIGHT_M_PER_UWB_TU 0.00469175196f

/* Convert node_entry to measurement for localization API */
static void entry_to_measurement(const struct node_entry *entry,
                                 deca_short_addr_t local_addr,
                                 struct measurement *meas)
{
    meas->ranging_initiator_id = local_addr;
    meas->ranging_responder_id = entry->node_id;
    /* Get filtered distance in meters and convert to ToF */
    float distance_m;
    if (node_table_get_filtered_distance_m(entry->node_id, &distance_m) != 0) {
        distance_m = 0.0f;
    }
    meas->tof = distance_m / SPEED_OF_LIGHT_M_PER_UWB_TU;
    meas->tdoa = 0.0f;
    meas->tof_quality = 1.0f;
    meas->tdoa_quality = 0.0f;
    meas->cfo = 0.0;
}

/**
 * @brief Process all received position payloads from MAC RX queue
 *
 * @param rtc Current RTC timestamp
 * @return Number of payloads processed
 */
static int process_rx_payloads(uint64_t rtc)
{
    struct mac_queue_frame rx_frame;
    uint8_t payload_buf[64]; /* Larger buffer for extracted payload */
    size_t payload_len;
    int count = 0;

    while (mac_queue_rx_pop(&rx_frame, K_NO_WAIT) == 0) {
        /* Extract payload from frame */
        payload_len = sizeof(payload_buf);
        if (mac_queue_extract_payload(&rx_frame, payload_buf, &payload_len) < 0) {
            LOG_WRN("Failed to extract payload from RX frame");
            continue;
        }

        /* Get sender address from frame */
        uint16_t sender_id;
        if (mac_queue_extract_src_addr(&rx_frame, &sender_id) < 0) {
            LOG_WRN("Failed to extract source address from RX frame");
            continue;
        }

        LOG_INF("RX from 0x%04x len=%u", sender_id, (unsigned)payload_len);
        LOG_HEXDUMP_INF(payload_buf, payload_len, "Extracted payload:");

        /* Process the position payload */
        int ret = position_payload_process(sender_id, payload_buf, payload_len, rtc);
        if (ret == 0) {
            count++;
        }
    }
    return count;
}

/**
 * @brief Compute position using least squares from anchor distances
 *
 * @param config Block configuration
 * @param rtc Current RTC timestamp
 * @param position Output position estimate
 * @param residual Output residual error (optional, can be NULL)
 * @return 0 on success, negative errno on failure
 */
static int compute_position(const struct ls_position_block_config *config,
                           uint64_t rtc,
                           struct vec3d_f *position, float *residual)
{
    struct node_entry anchors[LS_MAX_ANCHORS];
    struct node_position known_positions[LS_MAX_ANCHORS];
    struct measurement measurements[LS_MAX_ANCHORS];
    size_t anchor_count;
    int ret;

    /* Get anchors from neighbor table */
    anchor_count = node_table_get_anchors(anchors, LS_MAX_ANCHORS);

    uint8_t min_anchors = config ? config->min_anchors : LS_DEFAULT_MIN_ANCHORS;

    if (anchor_count < min_anchors) {
        LOG_DBG("Not enough anchors: %u < %u", (unsigned)anchor_count, min_anchors);
        return -EAGAIN;
    }

    /* Filter anchors by age if configured */
    uint16_t max_age_ms = config ? config->max_age_ms : LS_DEFAULT_MAX_AGE_MS;
    uint64_t max_age_ticks = k_ms_to_ticks_floor64(max_age_ms);

    size_t valid_count = 0;
    deca_short_addr_t local_addr = get_node_addr();

    for (size_t i = 0; i < anchor_count; i++) {
        uint64_t age = rtc - anchors[i].last_seen_rtc;

        if (age <= max_age_ticks) {
            /* Only use anchors with valid distance measurements */
            float dist_m;
            if (node_table_get_filtered_distance_m(anchors[i].node_id, &dist_m) == 0 && dist_m > 0.0f) {
                entry_to_node_position(&anchors[i], &known_positions[valid_count]);
                entry_to_measurement(&anchors[i], local_addr, &measurements[valid_count]);
                valid_count++;
            }
        }
    }

    if (valid_count < min_anchors) {
        LOG_DBG("Not enough valid anchors: %u < %u", (unsigned)valid_count, min_anchors);
        return -EAGAIN;
    }

    LOG_DBG("LS with %u anchors", (unsigned)valid_count);

    /* Run least squares solver */
    struct node_position estimate;
    estimate.addr = local_addr;

    /* Build solver flags from config */
    uint8_t solver_flags = 0;
    bool constrain_z = config ? config->constrain_z_positive : true;
    if (constrain_z) {
        solver_flags |= LS_FLAG_CONSTRAIN_Z_POSITIVE;
    }

    ret = cholesky_linear_localization(local_addr, known_positions, valid_count,
                                       measurements, valid_count, &estimate, solver_flags);
    if (ret < 0) {
        LOG_WRN("Least squares failed: %d", ret);
        return ret;
    }

    position->x = estimate.position.x;
    position->y = estimate.position.y;
    position->z = estimate.position.z;



    /* Compute residual if requested */
    if (residual) {
        float sum_sq = 0.0f;
        for (size_t i = 0; i < valid_count; i++) {
            float dx = position->x - known_positions[i].position.x;
            float dy = position->y - known_positions[i].position.y;
            float dz = position->z - known_positions[i].position.z;
            float computed_dist = sqrtf(dx*dx + dy*dy + dz*dz);
            float measured_dist;
            if (node_table_get_filtered_distance_m(anchors[i].node_id, &measured_dist) != 0) {
                measured_dist = 0.0f;
            }
            float diff = computed_dist - measured_dist;
            sum_sq += diff * diff;
        }
        *residual = sqrtf(sum_sq / valid_count);
    }

    return 0;
}

/**
 * @brief Queue position payload for next ranging round
 *
 * @return 0 on success, negative errno on failure
 */
static int queue_tx_payload(void)
{
    uint8_t payload[POSITION_PAYLOAD_MAX_SIZE];
    int len;

    /* Build position payload based on our mode */
    len = position_payload_build(payload, sizeof(payload));
    if (len < 0) {
        LOG_ERR("Failed to build position payload: %d", len);
        return len;
    }

    LOG_DBG("TX len=%d", len);
    LOG_HEXDUMP_DBG(payload, len, "TX:");

    /* Prepare and queue broadcast frame */
    struct mac_queue_frame tx_frame;
    int ret = mac_queue_prepare_broadcast(&tx_frame, payload, len);
    if (ret < 0) {
        LOG_ERR("Failed to prepare broadcast frame: %d", ret);
        return ret;
    }

    ret = mac_queue_tx_push(&tx_frame, K_NO_WAIT);
    if (ret < 0) {
        LOG_WRN("Failed to queue TX: %d", ret);
        return ret;
    }

    return 0;
}

#if IS_ENABLED(CONFIG_CRAZYFLIE)
static void send_position_to_crazyflie(const struct vec3d_f *position)
{
    crazyflie_measurements_t *m;
    if (crazyflie_measurements_alloc(1, &m) < 0) {
        return;
    }

    crazyflie_measurements_add_position(get_node_addr(),
        (float[]){position->x, position->y, position->z}, m);

    send_measurement_to_crazyflie(m, K_NO_WAIT);
    crazyflie_measurements_free(m);
}
#endif

void ls_position_block_handler(uint64_t rtc_event_time, void *user_data)
{
    struct ls_position_block_config *config = (struct ls_position_block_config *)user_data;
    int processed;
    node_position_mode_t mode = get_node_position_mode();

    LOG_INF("LS block");

    /* Step 1: Process received position payloads */
    processed = process_rx_payloads(rtc_event_time);

    /* Step 2: Compute position if we're a mobile node, otherwise just log status */
    if (mode == NODE_POSITION_MODE_LEAST_SQUARES) {
        struct vec3d_f position;
        float residual = 0.0f;

        int ret = compute_position(config, rtc_event_time, &position, &residual);
        if (ret == 0) {
            /* Update node's own position so it gets advertised.
             * Use volatile version to avoid NVS writes on every update. */
            struct node_config_position node_pos = {
                .x = (double)position.x,
                .y = (double)position.y,
                .z = (double)position.z
            };
            set_node_position_volatile(&node_pos);

            /* Output JSON log - position in mm */
            printk("{\"event\": \"ls_position\", \"node_id\": \"0x%04x\", \"rtc\": %llu, "
                   "\"mode\": \"mobile\", \"x\": %d, \"y\": %d, \"z\": %d, "
                   "\"residual\": %d, \"rx_payloads\": %d}\n",
                   get_node_addr(), rtc_event_time,
                   (int)(position.x * 1000), (int)(position.y * 1000), (int)(position.z * 1000),
                   (int)(residual * 1000), processed);

            /* BLE position streaming is handled via callback registered with node.c
             * when set_node_position_volatile() is called above */

            /* Invoke callback if configured */
            if (config && config->position_cb) {
                config->position_cb(&position, residual, config->cb_user_data);
            }

#if IS_ENABLED(CONFIG_CRAZYFLIE)
            send_position_to_crazyflie(&position);
#endif
        } else if (ret == -EAGAIN) {
            /* Not enough anchors - log status */
            printk("{\"event\": \"ls_position\", \"node_id\": \"0x%04x\", \"rtc\": %llu, "
                   "\"mode\": \"mobile\", \"error\": \"not_enough_anchors\", \"rx_payloads\": %d}\n",
                   get_node_addr(), rtc_event_time, processed);
        } else {
            /* Solver failed */
            printk("{\"event\": \"ls_position\", \"node_id\": \"0x%04x\", \"rtc\": %llu, "
                   "\"mode\": \"mobile\", \"error\": \"solver_failed\", \"code\": %d}\n",
                   get_node_addr(), rtc_event_time, ret);
        }
    } else {
        /* Static/anchor node - just log that block ran */
        printk("{\"event\": \"ls_position\", \"node_id\": \"0x%04x\", \"rtc\": %llu, "
               "\"mode\": \"anchor\", \"rx_payloads\": %d}\n",
               get_node_addr(), rtc_event_time, processed);
    }

    /* Step 3: Queue our position payload for next round */
    queue_tx_payload();
}
