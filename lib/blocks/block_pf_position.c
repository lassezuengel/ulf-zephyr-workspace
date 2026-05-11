/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Particle Filter Position Block
 *
 * This block runs after ranging blocks to:
 * 1. Process received position payloads from the MAC RX queue
 * 2. Update neighbor table with anchor positions
 * 3. Compute position using particle filter (if mobile node)
 * 4. Queue position payload for next ranging round
 *
 * Anchor nodes (STATIC mode) simply broadcast their known position as a
 * single particle with weight 1.0. Mobile nodes run the full particle
 * filter cycle with configurable particle count.
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
#include <app/lib/localization/particle_filter.h>
#include <app/lib/localization/location.h>
#include <app/lib/node_table/node_table.h>
#include <app/lib/timesync/time_synchronization.h>

LOG_MODULE_REGISTER(block_pf_position, LOG_LEVEL_INF);

/* Maximum anchors to use for position estimation */
#define PF_MAX_ANCHORS 16

/* Default configuration values */
#define PF_DEFAULT_PARTICLE_COUNT    50
#define PF_DEFAULT_MIN_ANCHORS       3
#define PF_DEFAULT_MAX_AGE_MS        2000
#define PF_DEFAULT_MEAS_VARIANCE     0.04f   /* 0.2m std dev squared */
#define PF_DEFAULT_PROCESS_NOISE     0.05f   /* 5cm motion noise */

/* Particle buffer - allocated on first use */
static struct particle *particles = NULL;
static size_t particle_count = 0;
static size_t particle_capacity = 0;
static bool pf_initialized = false;

/**
 * @brief Process all received position payloads from MAC RX queue
 *
 * Identical to LS block - extracts anchor positions and updates node table.
 *
 * @param rtc Current RTC timestamp
 * @return Number of payloads processed
 */
static int process_rx_payloads(uint64_t rtc)
{
    struct mac_queue_frame rx_frame;
    uint8_t payload_buf[64];
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

        LOG_DBG("RX from 0x%04x len=%u", sender_id, (unsigned)payload_len);

        /* Process the position payload */
        int ret = position_payload_process(sender_id, payload_buf, payload_len, rtc);
        if (ret == 0) {
            count++;
        }
    }
    return count;
}

/**
 * @brief Get valid anchor entries for particle filter update
 *
 * @param config Block configuration
 * @param rtc Current RTC timestamp
 * @param anchors Output buffer for anchor entries
 * @param max_anchors Maximum anchors to return
 * @return Number of valid anchors found
 */
static size_t get_valid_anchors(const struct pf_position_block_config *config,
                                uint64_t rtc,
                                struct node_entry *anchors,
                                size_t max_anchors)
{
    /* Get all anchors from neighbor table */
    size_t anchor_count = node_table_get_anchors(anchors, max_anchors);

    /* Filter by age */
    uint16_t max_age_ms = config ? config->max_age_ms : PF_DEFAULT_MAX_AGE_MS;
    uint64_t max_age_ticks = k_ms_to_ticks_floor64(max_age_ms);

    size_t valid_count = 0;
    for (size_t i = 0; i < anchor_count; i++) {
        uint64_t age = rtc - anchors[i].last_seen_rtc;

        /* Check if anchor has valid distance measurement */
        float dist_m;
        bool has_distance = (node_table_get_filtered_distance_m(anchors[i].node_id, &dist_m) == 0 && dist_m > 0.0f);

        if (age <= max_age_ticks && has_distance) {
            /* Move valid anchor to front of array */
            if (valid_count != i) {
                anchors[valid_count] = anchors[i];
            }
            valid_count++;
        }
    }

    return valid_count;
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

void pf_position_block_handler(uint64_t rtc_event_time, void *user_data)
{
    struct pf_position_block_config *config = (struct pf_position_block_config *)user_data;
    node_position_mode_t mode = get_node_position_mode();
    int processed;

    LOG_INF("PF block");

    /* Step 1: Process received position payloads */
    processed = process_rx_payloads(rtc_event_time);

    /* Step 2: Handle based on node mode */
    if (mode == NODE_POSITION_MODE_STATIC) {
        /*
         * ANCHOR NODE: No particle filter updates needed.
         * Just initialize single particle at our known position and transmit.
         */
        if (!pf_initialized) {
            /* Allocate single particle for anchor */
            if (!particles) {
                particles = k_malloc(sizeof(struct particle));
                if (!particles) {
                    LOG_ERR("Failed to allocate anchor particle");
                    return;
                }
                particle_capacity = 1;
            }

            struct node_config_position pos;
            if (get_node_config_position(&pos) == 0) {
                struct vec3d_f anchor_pos = {
                    .x = (float)pos.x,
                    .y = (float)pos.y,
                    .z = (float)pos.z
                };
                pf_init_anchor(&particles[0], &anchor_pos);
                particle_count = 1;
                pf_initialized = true;
                LOG_INF("Anchor particle initialized at (%.3f, %.3f, %.3f)",
                        (double)pos.x, (double)pos.y, (double)pos.z);
            } else {
                LOG_WRN("Anchor has no configured position");
            }
        }

        /* Log anchor status */
        printk("{\"event\": \"pf_position\", \"node_id\": \"0x%04x\", \"rtc\": %llu, "
               "\"mode\": \"anchor\", \"rx_payloads\": %d}\n",
               get_node_addr(), rtc_event_time, processed);

        /* Queue TX payload */
        queue_tx_payload();
        return;
    }

    /* MOBILE NODE: Full particle filter cycle */
    if (mode == NODE_POSITION_MODE_PARTICLE_FILTER) {
        /* Initialize particles if first run */
        if (!pf_initialized) {
            uint16_t target_count = config ? config->particle_count : PF_DEFAULT_PARTICLE_COUNT;
            size_t alloc_count = MIN(target_count, CONFIG_SYNCHROFLY_PF_MAX_PARTICLES);

            /* Allocate particle buffer */
            if (!particles || particle_capacity < alloc_count) {
                if (particles) {
                    k_free(particles);
                }
                particles = k_malloc(alloc_count * sizeof(struct particle));
                if (!particles) {
                    LOG_ERR("Failed to allocate %zu particles", alloc_count);
                    return;
                }
                particle_capacity = alloc_count;
            }
            particle_count = alloc_count;

            /* Initialize around origin with large spread - will converge with measurements */
            struct vec3d_f init_center = {0.0f, 0.0f, 0.0f};
            pf_init_gaussian(particles, particle_count, &init_center, 3.0f);
            pf_initialized = true;

            LOG_INF("Initialized %zu particles for mobile node", particle_count);
        }

        /* Get valid anchors */
        struct node_entry anchors[PF_MAX_ANCHORS];
        size_t anchor_count = get_valid_anchors(config, rtc_event_time, anchors, PF_MAX_ANCHORS);

        uint8_t min_anchors = config ? config->min_anchors : PF_DEFAULT_MIN_ANCHORS;

        if (anchor_count < min_anchors) {
            /* Not enough anchors - just log and TX */
            printk("{\"event\": \"pf_position\", \"node_id\": \"0x%04x\", \"rtc\": %llu, "
                   "\"mode\": \"mobile\", \"error\": \"not_enough_anchors\", "
                   "\"anchors\": %zu, \"rx_payloads\": %d}\n",
                   get_node_addr(), rtc_event_time, anchor_count, processed);

            queue_tx_payload();
            return;
        }

        /* Get configuration parameters */
        float process_noise = config ? config->process_noise_std : PF_DEFAULT_PROCESS_NOISE;
        float meas_variance = config ? config->measurement_variance : PF_DEFAULT_MEAS_VARIANCE;

        /* Step 3: Prediction - add motion noise */
        pf_predict(particles, particle_count, process_noise);

        /* Step 4: Measurement update for each anchor */
        for (size_t i = 0; i < anchor_count; i++) {
            float dist_m;
            if (node_table_get_filtered_distance_m(anchors[i].node_id, &dist_m) != 0) {
                continue; /* Skip if no valid distance */
            }
            struct vec3d_f anchor_pos = {
                .x = anchors[i].pos_x,
                .y = anchors[i].pos_y,
                .z = anchors[i].pos_z
            };

            pf_update_distance(particles, particle_count, &anchor_pos,
                               dist_m, meas_variance);
        }

        /* Step 5: Normalize weights */
        pf_normalize_weights(particles, particle_count);

        /* Step 6: Check ESS and resample if needed */
        float ess = pf_effective_sample_size(particles, particle_count);
        if (ess < (float)particle_count / 2.0f) {
            pf_resample(particles, particle_count);
            LOG_DBG("Resampled particles (ESS was %.1f)", (double)ess);
        }

        /* Step 7: Compute mean position and variance */
        struct vec3d_f mean = pf_mean_position(particles, particle_count);
        float variance = pf_position_variance(particles, particle_count);

        /* Update node's own position */
        struct node_config_position node_pos = {
            .x = (double)mean.x,
            .y = (double)mean.y,
            .z = (double)mean.z
        };
        set_node_position_volatile(&node_pos);

        /* Log position estimate */
        printk("{\"event\": \"pf_position\", \"node_id\": \"0x%04x\", \"rtc\": %llu, "
               "\"mode\": \"mobile\", \"x\": %d, \"y\": %d, \"z\": %d, "
               "\"var\": %d, \"ess\": %d, \"anchors\": %zu, \"rx_payloads\": %d}\n",
               get_node_addr(), rtc_event_time,
               (int)(mean.x * 1000), (int)(mean.y * 1000), (int)(mean.z * 1000),
               (int)(variance * 1000000), (int)ess, anchor_count, processed);

        /* Invoke callback if configured */
        if (config && config->position_cb) {
            config->position_cb(&mean, variance, config->cb_user_data);
        }
    } else {
        /* Unknown/unsupported mode for this block */
        printk("{\"event\": \"pf_position\", \"node_id\": \"0x%04x\", \"rtc\": %llu, "
               "\"mode\": \"unsupported\", \"position_mode\": %d}\n",
               get_node_addr(), rtc_event_time, mode);
    }

    /* Step 8: Queue TX payload */
    queue_tx_payload();
}
