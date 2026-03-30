#ifndef SYNCHROFLY_BLOCKS_H
#define SYNCHROFLY_BLOCKS_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <app/lib/scheduling/lower/schedule_functions.h>
#include <app/lib/ranging/twr.h>
#include <app/lib/localization/location.h>

#include "mtm.h"
#include "glossy.h"

typedef void (*block_handler_t)(uint64_t event_time, void *user_data);

struct time_sync_check_config {
    bool isNetworkRoot;
};

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_LS_POSITION)
/**
 * @brief Least squares position block configuration
 */
struct ls_position_block_config {
    uint8_t min_anchors;      /* Minimum anchors required (default: 3) */
    uint16_t max_age_ms;      /* Maximum anchor measurement age (default: 2000) */
    void (*position_cb)(const struct vec3d_f *position, float residual, void *user_data);
    void *cb_user_data;
};

void ls_position_block_handler(uint64_t rtc_event_time, void *user_data);
#endif

#if IS_ENABLED(CONFIG_SYNCHROFLY_BLOCK_PF_POSITION)
/**
 * @brief Particle filter position block configuration
 *
 * For anchor nodes (STATIC mode): particle_count is ignored, always uses 1 particle.
 * For mobile nodes (PARTICLE_FILTER mode): uses particle_count particles for estimation.
 */
struct pf_position_block_config {
    uint16_t particle_count;      /* Number of particles for mobile nodes (default: 50) */
    float measurement_variance;   /* Measurement noise variance in m^2 (default: 0.04) */
    float process_noise_std;      /* Motion model noise std dev in m (default: 0.05) */
    bool send_particles;          /* true: TX particles, false: TX mean only (default: false) */
    uint8_t min_anchors;          /* Minimum anchors required (default: 3) */
    uint16_t max_age_ms;          /* Maximum anchor measurement age in ms (default: 2000) */
    void (*position_cb)(const struct vec3d_f *position, float variance, void *user_data);
    void *cb_user_data;
};

void pf_position_block_handler(uint64_t rtc_event_time, void *user_data);
#endif

void time_sync_check_block_handler(uint64_t rtc_event_time, void *user_data);
void glossy_block_handler(uint64_t rtc_event_time, void *user_data);
void mtm_block_handler(uint64_t rtc_event_time, void *user_data);
void mm_block_handler(uint64_t rtc_event_time, void *user_data); // currently just uses mtm_block_config
void mm_reference_block_handler(uint64_t rtc_event_time, void *user_data);

#endif
