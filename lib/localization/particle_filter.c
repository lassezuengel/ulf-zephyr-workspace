/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Particle filter implementation for embedded systems
 *
 * Uses Zephyr's random number generator and avoids external dependencies
 * like GSL. Implements Box-Muller transform for Gaussian sampling.
 */

#include <app/lib/localization/particle_filter.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <app/lib/system/block_heap.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(particle_filter, LOG_LEVEL_INF);

/* Pi constant */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief Generate uniform random float in [0, 1)
 */
static float rand_uniform(void)
{
    uint32_t r = sys_rand32_get();
    return (float)r / (float)UINT32_MAX;
}

/**
 * @brief Generate Gaussian random number using Box-Muller transform
 *
 * @param mean Mean of the distribution
 * @param std_dev Standard deviation
 * @return Random sample from N(mean, std_dev^2)
 */
static float rand_gaussian(float mean, float std_dev)
{
    /* Box-Muller transform */
    float u1, u2;

    /* Avoid log(0) */
    do {
        u1 = rand_uniform();
    } while (u1 < 1e-10f);

    u2 = rand_uniform();

    float z = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    return mean + std_dev * z;
}

void pf_init_uniform(struct particle *particles, size_t count,
                     const struct vec3d_f *bounds_min,
                     const struct vec3d_f *bounds_max)
{
    if (!particles || count == 0 || !bounds_min || !bounds_max) {
        return;
    }

    float dx = bounds_max->x - bounds_min->x;
    float dy = bounds_max->y - bounds_min->y;
    float dz = bounds_max->z - bounds_min->z;
    float w = 1.0f / (float)count;

    for (size_t i = 0; i < count; i++) {
        particles[i].x = bounds_min->x + rand_uniform() * dx;
        particles[i].y = bounds_min->y + rand_uniform() * dy;
        particles[i].z = bounds_min->z + rand_uniform() * dz;
        particles[i].weight = w;
    }

    LOG_DBG("Initialized %zu particles uniformly in [%.2f,%.2f]x[%.2f,%.2f]x[%.2f,%.2f]",
            count, (double)bounds_min->x, (double)bounds_max->x,
            (double)bounds_min->y, (double)bounds_max->y,
            (double)bounds_min->z, (double)bounds_max->z);
}

void pf_init_gaussian(struct particle *particles, size_t count,
                      const struct vec3d_f *center, float std_dev)
{
    if (!particles || count == 0 || !center) {
        return;
    }

    float w = 1.0f / (float)count;

    for (size_t i = 0; i < count; i++) {
        particles[i].x = rand_gaussian(center->x, std_dev);
        particles[i].y = rand_gaussian(center->y, std_dev);
        particles[i].z = rand_gaussian(center->z, std_dev);
        particles[i].weight = w;
    }

    LOG_DBG("Initialized %zu particles Gaussian around (%.2f,%.2f,%.2f) std=%.2f",
            count, (double)center->x, (double)center->y, (double)center->z,
            (double)std_dev);
}

void pf_init_anchor(struct particle *particle, const struct vec3d_f *position)
{
    if (!particle || !position) {
        return;
    }

    particle->x = position->x;
    particle->y = position->y;
    particle->z = position->z;
    particle->weight = 1.0f;

    LOG_DBG("Initialized anchor particle at (%.2f,%.2f,%.2f)",
            (double)position->x, (double)position->y, (double)position->z);
}

void pf_predict(struct particle *particles, size_t count,
                float process_noise_std)
{
    if (!particles || count == 0 || process_noise_std <= 0.0f) {
        return;
    }

    for (size_t i = 0; i < count; i++) {
        particles[i].x += rand_gaussian(0.0f, process_noise_std);
        particles[i].y += rand_gaussian(0.0f, process_noise_std);
        particles[i].z += rand_gaussian(0.0f, process_noise_std);
    }
}

void pf_update_distance(struct particle *particles, size_t count,
                        const struct vec3d_f *anchor_pos,
                        float measured_distance,
                        float measurement_variance)
{
    if (!particles || count == 0 || !anchor_pos || measurement_variance <= 0.0f) {
        return;
    }

    float inv_2var = -0.5f / measurement_variance;

    for (size_t i = 0; i < count; i++) {
        /* Compute expected distance from particle to anchor */
        float dx = particles[i].x - anchor_pos->x;
        float dy = particles[i].y - anchor_pos->y;
        float dz = particles[i].z - anchor_pos->z;
        float expected_dist = sqrtf(dx*dx + dy*dy + dz*dz);

        /* Compute innovation (measurement - expected) */
        float error = measured_distance - expected_dist;

        /* Gaussian likelihood: exp(-0.5 * error^2 / variance) */
        float likelihood = expf(inv_2var * error * error);

        /* Penalize particles below ground (z < 0) - physically implausible */
        if (particles[i].z < 0.0f) {
            /* Exponential penalty: deeper below ground = stronger penalty */
            /* At z=-0.1m: factor ~0.37, at z=-0.5m: factor ~0.007 */
            likelihood *= expf(particles[i].z * 10.0f);
        }

        /* Update weight (multiplicative for sequential updates) */
        particles[i].weight *= likelihood;
    }
}

void pf_normalize_weights(struct particle *particles, size_t count)
{
    if (!particles || count == 0) {
        return;
    }

    /* Sum weights */
    float total = 0.0f;
    for (size_t i = 0; i < count; i++) {
        total += particles[i].weight;
    }

    /* Avoid division by zero */
    if (total < 1e-30f) {
        /* All weights are essentially zero - reset to uniform */
        float w = 1.0f / (float)count;
        for (size_t i = 0; i < count; i++) {
            particles[i].weight = w;
        }
        LOG_WRN("Weight underflow, reset to uniform");
        return;
    }

    /* Normalize */
    float inv_total = 1.0f / total;
    for (size_t i = 0; i < count; i++) {
        particles[i].weight *= inv_total;
    }
}

void pf_resample(struct particle *particles, size_t count)
{
    if (!particles || count == 0) {
        return;
    }

    /* Allocate scratch buffer for resampling */
    struct particle *scratch = block_malloc(count * sizeof(struct particle));
    if (!scratch) {
        LOG_ERR("Failed to allocate resample scratch buffer for %zu particles", count);
        return;
    }

    /* Low-variance (systematic) resampling */
    float r = rand_uniform() / (float)count;
    float c = particles[0].weight;
    size_t j = 0;

    for (size_t i = 0; i < count; i++) {
        float u = r + (float)i / (float)count;

        while (u > c && j < count - 1) {
            j++;
            c += particles[j].weight;
        }

        /* Copy particle j to scratch[i] */
        scratch[i] = particles[j];
        scratch[i].weight = 1.0f / (float)count;
    }

    /* Copy back to original array */
    memcpy(particles, scratch, count * sizeof(struct particle));

    block_free(scratch);
}

float pf_effective_sample_size(const struct particle *particles, size_t count)
{
    if (!particles || count == 0) {
        return 0.0f;
    }

    float sum_sq = 0.0f;
    for (size_t i = 0; i < count; i++) {
        sum_sq += particles[i].weight * particles[i].weight;
    }

    if (sum_sq < 1e-30f) {
        return (float)count; /* Uniform weights */
    }

    return 1.0f / sum_sq;
}

struct vec3d_f pf_mean_position(const struct particle *particles, size_t count)
{
    struct vec3d_f mean = {0.0f, 0.0f, 0.0f};

    if (!particles || count == 0) {
        return mean;
    }

    for (size_t i = 0; i < count; i++) {
        mean.x += particles[i].weight * particles[i].x;
        mean.y += particles[i].weight * particles[i].y;
        mean.z += particles[i].weight * particles[i].z;
    }

    return mean;
}

float pf_position_variance(const struct particle *particles, size_t count)
{
    if (!particles || count == 0) {
        return 0.0f;
    }

    /* First compute mean */
    struct vec3d_f mean = pf_mean_position(particles, count);

    /* Then compute weighted variance in each dimension */
    float var_x = 0.0f, var_y = 0.0f, var_z = 0.0f;

    for (size_t i = 0; i < count; i++) {
        float dx = particles[i].x - mean.x;
        float dy = particles[i].y - mean.y;
        float dz = particles[i].z - mean.z;

        var_x += particles[i].weight * dx * dx;
        var_y += particles[i].weight * dy * dy;
        var_z += particles[i].weight * dz * dz;
    }

    /* Return trace of covariance matrix (sum of variances) */
    return var_x + var_y + var_z;
}
