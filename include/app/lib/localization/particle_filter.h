/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Particle filter for position estimation
 *
 * Implements a Sequential Monte Carlo (particle filter) approach for
 * localization from distance measurements to anchor nodes.
 */

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <stdint.h>
#include <stddef.h>
#include <app/lib/localization/location.h>

/**
 * @brief Single particle: 3D position + weight (16 bytes)
 */
struct particle {
    float x;       /* X coordinate in meters */
    float y;       /* Y coordinate in meters */
    float z;       /* Z coordinate in meters */
    float weight;  /* Particle weight (normalized to sum to 1) */
};

/**
 * @brief Initialize particle filter with uniform distribution in bounds
 *
 * Distributes particles uniformly within a 3D bounding box.
 * All particles receive equal weight (1/count).
 *
 * @param particles Output particle array
 * @param count Number of particles
 * @param bounds_min Minimum corner of initial volume (meters)
 * @param bounds_max Maximum corner of initial volume (meters)
 */
void pf_init_uniform(struct particle *particles, size_t count,
                     const struct vec3d_f *bounds_min,
                     const struct vec3d_f *bounds_max);

/**
 * @brief Initialize particles around a point with Gaussian spread
 *
 * Distributes particles according to a 3D Gaussian distribution
 * centered at the given point. All particles receive equal weight.
 *
 * @param particles Output particle array
 * @param count Number of particles
 * @param center Initial position estimate (meters)
 * @param std_dev Standard deviation for spread (meters)
 */
void pf_init_gaussian(struct particle *particles, size_t count,
                      const struct vec3d_f *center, float std_dev);

/**
 * @brief Initialize single anchor particle (for static nodes)
 *
 * Creates a degenerate particle set with one particle at the anchor's
 * known position with weight 1.0. Used by anchors which have no uncertainty.
 *
 * @param particle Single particle to initialize
 * @param position Known anchor position (meters)
 */
void pf_init_anchor(struct particle *particle, const struct vec3d_f *position);

/**
 * @brief Prediction step - add process noise to particles
 *
 * Applies a random walk motion model by adding Gaussian noise to each
 * particle's position. Weights are unchanged.
 *
 * @param particles Particle array (modified in place)
 * @param count Number of particles
 * @param process_noise_std Standard deviation of motion noise (meters)
 */
void pf_predict(struct particle *particles, size_t count,
                float process_noise_std);

/**
 * @brief Measurement update - weight particles by distance measurement likelihood
 *
 * Updates particle weights based on how well each particle's position
 * explains the measured distance to an anchor. Uses Gaussian likelihood:
 *   p(z|x) = exp(-0.5 * (d_measured - d_expected)^2 / variance)
 *
 * Weights are multiplied (not replaced) to allow sequential updates
 * from multiple anchors before normalization.
 *
 * @param particles Particle array (weights modified in place)
 * @param count Number of particles
 * @param anchor_pos Known anchor position (meters)
 * @param measured_distance Measured distance to anchor (meters)
 * @param measurement_variance Measurement noise variance (sigma^2, meters^2)
 */
void pf_update_distance(struct particle *particles, size_t count,
                        const struct vec3d_f *anchor_pos,
                        float measured_distance,
                        float measurement_variance);

/**
 * @brief Normalize particle weights to sum to 1
 *
 * @param particles Particle array (weights modified in place)
 * @param count Number of particles
 */
void pf_normalize_weights(struct particle *particles, size_t count);

/**
 * @brief Low-variance resampling (systematic resampling)
 *
 * Resamples particles according to their weights using systematic
 * resampling, which has lower variance than multinomial resampling.
 * After resampling, all particles have equal weight (1/count).
 *
 * @param particles Particle array (modified in place)
 * @param count Number of particles
 */
void pf_resample(struct particle *particles, size_t count);

/**
 * @brief Compute effective sample size (ESS)
 *
 * ESS = 1 / sum(w_i^2) indicates particle degeneracy.
 * - ESS = count: all particles have equal weight (ideal)
 * - ESS = 1: one particle has all weight (fully degenerate)
 *
 * Resample when ESS < count/2 to maintain diversity.
 *
 * @param particles Particle array
 * @param count Number of particles
 * @return ESS value (1 to count)
 */
float pf_effective_sample_size(const struct particle *particles, size_t count);

/**
 * @brief Compute weighted mean position
 *
 * @param particles Particle array
 * @param count Number of particles
 * @return Weighted mean position (meters)
 */
struct vec3d_f pf_mean_position(const struct particle *particles, size_t count);

/**
 * @brief Compute position variance (trace of covariance matrix)
 *
 * Returns sum of variances in x, y, z dimensions, which indicates
 * the overall uncertainty in the position estimate.
 *
 * @param particles Particle array
 * @param count Number of particles
 * @return Position variance (meters^2)
 */
float pf_position_variance(const struct particle *particles, size_t count);

#endif /* PARTICLE_FILTER_H */
