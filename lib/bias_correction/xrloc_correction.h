/*
 * Copyright (c) 2026 SynchroFly
 * SPDX-License-Identifier: Apache-2.0
 *
 * XRLoc-style distance correction for TWR ranging.
 *
 * Implements the exact phase bias model from:
 *   Arun et al., "XRLoc", SenSys '23, Section 4.2
 *
 * Phase bias model per receiver:
 *   Phi_measured = (2*pi*d / lambda) + alpha + beta * d^gamma
 *
 * The correction converts phase bias to distance:
 *   delta_d = -(alpha + beta * d^gamma) / 1000 * lambda / (2*pi)
 *
 * where alpha, beta are in milliradians, gamma is dimensionless,
 * and lambda is the half-wavelength c/(2*f) in meters.
 *
 * Correction is additive: corrected_distance = raw_distance + delta_d
 */

#ifndef XRLOC_CORRECTION_H
#define XRLOC_CORRECTION_H

#include <stdint.h>
#include <stddef.h>

/**
 * XRLoc phase bias correction parameters.
 * Stored per node, evaluated at runtime with powf().
 */
struct xrloc_correction {
	float alpha;     /**< Constant phase bias (mrad) */
	float beta;      /**< Distance-dependent slope (mrad) */
	float gamma;     /**< Exponent for distance term */
	float lambda_m;  /**< Half-wavelength = c / (2*f) in meters */
	float d_min;     /**< Lower bound of valid range (meters) */
	float d_max;     /**< Upper bound of valid range (meters) */
};

/** Per-(node, channel) XRLoc correction entry. */
struct per_node_xrloc {
	uint16_t node_id;
	uint8_t channel;
	uint8_t _pad;
	struct xrloc_correction xrloc;
};

/**
 * Evaluate XRLoc correction at distance d (meters).
 * Returns the delta to ADD to the raw TWR distance.
 * Uses smoothstep blending at range boundaries (same as poly_correction).
 */
float xrloc_correction_eval(const struct xrloc_correction *x, float d);

/**
 * Evaluate raw phase bias at distance d (meters).
 * Returns bias in milliradians: alpha + beta * d^gamma
 * (with smoothstep blending at range boundaries).
 * Subtract this from the measured phase to get the corrected phase.
 */
float xrloc_correction_phase_bias_mrad(const struct xrloc_correction *x, float d);

/**
 * Set a per-(node, channel) XRLoc correction.
 * Overwrites existing entry for (node_id, channel), or creates a new one.
 * Returns 0 on success, -ENOMEM if table full.
 */
int xrloc_correction_set_node(uint16_t node_id, uint8_t channel,
			      const struct xrloc_correction *x);

/**
 * Get XRLoc correction for a specific (node, channel).
 * Returns NULL if no entry exists.
 */
const struct xrloc_correction *xrloc_correction_get_node(uint16_t node_id,
							 uint8_t channel);

/** Remove a per-(node, channel) XRLoc correction entry. Returns 0 or -ENOENT. */
int xrloc_correction_remove_node(uint16_t node_id, uint8_t channel);

/** Clear all per-node XRLoc entries. */
void xrloc_correction_clear_nodes(void);

/** Get count of per-node XRLoc entries currently stored. */
size_t xrloc_correction_node_count(void);

#endif /* XRLOC_CORRECTION_H */
