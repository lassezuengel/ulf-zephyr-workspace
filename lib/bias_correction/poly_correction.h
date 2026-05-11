/*
 * Copyright (c) 2026 SynchroFly
 * SPDX-License-Identifier: Apache-2.0
 *
 * Polynomial-based distance correction for TWR ranging.
 *
 * Two modes:
 *   - Global polynomial: single correction applied to all nodes
 *   - Per-node polynomial: lookup table mapping node addresses to polynomials
 *
 * Correction is additive: corrected_distance = raw_distance + poly(raw_distance)
 * Coefficients are stored highest-power-first (numpy convention):
 *   coeffs[0]*x^N + coeffs[1]*x^(N-1) + ... + coeffs[N]
 */

#ifndef POLY_CORRECTION_H
#define POLY_CORRECTION_H

#include <stdint.h>
#include <stddef.h>

/** Maximum polynomial degree supported. Degree N has N+1 coefficients. */
#define POLY_CORRECTION_MAX_DEGREE  3
#define POLY_CORRECTION_MAX_COEFFS  (POLY_CORRECTION_MAX_DEGREE + 1)

/**
 * Correction polynomial.
 * coeffs[0]*x^degree + coeffs[1]*x^(degree-1) + ... + coeffs[degree]
 *
 * The polynomial is only valid within [d_min, d_max].  Outside that range
 * the correction delta is smoothly blended to zero over a transition zone
 * of POLY_CORRECTION_BLEND_FRAC * (d_max - d_min) on each side, so there
 * is no discontinuity at the boundary.
 */
#define POLY_CORRECTION_BLEND_FRAC 0.2f

struct correction_poly {
	float coeffs[POLY_CORRECTION_MAX_COEFFS];
	float d_min;       /* lower bound of valid range (meters) */
	float d_max;       /* upper bound of valid range (meters) */
	uint8_t degree;    /* 0 = constant, 1 = linear, 2 = quadratic, 3 = cubic */
	uint8_t _pad[3];
};

/** Per-(node, channel) correction entry. */
struct per_node_correction {
	uint16_t node_id;
	uint8_t channel;
	uint8_t _pad;
	struct correction_poly poly;
};

/**
 * Evaluate a correction polynomial at distance d (meters).
 * Returns the delta to ADD to the raw TWR distance.
 * Uses Horner's method for efficient evaluation.
 */
float poly_correction_eval(const struct correction_poly *poly, float d);

/**
 * Set the global correction polynomial.
 * Persists to NVS. Pass degree=0, coeffs[0]=0 to clear.
 */
int poly_correction_set_global(const struct correction_poly *poly);

/** Get the current global correction polynomial. */
const struct correction_poly *poly_correction_get_global(void);

/**
 * Set a per-(node, channel) correction polynomial.
 * Overwrites existing entry for (node_id, channel), or creates a new one.
 * Returns 0 on success, -ENOMEM if table full.
 */
int poly_correction_set_node(uint16_t node_id, uint8_t channel,
			     const struct correction_poly *poly);

/**
 * Get correction polynomial for a specific (node, channel).
 * Returns NULL if no entry exists.
 */
const struct correction_poly *poly_correction_get_node(uint16_t node_id,
						       uint8_t channel);

/** Remove a per-(node, channel) correction entry. Returns 0 or -ENOENT. */
int poly_correction_remove_node(uint16_t node_id, uint8_t channel);

/** Clear all per-node entries. */
void poly_correction_clear_nodes(void);

/** Get count of per-node entries currently stored. */
size_t poly_correction_node_count(void);

#endif /* POLY_CORRECTION_H */
