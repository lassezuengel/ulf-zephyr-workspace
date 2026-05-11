/*
 * Copyright (c) 2026 SynchroFly
 * SPDX-License-Identifier: Apache-2.0
 *
 * Polynomial-based distance correction -- storage, evaluation, persistence.
 */

#include "poly_correction.h"

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(poly_correction, LOG_LEVEL_INF);

/* ------------------------------------------------------------------ */
/* Static storage                                                      */
/* ------------------------------------------------------------------ */

static struct correction_poly global_poly;
static struct per_node_correction node_polys[CONFIG_POLY_CORRECTION_MAX_NODES];
static size_t node_poly_count;

/* ------------------------------------------------------------------ */
/* Evaluation (Horner's method)                                        */
/* ------------------------------------------------------------------ */

/**
 * Smoothstep: 0 at edge0, 1 at edge1, smooth Hermite in between.
 * Clamped to [0,1] outside the range.
 */
static float smoothstep(float edge0, float edge1, float x)
{
	if (edge1 <= edge0) {
		return 1.0f;
	}
	float t = (x - edge0) / (edge1 - edge0);
	if (t < 0.0f) {
		t = 0.0f;
	} else if (t > 1.0f) {
		t = 1.0f;
	}
	return t * t * (3.0f - 2.0f * t);
}

float poly_correction_eval(const struct correction_poly *poly, float d)
{
	if (!poly) {
		return 0.0f;
	}

	/* Evaluate the raw polynomial via Horner's method */
	float result;
	if (poly->degree == 0) {
		result = poly->coeffs[0];
	} else {
		result = poly->coeffs[0];
		for (int i = 1; i <= (int)poly->degree; i++) {
			result = result * d + poly->coeffs[i];
		}
	}

	/* Apply range blending if a valid range is set (d_min < d_max) */
	float range = poly->d_max - poly->d_min;
	if (range > 0.0f) {
		float margin = range * POLY_CORRECTION_BLEND_FRAC;

		/* Weight: 1.0 inside [d_min, d_max], fades to 0 outside */
		float w_lo = smoothstep(poly->d_min - margin, poly->d_min, d);
		float w_hi = smoothstep(poly->d_max + margin, poly->d_max, d);
		result *= w_lo * w_hi;
	}

	return result;
}

/* ------------------------------------------------------------------ */
/* Global polynomial                                                   */
/* ------------------------------------------------------------------ */

int poly_correction_set_global(const struct correction_poly *poly)
{
	if (!poly || poly->degree > POLY_CORRECTION_MAX_DEGREE) {
		return -EINVAL;
	}

	memcpy(&global_poly, poly, sizeof(global_poly));

	int ret = settings_save_one("polycorr/global", &global_poly,
				    sizeof(global_poly));
	if (ret < 0) {
		LOG_ERR("Failed to persist global poly: %d", ret);
	} else {
		LOG_INF("Global correction poly set: degree=%u", poly->degree);
	}
	return ret;
}

const struct correction_poly *poly_correction_get_global(void)
{
	return &global_poly;
}

/* ------------------------------------------------------------------ */
/* Per-node polynomials                                                */
/* ------------------------------------------------------------------ */

int poly_correction_set_node(uint16_t node_id, uint8_t channel,
			     const struct correction_poly *poly)
{
	if (!poly || poly->degree > POLY_CORRECTION_MAX_DEGREE) {
		return -EINVAL;
	}

	/* Check if entry already exists for (node, channel) */
	for (size_t i = 0; i < node_poly_count; i++) {
		if (node_polys[i].node_id == node_id &&
		    node_polys[i].channel == channel) {
			memcpy(&node_polys[i].poly, poly, sizeof(*poly));
			goto persist;
		}
	}

	/* Add new entry */
	if (node_poly_count >= CONFIG_POLY_CORRECTION_MAX_NODES) {
		LOG_ERR("Per-node poly table full (%zu/%d)",
			node_poly_count, CONFIG_POLY_CORRECTION_MAX_NODES);
		return -ENOMEM;
	}

	node_polys[node_poly_count].node_id = node_id;
	node_polys[node_poly_count].channel = channel;
	memcpy(&node_polys[node_poly_count].poly, poly, sizeof(*poly));
	node_poly_count++;

persist:
	LOG_INF("Per-node correction set: node=0x%04x ch%u, degree=%u",
		node_id, channel, poly->degree);

	/* Persist count and all entries */
	uint8_t count = (uint8_t)node_poly_count;
	settings_save_one("polycorr/count", &count, sizeof(count));

	for (size_t i = 0; i < node_poly_count; i++) {
		char key[32];
		snprintf(key, sizeof(key), "polycorr/n%zu", i);
		settings_save_one(key, &node_polys[i], sizeof(node_polys[i]));
	}

	return 0;
}

const struct correction_poly *poly_correction_get_node(uint16_t node_id,
						       uint8_t channel)
{
	for (size_t i = 0; i < node_poly_count; i++) {
		if (node_polys[i].node_id == node_id &&
		    node_polys[i].channel == channel) {
			return &node_polys[i].poly;
		}
	}
	return NULL;
}

int poly_correction_remove_node(uint16_t node_id, uint8_t channel)
{
	for (size_t i = 0; i < node_poly_count; i++) {
		if (node_polys[i].node_id == node_id &&
		    node_polys[i].channel == channel) {
			/* Shift remaining entries down */
			if (i < node_poly_count - 1) {
				memmove(&node_polys[i], &node_polys[i + 1],
					(node_poly_count - 1 - i) *
					sizeof(node_polys[0]));
			}
			node_poly_count--;

			/* Persist updated state */
			uint8_t count = (uint8_t)node_poly_count;
			settings_save_one("polycorr/count", &count,
					  sizeof(count));
			return 0;
		}
	}
	return -ENOENT;
}

void poly_correction_clear_nodes(void)
{
	node_poly_count = 0;
	memset(node_polys, 0, sizeof(node_polys));

	uint8_t count = 0;
	settings_save_one("polycorr/count", &count, sizeof(count));
	LOG_INF("Per-node correction table cleared");
}

size_t poly_correction_node_count(void)
{
	return node_poly_count;
}

/* ------------------------------------------------------------------ */
/* Settings persistence                                                */
/* ------------------------------------------------------------------ */

static int settings_set(const char *name, size_t len,
			settings_read_cb read_cb, void *cb_arg)
{
	const char *next;
	int rc;

	if (settings_name_steq(name, "global", &next) && !next) {
		if (len != sizeof(global_poly)) {
			return -EINVAL;
		}
		rc = read_cb(cb_arg, &global_poly, sizeof(global_poly));
		if (rc < 0) {
			return rc;
		}
		LOG_INF("Loaded global poly: degree=%u", global_poly.degree);
		return 0;
	}

	if (settings_name_steq(name, "count", &next) && !next) {
		uint8_t count;
		if (len != sizeof(count)) {
			return -EINVAL;
		}
		rc = read_cb(cb_arg, &count, sizeof(count));
		if (rc < 0) {
			return rc;
		}
		node_poly_count = count;
		LOG_INF("Loaded per-node poly count: %u", count);
		return 0;
	}

	/* Per-node entries: "n0", "n1", ... */
	if (name[0] == 'n' && name[1] >= '0' && name[1] <= '9') {
		/* Parse index from name */
		size_t idx = 0;
		for (const char *p = name + 1; *p >= '0' && *p <= '9'; p++) {
			idx = idx * 10 + (*p - '0');
		}
		if (idx >= CONFIG_POLY_CORRECTION_MAX_NODES) {
			return -EINVAL;
		}
		if (len != sizeof(node_polys[0])) {
			return -EINVAL;
		}
		rc = read_cb(cb_arg, &node_polys[idx], sizeof(node_polys[0]));
		if (rc < 0) {
			return rc;
		}
		LOG_INF("Loaded per-node poly[%zu]: node=0x%04x ch%u, degree=%u",
			idx, node_polys[idx].node_id,
			node_polys[idx].channel,
			node_polys[idx].poly.degree);
		return 0;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(polycorr, "polycorr", NULL, settings_set,
			       NULL, NULL);
