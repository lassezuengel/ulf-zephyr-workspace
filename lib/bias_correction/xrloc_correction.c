/*
 * Copyright (c) 2026 SynchroFly
 * SPDX-License-Identifier: Apache-2.0
 *
 * XRLoc phase bias correction -- evaluation, storage, persistence.
 *
 * Model: delta_d = -(alpha + beta * powf(d, gamma)) / 1000 * lambda / (2*pi)
 */

#include "xrloc_correction.h"
#include "poly_correction.h"  /* for POLY_CORRECTION_BLEND_FRAC */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(xrloc_correction, LOG_LEVEL_INF);

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ------------------------------------------------------------------ */
/* Static storage                                                      */
/* ------------------------------------------------------------------ */

static struct per_node_xrloc node_xrlocs[CONFIG_POLY_CORRECTION_MAX_NODES];
static size_t node_xrloc_count;

/* ------------------------------------------------------------------ */
/* Evaluation                                                          */
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

float xrloc_correction_phase_bias_mrad(const struct xrloc_correction *x, float d)
{
	if (!x) {
		return 0.0f;
	}

	float bias_mrad = x->alpha + x->beta * powf(fmaxf(d, 1e-6f), x->gamma);

	/* Apply range blending if a valid range is set */
	float range = x->d_max - x->d_min;
	if (range > 0.0f) {
		float margin = range * POLY_CORRECTION_BLEND_FRAC;
		float w_lo = smoothstep(x->d_min - margin, x->d_min, d);
		float w_hi = smoothstep(x->d_max + margin, x->d_max, d);
		bias_mrad *= w_lo * w_hi;
	}

	return bias_mrad;
}

float xrloc_correction_eval(const struct xrloc_correction *x, float d)
{
	if (!x || x->lambda_m <= 0.0f) {
		return 0.0f;
	}

	float bias_mrad = xrloc_correction_phase_bias_mrad(x, d);

	/* Convert phase bias to distance correction:
	 * delta_d = -(bias_mrad / 1000) * lambda / (2*pi)
	 * Negative because positive phase bias = measured distance too long */
	return -(bias_mrad / 1000.0f) * x->lambda_m / (2.0f * M_PI);
}

/* ------------------------------------------------------------------ */
/* Per-node storage                                                    */
/* ------------------------------------------------------------------ */

int xrloc_correction_set_node(uint16_t node_id, uint8_t channel,
			      const struct xrloc_correction *x)
{
	if (!x) {
		return -EINVAL;
	}

	/* Check if entry already exists for (node, channel) */
	for (size_t i = 0; i < node_xrloc_count; i++) {
		if (node_xrlocs[i].node_id == node_id &&
		    node_xrlocs[i].channel == channel) {
			memcpy(&node_xrlocs[i].xrloc, x, sizeof(*x));
			goto persist;
		}
	}

	/* Add new entry */
	if (node_xrloc_count >= CONFIG_POLY_CORRECTION_MAX_NODES) {
		LOG_ERR("XRLoc table full (%zu/%d)",
			node_xrloc_count, CONFIG_POLY_CORRECTION_MAX_NODES);
		return -ENOMEM;
	}

	node_xrlocs[node_xrloc_count].node_id = node_id;
	node_xrlocs[node_xrloc_count].channel = channel;
	memcpy(&node_xrlocs[node_xrloc_count].xrloc, x, sizeof(*x));
	node_xrloc_count++;

persist:
	LOG_INF("XRLoc correction set: node=0x%04x ch%u, alpha=%.2f, beta=%.4f, "
		"gamma=%.3f, lambda=%.4fm",
		node_id, channel, (double)x->alpha, (double)x->beta,
		(double)x->gamma, (double)x->lambda_m);

	/* Persist count and all entries */
	uint8_t count = (uint8_t)node_xrloc_count;
	settings_save_one("xrloccorr/count", &count, sizeof(count));

	for (size_t i = 0; i < node_xrloc_count; i++) {
		char key[32];
		snprintf(key, sizeof(key), "xrloccorr/n%zu", i);
		settings_save_one(key, &node_xrlocs[i], sizeof(node_xrlocs[i]));
	}

	return 0;
}

const struct xrloc_correction *xrloc_correction_get_node(uint16_t node_id,
							 uint8_t channel)
{
	for (size_t i = 0; i < node_xrloc_count; i++) {
		if (node_xrlocs[i].node_id == node_id &&
		    node_xrlocs[i].channel == channel) {
			return &node_xrlocs[i].xrloc;
		}
	}
	return NULL;
}

int xrloc_correction_remove_node(uint16_t node_id, uint8_t channel)
{
	for (size_t i = 0; i < node_xrloc_count; i++) {
		if (node_xrlocs[i].node_id == node_id &&
		    node_xrlocs[i].channel == channel) {
			if (i < node_xrloc_count - 1) {
				memmove(&node_xrlocs[i], &node_xrlocs[i + 1],
					(node_xrloc_count - 1 - i) *
					sizeof(node_xrlocs[0]));
			}
			node_xrloc_count--;

			uint8_t count = (uint8_t)node_xrloc_count;
			settings_save_one("xrloccorr/count", &count,
					  sizeof(count));
			return 0;
		}
	}
	return -ENOENT;
}

void xrloc_correction_clear_nodes(void)
{
	node_xrloc_count = 0;
	memset(node_xrlocs, 0, sizeof(node_xrlocs));

	uint8_t count = 0;
	settings_save_one("xrloccorr/count", &count, sizeof(count));
	LOG_INF("XRLoc correction table cleared");
}

size_t xrloc_correction_node_count(void)
{
	return node_xrloc_count;
}

/* ------------------------------------------------------------------ */
/* Settings persistence                                                */
/* ------------------------------------------------------------------ */

static int settings_set(const char *name, size_t len,
			settings_read_cb read_cb, void *cb_arg)
{
	const char *next;
	int rc;

	if (settings_name_steq(name, "count", &next) && !next) {
		uint8_t count;
		if (len != sizeof(count)) {
			return -EINVAL;
		}
		rc = read_cb(cb_arg, &count, sizeof(count));
		if (rc < 0) {
			return rc;
		}
		node_xrloc_count = count;
		LOG_INF("Loaded XRLoc correction count: %u", count);
		return 0;
	}

	/* Per-node entries: "n0", "n1", ... */
	if (name[0] == 'n' && name[1] >= '0' && name[1] <= '9') {
		size_t idx = 0;
		for (const char *p = name + 1; *p >= '0' && *p <= '9'; p++) {
			idx = idx * 10 + (*p - '0');
		}
		if (idx >= CONFIG_POLY_CORRECTION_MAX_NODES) {
			return -EINVAL;
		}
		if (len != sizeof(node_xrlocs[0])) {
			return -EINVAL;
		}
		rc = read_cb(cb_arg, &node_xrlocs[idx], sizeof(node_xrlocs[0]));
		if (rc < 0) {
			return rc;
		}
		LOG_INF("Loaded XRLoc[%zu]: node=0x%04x ch%u, alpha=%.2f, beta=%.4f, gamma=%.3f",
			idx, node_xrlocs[idx].node_id,
			node_xrlocs[idx].channel,
			(double)node_xrlocs[idx].xrloc.alpha,
			(double)node_xrlocs[idx].xrloc.beta,
			(double)node_xrlocs[idx].xrloc.gamma);
		return 0;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(xrloccorr, "xrloccorr", NULL, settings_set,
			       NULL, NULL);
