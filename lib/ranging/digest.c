/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <app/lib/system/block_heap.h>

#include <app/lib/ranging/digest.h>

LOG_MODULE_REGISTER(ranging_digest);

int deca_ranging_digest_alloc(int frame_capacity, struct deca_ranging_digest **digest)
{
    if (!digest || frame_capacity <= 0) {
        return -EINVAL;
    }

    /* Allocate digest structure */
    struct deca_ranging_digest *d = block_malloc(sizeof(struct deca_ranging_digest));
    if (!d) {
        LOG_ERR("Failed to allocate digest structure");
        return -ENOMEM;
    }

    /* Allocate frame containers */
    d->frames = block_malloc(frame_capacity * sizeof(struct deca_ranging_frame_container));
    if (!d->frames) {
        block_free(d);
        LOG_ERR("Failed to allocate frame containers");
        return -ENOMEM;
    }

    /* Initialize containers and allocate frames */
    memset(d->frames, 0, frame_capacity * sizeof(struct deca_ranging_frame_container));

    /* Allocate a frame for each container */
    for (int i = 0; i < frame_capacity; i++) {
        d->frames[i].frame = block_malloc(sizeof(struct deca_ranging_frame));
        if (!d->frames[i].frame) {
            LOG_ERR("Failed to allocate frame %d", i);
            /* Clean up previously allocated frames */
            for (int j = 0; j < i; j++) {
                block_free(d->frames[j].frame);
            }
            block_free(d->frames);
            block_free(d);
            return -ENOMEM;
        }
        /* Initialize frame to zero */
        memset(d->frames[i].frame, 0, sizeof(struct deca_ranging_frame));
    }

    /* Initialize digest */
    d->length = 0;
    d->capacity = frame_capacity;

    *digest = d;
    return 0;
}

int deca_ranging_digest_free(struct deca_ranging_digest *digest)
{
    if (!digest) {
        return -EINVAL;
    }

    /* Free individual frames first */
    if (digest->frames && digest->capacity > 0) {
        for (int i = 0; i < digest->capacity; i++) {
            if (digest->frames[i].frame) {
                block_free(digest->frames[i].frame);
            }
        }
        /* Free frame containers */
        block_free(digest->frames);
    }

    /* Free digest structure */
    block_free(digest);

    return 0;
}

int deca_ranging_digest_clear(struct deca_ranging_digest *digest)
{
    if (!digest) {
        return -EINVAL;
    }

    /* Reset length but keep all allocated memory (containers and frames) */
    digest->length = 0;

    /* Optionally clear frame data */
    if (digest->frames && digest->capacity > 0) {
        for (int i = 0; i < digest->capacity; i++) {
            if (digest->frames[i].frame) {
                memset(digest->frames[i].frame, 0, sizeof(struct deca_ranging_frame));
            }
        }
    }

    return 0;
}
