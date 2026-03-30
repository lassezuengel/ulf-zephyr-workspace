/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_APP_LIB_RANGING_DIGEST_H_
#define ZEPHYR_INCLUDE_APP_LIB_RANGING_DIGEST_H_

#include <zephyr/kernel.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Allocate a ranging digest with capacity for frames
 *
 * @param frame_capacity Maximum number of frames this digest can hold
 * @param digest Pointer to store allocated digest
 * @return 0 on success, negative error code on failure
 */
int deca_ranging_digest_alloc(int frame_capacity, struct deca_ranging_digest **digest);

/**
 * @brief Free a ranging digest and all associated resources
 *
 * @param digest Digest to free
 * @return 0 on success, negative error code on failure
 */
int deca_ranging_digest_free(struct deca_ranging_digest *digest);

/**
 * @brief Clear all data from a digest (reuse without reallocation)
 *
 * @param digest Digest to clear
 * @return 0 on success, negative error code on failure
 */
int deca_ranging_digest_clear(struct deca_ranging_digest *digest);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_APP_LIB_RANGING_DIGEST_H_ */