/*
 * Copyright (c) 2024 SynchroFly Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dedicated heap for transient per-block allocations (digest, schedule,
 * measurements, swarm ranging). Separated from the system heap to prevent
 * fragmentation caused by interleaving short-lived block allocations with
 * long-lived time series and node table entries.
 */

#ifndef SYNCHROFLY_BLOCK_HEAP_H
#define SYNCHROFLY_BLOCK_HEAP_H

#include <stddef.h>

void *block_malloc(size_t size);
void  block_free(void *ptr);

/**
 * @brief Get the block heap for runtime stats queries.
 * Use with sys_heap_runtime_stats_get(&block_heap_get()->heap, &stats).
 */
struct k_heap *block_heap_get(void);

#endif /* SYNCHROFLY_BLOCK_HEAP_H */
