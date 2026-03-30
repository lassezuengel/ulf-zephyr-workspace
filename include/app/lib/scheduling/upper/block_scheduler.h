#ifndef SYNCHROFLY_BLOCK_SCHEDULER_H
#define SYNCHROFLY_BLOCK_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#include <app/lib/blocks/blocks.h>

enum superframe_status {
    SUPERFRAME_STATUS_DISABLED,    // Node is disabled
    SUPERFRAME_STATUS_SUCCESS,     // Finished successfully
    SUPERFRAME_STATUS_ERROR        // Finished because of an error
};

typedef void (*superframe_callback_t)(enum superframe_status, void*);

struct block_config {
    block_handler_t block_handler;
    void *config;
    uint32_t duration_ms; // Duration in ms (0 = use global default)
};

struct superframe {
    uint16_t block_count;
    struct block_config *blocks;        // Array of blocks that make up the superframe
    uint32_t total_duration_ms;         // Sum of all block durations
    uint32_t *block_start_offsets_ms;   // Cumulative start offsets [0, d0, d0+d1, ...]
};

int superframe_alloc(uint16_t block_count, struct superframe **sframe);
int superframe_free(struct superframe *sframe);

/**
 * Initialize the block scheduler with a superframe
 *
 * @param frame The superframe configuration to use
 * @param repeat Whether to repeat the superframe
 * @param callback Function to call when superframe execution finishes
 * @return used internally by time synchronization scheduler.
 */
int execute_superframe(struct superframe *frame, bool repeat, superframe_callback_t callback, void *user_data);

/**
 * Get max execution time statistics for each block.
 *
 * Returns the maximum measured execution time (in ms) for each block
 * since the last superframe execution started.
 *
 * @param max_times_ms Output array for max times per block
 * @param max_count Size of output array
 * @return Number of entries written, or negative error code
 */
int block_scheduler_get_timing_stats(uint16_t *max_times_ms, uint8_t max_count);

#endif
