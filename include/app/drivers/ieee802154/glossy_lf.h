#ifndef GLOSSY_LF_H
#define GLOSSY_LF_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief State structure for Glossy LF synchronization.
 * This structure holds the current clock offset, whether the offset is known,
 * whether synchronization is lost, and the number of consecutive failures.
 */
struct glossy_lf_state {
	int64_t clock_offset_ms;
	bool offset_known;
	bool sync_lost;
	int failures_in_row;
};

/**
 * @brief Initialize the glossy clock synchronization module.
 */
int glossy_lf_init(bool initiator, int node_id);

/**
 * @brief Get the current state of the glossy clock synchronization.
 * @param state Pointer to a glossy_lf_state struct to be filled with the current state
 */
void glossy_lf_get_state(struct glossy_lf_state *state);


/* LF-facing API */
typedef void (*ExternalClockSyncResultCallback)(void *user_data, int sync_status,
                                                int64_t next_sync_run_ms,
                                                int64_t clock_offset_ms);

/**
 * @brief Initialize the Glossy LF external clock synchronization module.
 *
 * Glossy does not run its own scheduling thread, so this sets
 * @p lf_drives_sync_schedule to true. The callback receives each round's
 * synchronization status, absolute next-run time, and local-minus-reference
 * clock offset.
 */
int lf_clock_sync_init(bool grandmaster, int federate_id,
                       ExternalClockSyncResultCallback result_callback,
                       void *result_callback_user_data,
                       bool *lf_drives_sync_schedule);

/**
 * @brief Run a clock synchronization round and report its result by callback.
 * @return 0 if the round was accepted, or a negative error before it started.
 */
int lf_clock_sync_schedule(void);

#endif /* GLOSSY_LF_H_ */
