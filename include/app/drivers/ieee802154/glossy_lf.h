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

/**
 * @brief Initialize the Glossy LF module. The grandmaster (initiator) will start the synchronization rounds,
 * while followers will listen and synchronize to the grandmaster.
 */
int lf_clock_sync_init(bool grandmaster, int federate_id);

/**
 * @brief Schedule the next clock synchronization round.
 * @param next_sync_run_ms Pointer to store the time of the next synchronization round.
 * @param clock_offset_ms Pointer to store the measured clock offset.
 * @return 0 on success, negative error code on failure.
 */
int lf_clock_sync_schedule(int64_t* next_sync_run_ms, int64_t* clock_offset_ms);

#endif /* GLOSSY_LF_H_ */