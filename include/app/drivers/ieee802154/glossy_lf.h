#ifndef GLOSSY_LF_H
#define GLOSSY_LF_H

#include <stdbool.h>
#include <stdint.h>

struct glossy_lf_state {
	int64_t clock_offset_ms;
	bool offset_known;
	bool sync_lost;
	int failures_in_row;
};

int glossy_lf_init(bool initiator, int node_id);
void glossy_lf_get_state(struct glossy_lf_state *state);


/* LF-facing API */

int lf_clock_sync_init(bool grandmaster, int federate_id);
int lf_clock_sync_schedule(int64_t *next_glossy_ms);

#endif /* GLOSSY_LF_H_ */