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

#endif /* GLOSSY_LF_H_ */