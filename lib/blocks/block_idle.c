#include <stdint.h>

/* Idle block: intentionally does nothing. The slot duration provides a
 * quiet period with no radio activity. Can be extended later to
 * disable the radio or enter low-power mode. */
void idle_block_handler(uint64_t event_time, void *user_data)
{
    (void)event_time;
    (void)user_data;
}
