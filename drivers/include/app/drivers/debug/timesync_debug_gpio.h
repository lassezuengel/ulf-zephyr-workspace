#ifndef TIMESYNC_DEBUG_GPIO_H_
#define TIMESYNC_DEBUG_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

void timesync_debug_assert(void);
void timesync_debug_deassert(void);
void timesync_debug_pulse(void);

#ifdef __cplusplus
}
#endif

#endif /* TIMESYNC_DEBUG_GPIO_H_ */
