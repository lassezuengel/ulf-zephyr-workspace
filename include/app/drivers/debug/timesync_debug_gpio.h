#ifndef TIMESYNC_DEBUG_GPIO_H
#define TIMESYNC_DEBUG_GPIO_H

#include <zephyr/kernel.h>

/**
 * @brief Assert the time synchronization debug GPIO (set high)
 */
void timesync_debug_assert(void);

/**
 * @brief Deassert the time synchronization debug GPIO (set low)
 */
void timesync_debug_deassert(void);

/**
 * @brief Generate a brief pulse on the time synchronization debug GPIO
 *
 * Asserts the GPIO and then immediately deasserts it, creating a brief pulse
 * that can be observed on an oscilloscope or logic analyzer.
 */
void timesync_debug_pulse(void);

#endif /* TIMESYNC_DEBUG_GPIO_H */
