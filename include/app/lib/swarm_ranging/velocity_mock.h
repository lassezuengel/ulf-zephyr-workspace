/**
 * @file velocity_mock.h
 * @brief Mock interface for flight controller velocity data
 *
 * Provides a simple interface to get velocity data. In the original
 * implementation, this comes from the flight controller via log variables.
 * This mock version returns zero or test values.
 *
 * Future: Replace with actual interface to flight controller when available.
 */

#ifndef VELOCITY_MOCK_H
#define VELOCITY_MOCK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Velocity data structure (m/s)
 */
typedef struct {
    float vx;  // Velocity in x direction (m/s)
    float vy;  // Velocity in y direction (m/s)
    float vz;  // Velocity in z direction (m/s)
} velocity_data_t;

/**
 * Get current velocity data
 *
 * @param vel Pointer to velocity_data_t structure to fill
 *
 * Currently returns zero velocity. In future implementations, this
 * could poll shared memory, use message queue, or callback mechanism
 * to get actual velocity from flight controller.
 */
void velocity_get(velocity_data_t *vel);

/**
 * Get velocity magnitude (m/s)
 *
 * @return Velocity magnitude in m/s
 */
float velocity_get_magnitude(void);

/**
 * Get velocity magnitude in cm/s (for ranging messages)
 *
 * @return Velocity magnitude in cm/s
 */
int16_t velocity_get_magnitude_cm_s(void);

/**
 * Set test velocity (for testing/simulation)
 *
 * @param vx Velocity x component (m/s)
 * @param vy Velocity y component (m/s)
 * @param vz Velocity z component (m/s)
 */
void velocity_set_test(float vx, float vy, float vz);

#ifdef __cplusplus
}
#endif

#endif /* VELOCITY_MOCK_H */
