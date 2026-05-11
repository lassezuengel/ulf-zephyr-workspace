/**
 * @file velocity_mock.c
 * @brief Mock implementation for flight controller velocity data
 */

#include <app/lib/swarm_ranging/velocity_mock.h>
#include <math.h>

/* Static velocity storage for testing */
static velocity_data_t test_velocity = {
    .vx = 0.0f,
    .vy = 0.0f,
    .vz = 0.0f
};

void velocity_get(velocity_data_t *vel)
{
    if (vel == NULL) {
        return;
    }

    /* Return test velocity (default: zero) */
    vel->vx = test_velocity.vx;
    vel->vy = test_velocity.vy;
    vel->vz = test_velocity.vz;
}

float velocity_get_magnitude(void)
{
    /* Calculate magnitude: sqrt(vx^2 + vy^2 + vz^2) */
    float mag = sqrtf(
        test_velocity.vx * test_velocity.vx +
        test_velocity.vy * test_velocity.vy +
        test_velocity.vz * test_velocity.vz
    );
    return mag;
}

int16_t velocity_get_magnitude_cm_s(void)
{
    /* Convert m/s to cm/s */
    float mag_m_s = velocity_get_magnitude();
    return (int16_t)(mag_m_s * 100.0f);
}

void velocity_set_test(float vx, float vy, float vz)
{
    test_velocity.vx = vx;
    test_velocity.vy = vy;
    test_velocity.vz = vz;
}
