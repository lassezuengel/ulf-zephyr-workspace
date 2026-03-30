#ifndef GLOSSY_BLOCK_H
#define GLOSSY_BLOCK_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <app/lib/scheduling/lower/schedule_functions.h>

/* Block Configuration objects */
struct glossy_block_config {
    uint16_t max_depth;           // Maximum glossy flooding depth
    uint16_t transmission_delay_us; // Transmission delay in microseconds
    uint16_t guard_period_us;     // Guard period in microseconds
    uint8_t channel;              // Optional UWB channel to use for Glossy (0 = unchanged)
};

void glossy_set_status_message(bool enabled);

#endif
