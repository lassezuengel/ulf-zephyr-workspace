#ifndef SYNCHROFLY_BLOCK_BUTLER_H
#define SYNCHROFLY_BLOCK_BUTLER_H

#include <stdint.h>

/**
 * @brief Butler block runtime configuration
 */
struct butler_block_config {
    uint8_t max_subslots;
    uint16_t subslot_duration_us;
    uint16_t guard_period_us;
    uint8_t p_tx_pct;           /**< TX probability percentage (0-100) */
    uint8_t channel;
};

/**
 * @brief Butler block handler
 *
 * Sets dirty flag, runs Butler flood, feeds result into time_sync_update().
 */
void butler_block_handler(uint64_t event_time, void *user_data);

#endif /* SYNCHROFLY_BLOCK_BUTLER_H */
