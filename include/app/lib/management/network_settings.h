#ifndef SYNCHROFLY_MANAGEMENT_NETWORK_SETTINGS_H
#define SYNCHROFLY_MANAGEMENT_NETWORK_SETTINGS_H

#include <stdint.h>

#include <app/lib/blocks/blocks.h>

schedule_type_t network_get_schedule_type(void);
uint8_t  network_get_ranging_round_slots_per_phase(void);
uint8_t  network_get_ranging_round_phases(void);
uint16_t network_get_glossy_guard_us(void);
uint16_t network_get_superframe_slots(void);
uint32_t network_get_scheduler_slot_duration_ms(void);
uint16_t network_get_glossy_max_depth(void);
uint16_t network_get_glossy_transmission_delay_us(void);
uint16_t network_get_slot_padding_us(void);

void network_set_schedule_type(schedule_type_t schedule_type);
void network_set_ranging_round_slots_per_phase(uint8_t slots);
void network_set_ranging_round_phases(uint8_t phases);
void network_set_glossy_guard_us(uint16_t guards_us);
void network_set_superframe_slots(uint16_t slots);
void network_set_scheduler_slot_duration_ms(uint32_t duration);
void network_set_glossy_max_depth(uint16_t max_depth);
void network_set_glossy_transmission_delay_us(uint16_t delay_us);
void network_set_slot_padding_us(uint16_t padding_us);

void network_print_settings(void);

// Diagnostic functions
const char* network_get_settings_source(void);
void network_save_config_defaults_to_nvs(void);

#endif /* APP_LIB_MANAGEMENT_NETWORK_SETTINGS_H */
