#ifndef SYNCHROFLY_LOG_H
#define SYNCHROFLY_LOG_H

#include <stdint.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */
#include <app/lib/ranging/twr.h>

void uart_out(char* msg);
void uart_disabled(char *msg);

void log_measurements_json(const struct measurement *measurements,
    int tof_count, uint64_t rtc_slot_ts, enum distance_unit unit, bool output_local_only, const char *additional);
void log_schedule_transmission_slots_json(const struct deca_schedule *schedule, int rtc);
void log_schedule_json(const struct deca_schedule *schedule, int rtc);
void log_frames_json(const struct deca_ranging_frame_container *frame_infos,
    int frame_info_count, const struct deca_ranging_configuration *conf, uint64_t rtc_slot_ts, uint8_t print_rx_ts, const char *additional);


void log_out(char* msg);
void log_flush();
#endif
