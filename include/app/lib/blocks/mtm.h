#ifndef MTM_BLOCK_H
#define MTM_BLOCK_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <app/drivers/ieee802154/dw1000.h>

struct mtm_block_config;

typedef enum {
    SCHEDULE_BASIC = 0,
    SCHEDULE_HASHED = 1,
    SCHEDULE_CONTENTION = 2,
} schedule_type_t;

typedef enum {
    MTM_STATUS_SUCCESS = 0,
    MTM_STATUS_ERROR = 1,
} mtm_status_t;

struct mtm_block_result {
    const struct deca_ranging_digest *digest;
    const struct mtm_block_config *config;
    const struct deca_schedule *schedule;
    uint64_t rtc;
};

#define MM_CIR_SAMPLE_MAX_BUFFER 5
struct leading_edge_cir_samples {
    int rx_slot;
    uint16_t cir_capture_size;
    int16_t re[MM_CIR_SAMPLE_MAX_BUFFER], im[MM_CIR_SAMPLE_MAX_BUFFER];
};

struct mm_block_result {
    const struct deca_ranging_digest *digest;
    const struct mm_block_config *config;
    const struct deca_schedule *schedule;
    uint64_t rtc;

    struct leading_edge_cir_samples *samples;
    size_t sample_count;
};

typedef void (*mtm_cb_t)(mtm_status_t status, const struct mtm_block_result *result, void *user_data);
typedef void (*mm_cb_t)(mtm_status_t status,  const struct mm_block_result *result, void *user_data);

struct mm_reference_block_result {
    const struct deca_ranging_digest *digest_chan_5;
    const struct deca_ranging_digest *digest_chan_3;
    const struct mm_reference_block_config *config;
    uint64_t rtc;
    // Channels used for this dual-channel reference ranging
    uint8_t channel_a;
    uint8_t channel_b;
};

typedef void (*mm_reference_cb_t)(mtm_status_t status, const struct mm_reference_block_result *result, void *user_data);

struct mm_block_config {
    schedule_type_t schedule_type;
    uint8_t ranging_round_slots_per_phase;
    uint8_t ranging_round_phases;

    deca_short_addr_t initiator_addr, responder_addr;

    mm_cb_t mm_cb;
    void *cb_user_data;
};

struct mtm_block_config {
    schedule_type_t schedule_type;
    uint8_t ranging_round_slots_per_phase;
    uint8_t ranging_round_phases;

    mtm_cb_t mtm_cb;
    void *cb_user_data;
};

struct mm_reference_block_config {
    uint32_t respond_interval_us;
    uint32_t guard_period_us;
    uint16_t timeout_us;
    deca_short_addr_t initiator_addr, responder_addr;

    // Configurable UWB channels for the dual-channel reference ranging
    // Defaults are 5 (primary) and 3 (secondary) if not set by the caller
    uint8_t channel_a; // first channel to use (e.g., 5)
    uint8_t channel_b; // second channel to use (e.g., 3)

    mm_reference_cb_t mm_ref_cb;
    void *cb_user_data;
};

void mtm_set_fp_index_threshold(uint16_t threshold);
void mtm_set_collect_cfo(bool enabled);
void mtm_set_correct_reject_frames(bool enabled);
void mtm_set_correct_timestamp_bias(bool enabled);

#endif
