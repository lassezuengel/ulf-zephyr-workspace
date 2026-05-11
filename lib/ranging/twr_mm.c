#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/lib/ranging/twr.h>
#include <app/lib/ranging/phase_recovery.h>
#include <app/lib/ranging/phase_utils.h>
#include <app/lib/ranging/ambiguity_resolution.h>
#include <app/lib/ranging/time_series_store.h>
#include <app/lib/ranging/time_series_filters.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/lib/blocks/mtm.h>
#include <app/lib/node_table/node_table.h>
#include "poly_correction.h"
#include "xrloc_correction.h"

LOG_MODULE_REGISTER(twr_mm, CONFIG_TWR_MM_LOG_LEVEL);

void mm_set_time_series_size(uint8_t size)
{
#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
    node_table_set_mm_series_capacity(size);
#else
    ARG_UNUSED(size);
#endif
}

uint8_t mm_get_time_series_size(void)
{
#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
    return node_table_get_mm_series_capacity();
#else
    return CONFIG_MM_RANGING_TIME_SERIES_SIZE;
#endif
}

static int get_prf_from_device(const struct device *dev) {
    if (!dev) {
        LOG_ERR("Cannot determine PRF: null device");
        return -EINVAL;
    }

    const uwb_driver_t *driver = uwb_driver_get(dev);
    if (!driver || !driver->get_config) {
        LOG_ERR("Cannot determine PRF: driver missing get_config method");
        return -ENOTSUP;
    }

    uwb_config_t config;
    int ret = driver->get_config(dev, &config);
    if (ret < 0) {
        LOG_ERR("Failed to get device config: %d", ret);
        return ret;
    }

    return config.prf;
}

#define SPEED_OF_LIGHT_M_PER_S 299702547.236f
#define SPEED_OF_LIGHT_M_PER_UWB_TU                                                                \
	((SPEED_OF_LIGHT_M_PER_S * 1.0E-15f) * 15650.0f) // around 0.00469175196

// Filter indices for different measurement types used in MM ranging
// These are specific to the twr_mm module and define what each filter slot is used for
#define FILTER_IDX_TWR_TOF          0   // TWR Time-of-Flight measurements
#define FILTER_IDX_FINE_PHASE_CH_A  1   // Channel A fine phase measurements
#define FILTER_IDX_FINE_PHASE_CH_B  2   // Channel B fine phase measurements
#define FILTER_IDX_CFO              3   // Carrier frequency offset measurements
#define FILTER_IDX_COARSE_PHASE     4   // Coarse phase measurements

#include <app/lib/ranging/ts_series_ids.h>

static double process_fine_phase_sample(double wrapped_phase,
                                        ts_series_t *wrapped_series,
                                        ts_series_t *unwrapped_series)
{
    double processed_phase = wrapped_phase;

#if CONFIG_MM_RANGING_ENABLE_FINE_PHASE_FILTERING
    if (unwrapped_series) {
        double unwrapped_phase = wrapped_phase;
        double last_unwrapped;

        if (ts_last(unwrapped_series, &last_unwrapped)) {
            double diff = wrapped_phase - last_unwrapped;
            double wraps = round(diff / (2.0 * MM_PI));
            unwrapped_phase = last_unwrapped + diff - wraps * (2.0 * MM_PI);
        }

        if (ts_append(unwrapped_series, unwrapped_phase) != 0) {
            LOG_WRN("Failed to append unwrapped fine phase to time series");
        }

        double smoothed_unwrapped = unwrapped_phase;
#ifdef CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY
        int rc = ts_filter_savgol(unwrapped_series,
                                  CONFIG_SAVITZKY_GOLAY_WINDOW_SIZE,
                                  CONFIG_SAVITZKY_GOLAY_POLY_ORDER,
                                  &smoothed_unwrapped);
#else
        int rc = ts_filter_moving_average(unwrapped_series,
                                          CONFIG_MM_RANGING_FILTER_WINDOW_SIZE,
                                          &smoothed_unwrapped);
#endif
        if (rc == 0) {
            processed_phase = phase_wrap(smoothed_unwrapped);
        } else {
            if (rc != -ENODATA) {
                LOG_DBG("Fine phase smoothing fallback (rc=%d), using raw wrapped value", rc);
            }
            processed_phase = wrapped_phase;
        }
    }
#else
    (void)unwrapped_series;
#endif

    if (wrapped_series) {
        if (ts_append(wrapped_series, processed_phase) != 0) {
            LOG_WRN("Failed to append fine phase to time series");
        }
    }

    return processed_phase;
}

struct dstwr_template
{
    uint16_t ranging_initiator_id, ranging_responder_id;
    uint64_t tx_init, rx_init, tx_resp, rx_resp, tx_final, rx_final, tx_post_final, rx_post_final;
    double phase_init, phase_resp, phase_final, phase_post_final;

    // these are only relevant if ranging_initiator_id and ranging_responder_id are both not our own ranging id
    uint64_t local_rx_init, local_rx_resp, local_rx_final, local_post_final;
};

/* struct mm_twr_template is declared in twr.h */

struct transmission_event
{
    int seq;
    dwt_ts_t transmission_timestamp, reception_timestamp;
    dwt_ts_t passive_reception_timestamp;

    double reception_phase, passive_reception_phase;
};

typedef struct
{
    void *context;
    int (*getFirstMatchingTransmissionEventFrom)(void *context, uint16_t sender, uint16_t receiver,
        int seq, struct transmission_event *transmission);
} pair_view_t;

struct freq_offset
{
    uint8_t from_id, to_id;
    float offset;
};


/* struct propagation_time is declared in twr.h */

static struct propagation_time compute_propagation_time(struct dstwr_template *twr)
{
    struct propagation_time prop;

    // Extract timing values from dstwr_template for DS-TWR calculation
    // Initiator roundtrip: from tx_init to rx_resp
    int64_t initiator_roundtrip = correct_overflow(twr->rx_resp, twr->tx_init);

    // Responder reply delay: from rx_init to tx_resp
    int64_t responder_reply_delay = correct_overflow(twr->tx_resp, twr->rx_init);

    // Responder roundtrip: from rx_init to rx_final (total time at responder side)
    int64_t responder_roundtrip = correct_overflow(twr->rx_final, twr->tx_resp);

    // Initiator reply delay: from rx_resp to tx_final
    int64_t initiator_reply_delay = correct_overflow(twr->tx_final, twr->rx_resp);

    // Classic DS-TWR formula for ToF:
    // ToF = (T_round1 * T_round2 - T_reply1 * T_reply2) / (T_round1 + T_round2 + T_reply1 + T_reply2)
    int32_t tof_raw = (int32_t)(( ((int64_t) initiator_roundtrip * responder_roundtrip)
            - ((int64_t) responder_reply_delay * initiator_reply_delay))
        /
        ((int64_t) initiator_roundtrip
            +  responder_roundtrip
            +  responder_reply_delay
            +  initiator_reply_delay));

    // Calculate CFO (clock frequency offset) from timing measurements
    // CFO is based on the ratio of initiator vs responder durations
    uint64_t initiator_duration = correct_overflow(twr->tx_final, twr->tx_init);
    uint64_t responder_duration = correct_overflow(twr->rx_final, twr->rx_init);
    double drift_offset = (double)initiator_duration / (double)responder_duration;

    prop.tof = (float)tof_raw;
    prop.cfo = drift_offset;

    return prop;
}

static struct propagation_time calculate_propagation_time(struct dstwr_template *twr)
{
    struct propagation_time prop;
    static double relative_drift_offset, drift_offset;
    static int64_t responder_duration, initiator_duration;
    static int64_t round_duration_a, delay_duration_b;
    static int64_t drift_offset_int, two_tof_int;

    initiator_duration   = correct_overflow(twr->tx_final, twr->tx_init);
    responder_duration = correct_overflow(twr->rx_final, twr->rx_init);

    // factor determining whether B's clock runs faster or slower measured from the perspective of our clock
    // a positive factor here means that the clock runs faster than ours
    relative_drift_offset = (float)((int64_t)initiator_duration-(int64_t)responder_duration) / (float)(responder_duration);
    drift_offset = (double)initiator_duration/ (double)(responder_duration);

    round_duration_a = correct_overflow(twr->rx_resp, twr->tx_init);
    delay_duration_b = correct_overflow(twr->tx_resp, twr->rx_init);

    drift_offset_int = (int64_t)round(-relative_drift_offset * (double) delay_duration_b);

    two_tof_int = (int64_t)round_duration_a - (int64_t)delay_duration_b + drift_offset_int;

    prop.tof = ((float) two_tof_int) * 0.5f;
    prop.cfo = drift_offset;

    return prop;
}

static struct propagation_time calculate_propagation_time_cfo(struct dstwr_template *twr, double cfo)
{
    struct propagation_time prop;
    static double relative_drift_offset;
    static int64_t round_duration_a, delay_duration_b;
    static int64_t drift_offset_int, two_tof_int;

    relative_drift_offset = -cfo;

    round_duration_a = correct_overflow(twr->rx_resp, twr->tx_init);
    delay_duration_b = correct_overflow(twr->tx_resp, twr->rx_init);

    drift_offset_int = (int64_t)round(-relative_drift_offset * (double) delay_duration_b);

    two_tof_int = (int64_t)round_duration_a - (int64_t)delay_duration_b + drift_offset_int;

    prop.tof = ((float) two_tof_int) * 0.5f;
    prop.cfo = relative_drift_offset;

    return prop;
}

static int get_first_matching_transmission_event_from_digest(void *context, uint16_t sender, uint16_t receiver, int seq, struct transmission_event *transmission)
{
    struct deca_ranging_digest *digest = (struct deca_ranging_digest *)context;
    int transmissionSeq = INT_MAX, receptionSeq = INT_MAX;
    dwt_ts_t transmission_timestamp = 0, reception_timestamp = 0;
    float reception_phase = -100;

    for(size_t i = 0; i < digest->length; i++) {
        const struct deca_ranging_frame_container *container = &digest->frames[i];
        const struct deca_ranging_frame *frame = container->frame;

        if (frame == NULL || container->status == DECA_FRAME_REJECTED) {
            continue;
	}

        if (container->slot < seq) {
            continue;
	}

        if (frame->addr == sender && container->slot < transmissionSeq) {
            transmissionSeq = container->slot;
            transmission_timestamp = from_packed_uwb_ts(frame->tx_ts);
        }
    }

    // find matching reception timestamp in frames
    for (size_t i = 0; i < digest->length && receptionSeq == INT_MAX; i++) {
        const struct deca_ranging_frame_container *container = &digest->frames[i];
	const struct deca_ranging_frame *frame = container->frame;
        struct deca_tagged_mm_timestamp *frame_timestamps = NULL;
        int rx_ts_count;

        if (frame == NULL || container->status == DECA_FRAME_REJECTED) {
		continue;
        }

        // first check if we have a received frame from which we can directly take the reception timestamp
        if(container->type == DECA_RECEIVED && container->slot == transmissionSeq && frame->addr == sender) {
            receptionSeq = container->slot;
	    reception_timestamp = container->timestamp;
            reception_phase = atan2(container->fp_im, container->fp_re);
            break;
        }

        rx_ts_count = deca_ranging_frame_get_mm_tagged_timestamps(frame, &frame_timestamps);

        if (frame->addr != receiver) {
            continue;
	}

        for (size_t j = 0; j < rx_ts_count; j++) {
            struct deca_tagged_mm_timestamp ts = frame_timestamps[j];

            uint16_t slot = ts.slot;
            if (slot < seq) {
                continue;
            }
            if (ts.addr == sender && slot == transmissionSeq) {
                receptionSeq = slot;
		reception_timestamp = from_packed_uwb_ts(ts.ts);
                reception_phase = atan2(ts.im, ts.re);
                break;
            }
	}
    }

    if(receptionSeq == INT_MAX || transmissionSeq == INT_MAX) {
        return -1;
    }

    transmission->seq = transmissionSeq;
    transmission->transmission_timestamp = transmission_timestamp;
    transmission->reception_timestamp = reception_timestamp;
    transmission->reception_phase = reception_phase;

    transmission->passive_reception_phase = NAN; // TODO, we need to get our uwb address to do this into this module
    transmission->passive_reception_timestamp = UINT64_MAX; // TODO, we need to get our uwb address to do this into this module

    return transmission->seq;
}

// in memory lookup
static pair_view_t createDigestPairView(const struct deca_ranging_digest *digest)
{
    pair_view_t view;
    view.context = (void *)digest;
    view.getFirstMatchingTransmissionEventFrom = get_first_matching_transmission_event_from_digest;

    return view;
}

static bool add_address_if_not_in_list(deca_short_addr_t *addrs, int length, deca_short_addr_t addr)
{
    for(int i = 0; i < length; i++) {
        if(addrs[i] == addr) {
            return 0;
        }
    }

    addrs[length] = addr;

    return 1;
}

static int get_digest_ranging_ids(const struct deca_ranging_digest *digest, deca_short_addr_t *addrs, int max_length)
{
    int length = 0;
    for(int i = 0; i < digest->length; i++) {
        const struct deca_ranging_frame_container *container = &digest->frames[i];
        const struct deca_ranging_frame *frame = container->frame;
        struct deca_tagged_mm_timestamp *frame_timestamps = NULL;
        int rx_ts_count;

        if(frame == NULL || container->status == DECA_FRAME_REJECTED) {
            continue;
        }

	deca_short_addr_t frame_id = frame->addr;
        rx_ts_count = deca_ranging_frame_get_mm_tagged_timestamps(frame, &frame_timestamps);

	length += add_address_if_not_in_list(addrs, length, frame_id);
	if (length >= max_length) {
            LOG_WRN("Ranging digest contains more than %d unique addresses", max_length);
            return length;
        }

        int err = 0;
        for (int j = 0; j < rx_ts_count; j++) {
            struct deca_tagged_mm_timestamp ts = frame_timestamps[j];
            deca_short_addr_t ranging_id = ts.addr;

	    length += add_address_if_not_in_list(addrs, length, ranging_id);
	    if (length >= max_length) {
                LOG_WRN("Ranging digest contains more than %d unique addresses", max_length);
                return length;
            }
        } if(err) printk("\n");
    }

    return length;
}

static bool single_sided_complete(const struct dstwr_template *twr_template)
{
    return twr_template->tx_init != UINT64_MAX && twr_template->rx_init != UINT64_MAX &&
        twr_template->tx_resp != UINT64_MAX && twr_template->rx_resp != UINT64_MAX;
}

static bool double_sided_complete(const struct dstwr_template *twr_template)
{
    return single_sided_complete(twr_template) && twr_template->tx_final != UINT64_MAX && twr_template->rx_final != UINT64_MAX;
}

bool mm_template_complete(const struct mm_twr_template *tmpl)
{
    if (!tmpl) {
        return false;
    }

    // Check that we have all required phase measurements for MM ranging
    if (isnan(tmpl->phase_poll) || isnan(tmpl->phase_resp) ||
        isnan(tmpl->phase_final) || isnan(tmpl->phase_post_final)) {
        return false;
    }

    // Check that we have valid timestamps for the MM protocol
    if (tmpl->tx_poll == UINT64_MAX || tmpl->rx_poll == UINT64_MAX ||
        tmpl->tx_resp == UINT64_MAX || tmpl->rx_resp == UINT64_MAX ||
        tmpl->tx_final == UINT64_MAX || tmpl->rx_final == UINT64_MAX ||
        tmpl->tx_post_final == UINT64_MAX || tmpl->rx_post_final == UINT64_MAX) {
        return false;
    }

    return true;
}

static struct dstwr_template create_dstwr_template(deca_short_addr_t initiator_id, deca_short_addr_t responder_id)
{
    struct dstwr_template twr_template;
    twr_template.ranging_initiator_id = initiator_id;
    twr_template.ranging_responder_id = responder_id;
    twr_template.local_rx_init = UINT64_MAX;
    twr_template.local_rx_resp = UINT64_MAX;
    twr_template.local_rx_final = UINT64_MAX;
    twr_template.local_post_final = UINT64_MAX;
    twr_template.tx_init = UINT64_MAX;
    twr_template.rx_init = UINT64_MAX;
    twr_template.tx_resp = UINT64_MAX;
    twr_template.rx_resp = UINT64_MAX;
    twr_template.tx_final = UINT64_MAX;
    twr_template.rx_final = UINT64_MAX;
    twr_template.rx_post_final = UINT64_MAX;

    // put nan
    twr_template.phase_init = NAN;
    twr_template.phase_resp = NAN;
    twr_template.phase_final = NAN;
    twr_template.phase_post_final = NAN;

    return twr_template;
}

static struct dstwr_template get_dstwr_for_node_pair(const struct deca_ranging_digest *digest, deca_short_addr_t initiator_id, deca_short_addr_t responder_id)
{
	/* PairView pairs = createDigestPairView(digest); */
    pair_view_t pairs = createDigestPairView(digest);
    struct dstwr_template DSTWRTemplate = create_dstwr_template(initiator_id, responder_id);
    int slot;

    struct transmission_event initiation;
    struct transmission_event response;
    struct transmission_event finalization;
    struct transmission_event post_finalization;

    if ((slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, initiator_id, responder_id, 0, &initiation)) >= 0) {
        DSTWRTemplate.tx_init = initiation.transmission_timestamp;
        DSTWRTemplate.rx_init = initiation.reception_timestamp;
	DSTWRTemplate.local_rx_init = initiation.passive_reception_timestamp;
        DSTWRTemplate.phase_init = initiation.reception_phase;
    }

    if (slot >= 0 && (slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, responder_id, initiator_id, slot + 1, &response)) >= 0) {
        DSTWRTemplate.tx_resp = response.transmission_timestamp;
        DSTWRTemplate.rx_resp = response.reception_timestamp;
	DSTWRTemplate.local_rx_resp = response.passive_reception_timestamp;
        DSTWRTemplate.phase_resp = response.reception_phase;
    }

    if (slot >= 0 && (slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, initiator_id, responder_id, slot + 1, &finalization)) >= 0) {
        DSTWRTemplate.tx_final = finalization.transmission_timestamp;
        DSTWRTemplate.rx_final = finalization.reception_timestamp;
	DSTWRTemplate.local_rx_final = finalization.passive_reception_timestamp;
        DSTWRTemplate.phase_final = finalization.reception_phase;
    }

    if (slot >= 0 && (slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, initiator_id, responder_id, slot + 1, &post_finalization)) >= 0) {
        DSTWRTemplate.rx_post_final = post_finalization.reception_timestamp;
        DSTWRTemplate.tx_post_final = post_finalization.transmission_timestamp;
        DSTWRTemplate.local_post_final = post_finalization.passive_reception_timestamp;
        DSTWRTemplate.phase_post_final = post_finalization.reception_phase;
    }

    return DSTWRTemplate;
}

static double __attribute__((unused)) frequencyOffsetFromDSTWR(struct dstwr_template *twr)
{
    uint64_t initiator_duration   = correct_overflow(twr->tx_final, twr->tx_init);
    uint64_t responder_duration = correct_overflow(twr->rx_final, twr->rx_init);
    return (double)((int64_t)initiator_duration - (int64_t)responder_duration) /
                (double)(responder_duration);
}

static int frequencyOffsetFromCFO(const struct deca_ranging_digest *digest, deca_short_addr_t transmitter, bool average, double *offset)
{
    double average_offset = 0;
    int count = 0;
    for(size_t i = 0; i < digest->length; i++) {
        const struct deca_ranging_frame_container *container = &digest->frames[i];
        const struct deca_ranging_frame *frame = container->frame;

        if(frame == NULL || container->status == DECA_FRAME_REJECTED
            || container->type != DECA_RECEIVED || container->frame->addr != transmitter
            ) {
            continue;
        }

        count++;
        average_offset += (double)(-(container->cfo_ppm / 1.0e6f));

        if(!average) {
            break;
        }
    }

    if(count <= 0) {
        return -1;
    }

    *offset = average_offset / (double)count;

    return count;
}



static int process_dual_channel_phases_with_templates(
    const struct mm_twr_template *tmpl_ch_a,
    const struct mm_twr_template *tmpl_ch_b,
    struct dual_channel_phases *result)
{
    if (!tmpl_ch_a || !tmpl_ch_b || !result) {
        LOG_ERR("Invalid parameters: tmpl_ch_a=%p, tmpl_ch_b=%p, result=%p",
               tmpl_ch_a, tmpl_ch_b, result);
        return -EINVAL;
    }

    // Initialize result structure
    memset(result, 0, sizeof(struct dual_channel_phases));
    result->initiator_id = tmpl_ch_a->initiator_id;
    result->responder_id = tmpl_ch_a->responder_id;

    // Check if we have all required phases for coarse recovery
    if (isnan(tmpl_ch_a->phase_poll) || isnan(tmpl_ch_a->phase_resp) ||
        isnan(tmpl_ch_b->phase_poll) || isnan(tmpl_ch_b->phase_resp)) {
        LOG_ERR("Missing required phases for coarse recovery (pair %u-%u)",
               tmpl_ch_a->initiator_id, tmpl_ch_a->responder_id);
        return -ENODATA;
    }

    // Compute coarse phases for both channels using phase_recovery functions directly
    result->coarse_phase_ch5 = compute_coarse_phase_mm(tmpl_ch_a->phase_poll, tmpl_ch_a->phase_resp);
    result->coarse_phase_ch3 = compute_coarse_phase_mm(tmpl_ch_b->phase_poll, tmpl_ch_b->phase_resp);

    // Compute fine phases using detailed variant to get all phase components
    if (!isnan(tmpl_ch_a->phase_final) && !isnan(tmpl_ch_a->phase_post_final)) {
        struct fine_phase_result phase_result_cha;
        int ret = compute_fine_phase_mm_detailed(result->coarse_phase_ch5,
                                                tmpl_ch_a->phase_final, tmpl_ch_a->phase_post_final,
                                                tmpl_ch_a->initiator_id, tmpl_ch_a->responder_id,
                                                tmpl_ch_a->channel, &phase_result_cha);
        if (ret == 0) {
            result->fine_phase_ch5 = phase_result_cha.fine_phase;
            result->residual_phase_raw_ch5 = phase_result_cha.residual_phase_raw;
        } else {
            LOG_ERR("Failed to compute fine phase for channel A (ret=%d)", ret);
            return ret;
        }
    } else {
        LOG_ERR("Missing fine phase data for channel A: final=%.6f, post_final=%.6f",
                tmpl_ch_a->phase_final, tmpl_ch_a->phase_post_final);
        return -ENODATA;
    }

    if (!isnan(tmpl_ch_b->phase_final) && !isnan(tmpl_ch_b->phase_post_final)) {
        struct fine_phase_result phase_result_chb;
        int ret = compute_fine_phase_mm_detailed(result->coarse_phase_ch3,
                                                tmpl_ch_b->phase_final, tmpl_ch_b->phase_post_final,
                                                tmpl_ch_b->initiator_id, tmpl_ch_b->responder_id,
                                                tmpl_ch_b->channel, &phase_result_chb);
        if (ret == 0) {
            result->fine_phase_ch3 = phase_result_chb.fine_phase;
            result->residual_phase_raw_ch3 = phase_result_chb.residual_phase_raw;
        } else {
            LOG_ERR("Failed to compute fine phase for channel B (ret=%d)", ret);
            return ret;
        }
    } else {
        LOG_ERR("Missing fine phase data for channel B: final=%.6f, post_final=%.6f",
                tmpl_ch_b->phase_final, tmpl_ch_b->phase_post_final);
        return -ENODATA;
    }

    // Extract CFO information from templates
    result->cfo_ch5 = tmpl_ch_a->cfo_poll;
    result->cfo_ch3 = tmpl_ch_b->cfo_poll;

    LOG_INF("Dual-channel phase processing (pair %u-%u): "
           "ch_a_coarse=%.6f, ch_a_fine=%.6f, ch_b_coarse=%.6f, ch_b_fine=%.6f, cfo_a=%.3f, cfo_b=%.3f",
           result->initiator_id, result->responder_id,
           result->coarse_phase_ch5, result->fine_phase_ch5,
           result->coarse_phase_ch3, result->fine_phase_ch3,
           result->cfo_ch5, result->cfo_ch3);

    return 0;
}

static struct mm_twr_template create_mm_template(uint16_t initiator_id, uint16_t responder_id, uint8_t channel)
{
    struct mm_twr_template tmpl;

    tmpl.initiator_id = initiator_id;
    tmpl.responder_id = responder_id;
    tmpl.channel = channel;

    // Initialize timestamps to invalid values
    tmpl.tx_poll = UINT64_MAX;
    tmpl.rx_poll = UINT64_MAX;
    tmpl.tx_resp = UINT64_MAX;
    tmpl.rx_resp = UINT64_MAX;
    tmpl.tx_final = UINT64_MAX;
    tmpl.rx_final = UINT64_MAX;
    tmpl.tx_post_final = UINT64_MAX;
    tmpl.rx_post_final = UINT64_MAX;

    // Initialize phases to NaN
    tmpl.phase_poll = NAN;
    tmpl.phase_resp = NAN;
    tmpl.phase_final = NAN;
    tmpl.phase_post_final = NAN;

    // Initialize CFO to NaN
    tmpl.cfo_poll = NAN;
    tmpl.cfo_resp = NAN;

    return tmpl;
}

struct mm_twr_template build_mm_template_from_digest(
    const struct deca_ranging_digest *digest,
    uint16_t initiator_id,
    uint16_t responder_id,
    uint8_t channel)
{
    struct mm_twr_template tmpl = create_mm_template(initiator_id, responder_id, channel);
    pair_view_t pairs = createDigestPairView(digest);
    int slot;

    struct transmission_event poll_event;
    struct transmission_event resp_event;
    struct transmission_event final_event;
    struct transmission_event post_final_event;

    // Extract Poll message (Initiator → Responder, slot 1)
    if ((slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, initiator_id, responder_id, 1, &poll_event)) >= 0) {
        tmpl.tx_poll = poll_event.transmission_timestamp;
        tmpl.rx_poll = poll_event.reception_timestamp;
        tmpl.phase_poll = poll_event.reception_phase;
    }

    // Extract Response message (Responder → Initiator, slot 2)
    if (slot >= 0 && (slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, responder_id, initiator_id, slot + 1, &resp_event)) >= 0) {
        tmpl.tx_resp = resp_event.transmission_timestamp;
        tmpl.rx_resp = resp_event.reception_timestamp;
        tmpl.phase_resp = resp_event.reception_phase;
    }

    // Extract Final message (Initiator → Responder, slot 4)
    if (slot >= 0 && (slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, initiator_id, responder_id, slot + 1, &final_event)) >= 0) {
        tmpl.tx_final = final_event.transmission_timestamp;
        tmpl.rx_final = final_event.reception_timestamp;
        tmpl.phase_final = final_event.reception_phase;
    }

    // Extract Post-Final message (Initiator → Responder, slot 5)
    if (slot >= 0 && (slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, initiator_id, responder_id, slot + 1, &post_final_event)) >= 0) {
        tmpl.tx_post_final = post_final_event.transmission_timestamp;
        tmpl.rx_post_final = post_final_event.reception_timestamp;
        tmpl.phase_post_final = post_final_event.reception_phase;
    }

    return tmpl;
}

struct propagation_time mm_calculate_propagation_time(const struct mm_twr_template *tmpl)
{
    /* Build a dstwr_template from the first 6 timestamps of the MM template */
    struct dstwr_template dstwr;
    memset(&dstwr, 0, sizeof(dstwr));
    dstwr.ranging_initiator_id = tmpl->initiator_id;
    dstwr.ranging_responder_id = tmpl->responder_id;
    dstwr.tx_init = tmpl->tx_poll;
    dstwr.rx_init = tmpl->rx_poll;
    dstwr.tx_resp = tmpl->tx_resp;
    dstwr.rx_resp = tmpl->rx_resp;
    dstwr.tx_final = tmpl->tx_final;
    dstwr.rx_final = tmpl->rx_final;
    return calculate_propagation_time(&dstwr);
}

int estimate_distances_mm(const struct device *dev,
                          const struct deca_ranging_digest *digest, uint8_t channel, bool estimateOnlyLocal,
			deca_short_addr_t local_addr, struct measurement_mm *measurements,
			int max_measurements)
{
    static deca_short_addr_t ranging_ids[100];
    int ranging_ids_length = get_digest_ranging_ids(digest, ranging_ids, sizeof(ranging_ids) / sizeof(ranging_ids[0]));
    int measurement_count = 0;
    for (size_t i = 0; i < ranging_ids_length; i++) {
        for (size_t j = 0; j < ranging_ids_length; j++) {
		// TODO skip here for option to calculate local only distances if(localOnly && ...
            if (i == j) {
                continue;
	    }

            struct dstwr_template twr = get_dstwr_for_node_pair(digest, ranging_ids[i], ranging_ids[j]);
            float __attribute__((unused)) tof = 0;
            bool success = 0;
            struct propagation_time prop;

            if (double_sided_complete(&twr)) {
                /* in case of double sided two way ranging we will have both direction available. just use the measurement where we are the initiator */
                if(estimateOnlyLocal && ranging_ids[i] != local_addr) {
                    continue;
                }
                prop = compute_propagation_time(&twr);
                success = true;
            } else if (estimateOnlyLocal && single_sided_complete(&twr)){ /* SS-TWR relies on CFO which we don't share with other nodes, thus only possible locally */
                double cfo;
                int other_addr;
                if(ranging_ids[i] != local_addr && ranging_ids[j] != local_addr) {
                    continue;
                }

                other_addr = ranging_ids[i] == local_addr ? ranging_ids[j] : ranging_ids[i];

                if(frequencyOffsetFromCFO(digest, other_addr, true, &cfo) >= 0) {
                    prop = calculate_propagation_time_cfo(&twr, cfo);
                    success = true;
                }
            }

            if(success) {
                if (measurement_count >= max_measurements) {
                    LOG_WRN("Not enough space to store all measurements");
                    return measurement_count;
                }

                struct measurement_mm m;
                m.ranging_initiator_id = twr.ranging_initiator_id;
                m.ranging_responder_id = twr.ranging_responder_id;

                // Apply range bias correction to TOF if mode is DISTANCE
                float corrected_tof = prop.tof;
                double raw_distance_m = prop.tof * SPEED_OF_LIGHT_M_PER_UWB_TU;
                double applied_bias_m = 0.0;
                if (mtm_get_bias_correction_mode() == BIAS_CORRECTION_DISTANCE) {
                    int prf = get_prf_from_device(dev);
                    if (prf > 0) {
                        const uwb_driver_t *driver = uwb_driver_get(dev);
                        if (driver && driver->get_range_bias) {
                            double bias = driver->get_range_bias(dev, channel, raw_distance_m, prf);
                            double corrected_distance = raw_distance_m - bias;
                            applied_bias_m = bias;
                            corrected_tof = ((float)corrected_distance / SPEED_OF_LIGHT_M_PER_UWB_TU);
                            LOG_INF("Range bias correction [single-ch]: initiator=%u, responder=%u, ch=%u, prf=%uMHz, raw_dist=%.3fm, bias=%.3fm, corrected_dist=%.3fm",
                                   twr.ranging_initiator_id, twr.ranging_responder_id, channel, prf, raw_distance_m, bias, corrected_distance);
                        }
                    }
                } else if (mtm_get_bias_correction_mode() == BIAS_CORRECTION_POLY_PER_NODE) {
                    uint16_t peer = twr.ranging_responder_id;
                    const struct correction_poly *poly = poly_correction_get_node(peer, channel);
                    if (poly && poly->degree > 0) {
                        double delta = (double)poly_correction_eval(poly, (float)raw_distance_m);
                        double corrected_distance = raw_distance_m + delta;
                        applied_bias_m = -delta;
                        corrected_tof = ((float)corrected_distance / SPEED_OF_LIGHT_M_PER_UWB_TU);
                        LOG_INF("Poly correction [per-node 0x%04x ch%u]: raw=%.3f, delta=%.4f, corrected=%.3f",
                               peer, channel, raw_distance_m, delta, corrected_distance);
                    }
                } else if (mtm_get_bias_correction_mode() == BIAS_CORRECTION_POLY_GLOBAL) {
                    const struct correction_poly *poly = poly_correction_get_global();
                    if (poly && poly->degree > 0) {
                        double delta = (double)poly_correction_eval(poly, (float)raw_distance_m);
                        double corrected_distance = raw_distance_m + delta;
                        applied_bias_m = -delta;
                        corrected_tof = ((float)corrected_distance / SPEED_OF_LIGHT_M_PER_UWB_TU);
                        LOG_INF("Poly correction [global]: raw=%.3f, delta=%.4f, corrected=%.3f",
                               raw_distance_m, delta, corrected_distance);
                    }
                } else if (mtm_get_bias_correction_mode() == BIAS_CORRECTION_XRLOC_PER_NODE) {
                    uint16_t peer = twr.ranging_responder_id;
                    const struct xrloc_correction *xc = xrloc_correction_get_node(peer, channel);
                    if (xc) {
                        double delta = (double)xrloc_correction_eval(xc, (float)raw_distance_m);
                        double corrected_distance = raw_distance_m + delta;
                        applied_bias_m = -delta;
                        corrected_tof = ((float)corrected_distance / SPEED_OF_LIGHT_M_PER_UWB_TU);
                        LOG_INF("XRLoc correction [per-node 0x%04x]: raw=%.3f, delta=%.4f, corrected=%.3f",
                               peer, raw_distance_m, delta, corrected_distance);
                    } else {
                        LOG_WRN("XRLoc mode active but no entry for peer 0x%04x (count=%zu)",
                               peer, xrloc_correction_node_count());
                    }
                }
                m.tof = corrected_tof;
                m.range_bias_m = (float)applied_bias_m;
                m.distance_twr_raw = (float)raw_distance_m;
                m.distance_twr = (float)(raw_distance_m - applied_bias_m);
                m.cfo = prop.cfo;
		m.tof_quality = 0;
                // Use correct MM phase calculation
                double coarse_phase = compute_coarse_phase_mm(twr.phase_init, twr.phase_resp);
                double fine_phase = compute_fine_phase_mm(coarse_phase, twr.phase_final, twr.phase_post_final,
                                                         twr.ranging_initiator_id, twr.ranging_responder_id, channel);

                uint16_t phase_series_id = (uint16_t)(TS_ID_PHASE_BASE + channel);
                ts_series_key_t phase_key = {
                    .initiator_id = twr.ranging_initiator_id,
                    .responder_id = twr.ranging_responder_id,
                    .series_id = phase_series_id,
                };
                ts_series_t *phase_series = ts_get_or_create(&phase_key, mm_get_time_series_size());
                if (!phase_series) {
                    LOG_WRN("Failed to create fine phase series for pair %u-%u ch%u",
                           twr.ranging_initiator_id, twr.ranging_responder_id, channel);
                }

#if CONFIG_MM_RANGING_ENABLE_FINE_PHASE_FILTERING
                ts_series_t *phase_unwrapped_series = NULL;
                ts_series_key_t phase_unwrapped_key = {
                    .initiator_id = twr.ranging_initiator_id,
                    .responder_id = twr.ranging_responder_id,
                    .series_id = (uint16_t)(TS_ID_PHASE_UNWRAPPED_BASE + channel),
                };
                phase_unwrapped_series = ts_get_or_create(&phase_unwrapped_key, mm_get_time_series_size());
                if (!phase_unwrapped_series) {
                    LOG_WRN("Failed to create unwrapped fine phase series for pair %u-%u ch%u",
                           twr.ranging_initiator_id, twr.ranging_responder_id, channel);
                }
#else
                ts_series_t *phase_unwrapped_series = NULL;
#endif

                fine_phase = process_fine_phase_sample(fine_phase, phase_series, phase_unwrapped_series);
                m.d_diff = fine_phase;
                /* m.d_diff = twr.phase_init + twr.phase_resp; */

                measurements[measurement_count++] = m;
            }
        }
    }

    return measurement_count;
}

int estimate_distances_mm_dual(const struct device *dev,
                               const struct deca_ranging_digest *digest_ch_a,
                               const struct deca_ranging_digest *digest_ch_b,
                               uint8_t channel_a,
                               uint8_t channel_b,
                               bool estimateOnlyLocal,
                               deca_short_addr_t local_addr,
                               struct measurement_mm *measurements,
                               int max_measurements)
{
    static deca_short_addr_t ranging_ids[100];
    int ranging_ids_length_ch_a = get_digest_ranging_ids(digest_ch_a, ranging_ids,
                                                        sizeof(ranging_ids) / sizeof(ranging_ids[0]));

    // Ensure both channels have the same node pairs (could intersect sets later for robustness)
    int measurement_count = 0;

    // Time series store is on-demand via ts_get_or_create

    for (size_t i = 0; i < ranging_ids_length_ch_a; i++) {
        for (size_t j = 0; j < ranging_ids_length_ch_a; j++) {
            if (i == j) {
                continue;
            }

            uint16_t initiator_id = ranging_ids[i];
            uint16_t responder_id = ranging_ids[j];

            // Skip non-local measurements if requested
            if (estimateOnlyLocal && initiator_id != local_addr) {
                continue;
            }

            // Get TWR templates for both channels
            struct dstwr_template twr_ch_a = get_dstwr_for_node_pair(digest_ch_a, initiator_id, responder_id);
            struct dstwr_template twr_ch_b = get_dstwr_for_node_pair(digest_ch_b, initiator_id, responder_id);

            // Check if we have sufficient data for dual-channel processing
            bool ch_a_complete = double_sided_complete(&twr_ch_a);
            bool ch_b_complete = double_sided_complete(&twr_ch_b);

            if (!ch_a_complete || !ch_b_complete) {
                LOG_DBG("Incomplete dual-channel data for pair %u-%u (ch_a=%d, ch_b=%d), skipping",
                       initiator_id, responder_id, ch_a_complete, ch_b_complete);
                continue;
            }

            // Build MM templates for both channels
            struct mm_twr_template tmpl_ch_a = build_mm_template_from_digest(digest_ch_a, initiator_id, responder_id, channel_a);
            struct mm_twr_template tmpl_ch_b = build_mm_template_from_digest(digest_ch_b, initiator_id, responder_id, channel_b);

            // Check if MM templates have all required data for phase processing
            bool tmpl_a_complete = mm_template_complete(&tmpl_ch_a);
            bool tmpl_b_complete = mm_template_complete(&tmpl_ch_b);

            if (!tmpl_a_complete || !tmpl_b_complete) {
                LOG_DBG("Incomplete MM template data for pair %u-%u (tmpl_a=%d, tmpl_b=%d), skipping",
                       initiator_id, responder_id, tmpl_a_complete, tmpl_b_complete);
                continue;
            }

            // Process dual-channel phases using new template approach
            struct dual_channel_phases phases;
            int ret = process_dual_channel_phases_with_templates(&tmpl_ch_a, &tmpl_ch_b, &phases);
            if (ret < 0) {
                LOG_ERR("Failed to process dual-channel phases for pair %u-%u: %d",
                       initiator_id, responder_id, ret);
                continue;
            }

            // Calculate coarse distance using Channel A (first digest)
            /* struct propagation_time prop_ch_a = compute_propagation_time(&twr_ch_a); */
            struct propagation_time prop_ch_a = calculate_propagation_time(&twr_ch_a);

            double raw_coarse_distance = prop_ch_a.tof * SPEED_OF_LIGHT_M_PER_UWB_TU;
            double uncorrected_distance = raw_coarse_distance;
            double applied_bias_m = 0.0;

            // Apply range bias correction if mode is DISTANCE
            if (mtm_get_bias_correction_mode() == BIAS_CORRECTION_DISTANCE) {
                int prf = get_prf_from_device(dev);
                if (prf > 0) {
                    const uwb_driver_t *driver = uwb_driver_get(dev);
                    if (driver && driver->get_range_bias) {
                        double bias = driver->get_range_bias(dev, channel_a, raw_coarse_distance, prf);
                        raw_coarse_distance -= bias;
                        applied_bias_m = bias;
                        LOG_INF("Range bias correction [dual-ch]: initiator=%u, responder=%u, ch_a=%u, prf=%uMHz, raw_dist=%.3fm, bias=%.3fm, corrected_dist=%.3fm",
                               initiator_id, responder_id, channel_a, prf, uncorrected_distance, bias, raw_coarse_distance);
                    }
                }
            } else if (mtm_get_bias_correction_mode() == BIAS_CORRECTION_POLY_PER_NODE) {
                const struct correction_poly *poly = poly_correction_get_node(responder_id, channel_a);
                if (poly && poly->degree > 0) {
                    double delta = (double)poly_correction_eval(poly, (float)raw_coarse_distance);
                    raw_coarse_distance += delta;
                    applied_bias_m = -delta;
                    LOG_INF("Poly correction [dual-ch per-node 0x%04x ch%u]: raw=%.3f, delta=%.4f, corrected=%.3f",
                           responder_id, channel_a, uncorrected_distance, delta, raw_coarse_distance);
                }
            } else if (mtm_get_bias_correction_mode() == BIAS_CORRECTION_POLY_GLOBAL) {
                const struct correction_poly *poly = poly_correction_get_global();
                if (poly && poly->degree > 0) {
                    double delta = (double)poly_correction_eval(poly, (float)raw_coarse_distance);
                    raw_coarse_distance += delta;
                    applied_bias_m = -delta;
                    LOG_INF("Poly correction [dual-ch global]: raw=%.3f, delta=%.4f, corrected=%.3f",
                           uncorrected_distance, delta, raw_coarse_distance);
                }
            } else if (mtm_get_bias_correction_mode() == BIAS_CORRECTION_XRLOC_PER_NODE) {
                const struct xrloc_correction *xc = xrloc_correction_get_node(responder_id, channel_a);
                if (xc) {
                    double delta = (double)xrloc_correction_eval(xc, (float)raw_coarse_distance);
                    raw_coarse_distance += delta;
                    applied_bias_m = -delta;
                    LOG_INF("XRLoc correction [dual-ch per-node 0x%04x]: raw=%.3f, delta=%.4f, corrected=%.3f",
                           responder_id, uncorrected_distance, delta, raw_coarse_distance);
                } else {
                    LOG_WRN("XRLoc mode active but no entry for peer 0x%04x (count=%zu)",
                           responder_id, xrloc_correction_node_count());
                }
            }

            // Use raw measurements initially; filtering can be applied at ambiguity resolution if desired
            double coarse_distance = raw_coarse_distance;
            double phase_a = phases.fine_phase_ch5;
            double phase_b = phases.fine_phase_ch3;

            LOG_DBG("Raw measurements: twr=%.6f, chA=%.6f, chB=%.6f",
                   raw_coarse_distance, phase_a, phase_b);

            /* Try node table first for local measurements */
            ts_series_t *s_twr = NULL;
            ts_series_t *s_pa = NULL;
            ts_series_t *s_pb = NULL;
            ts_series_t *s_d_diff_node = NULL;

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
            s_twr = node_table_get_mm_twr_series(responder_id);
            s_pa  = node_table_get_mm_phase_series(responder_id, 0);
            s_pb  = node_table_get_mm_phase_series(responder_id, 1);
            s_d_diff_node = node_table_get_mm_d_diff_series(responder_id);
#endif
            /* Fallback to global store if node table series not available */
            if (!s_twr || !s_pa || !s_pb) {
                ts_series_key_t key_twr = { initiator_id, responder_id, TS_ID_TWR_TOF };
                ts_series_key_t key_pa  = { initiator_id, responder_id, (uint16_t)(TS_ID_PHASE_BASE + channel_a) };
                ts_series_key_t key_pb  = { initiator_id, responder_id, (uint16_t)(TS_ID_PHASE_BASE + channel_b) };
                s_twr = ts_get_or_create(&key_twr, mm_get_time_series_size());
                s_pa  = ts_get_or_create(&key_pa, mm_get_time_series_size());
                s_pb  = ts_get_or_create(&key_pb, mm_get_time_series_size());
                s_d_diff_node = NULL;
            }

            // Critical time series required for ambiguity resolution
            if (!s_twr || !s_pa || !s_pb) {
                LOG_ERR("Failed to create time series for pair %u-%u (twr=%p, pa=%p, pb=%p)",
                       initiator_id, responder_id, (void*)s_twr, (void*)s_pa, (void*)s_pb);
                continue;
            }

            // Append raw measurements to time series.
            // Phase filtering is handled downstream by ts_filter_phase()
            // in resolve_ambiguity_dual_channel_ts() -- do NOT filter here
            // to avoid double-filtering.
            ts_append(s_twr, coarse_distance);
            ts_append(s_pa, phase_a);
            ts_append(s_pb, phase_b);

            phases.fine_phase_ch5 = phase_a;
            phases.fine_phase_ch3 = phase_b;

            // Resolve ambiguity using dual-channel method
            struct mm_distance_result mm_result;

            // Choose algorithm based on configuration

            // Use MATLAB-style algorithm (default)
            {
#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
                uint8_t win_diff = node_table_get_mm_filter_window_size();
                uint8_t poly_diff = node_table_get_mm_filter_poly_order();
                uint8_t win_phase = node_table_get_mm_phase_filter_window_size();
                uint8_t poly_phase = node_table_get_mm_phase_filter_poly_order();
                uint8_t win_twr = node_table_get_mm_twr_filter_window_size();
                uint8_t poly_twr = node_table_get_mm_twr_filter_poly_order();
#elif defined(CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY)
                size_t win_diff = CONFIG_SAVITZKY_GOLAY_WINDOW_SIZE;
                uint8_t poly_diff = CONFIG_SAVITZKY_GOLAY_POLY_ORDER;
                size_t win_phase = win_diff;
                uint8_t poly_phase = poly_diff;
                size_t win_twr = win_diff;
                uint8_t poly_twr = poly_diff;
#else
                size_t win_diff = CONFIG_MM_RANGING_FILTER_WINDOW_SIZE;
                uint8_t poly_diff = 0;
                size_t win_phase = win_diff;
                uint8_t poly_phase = poly_diff;
                size_t win_twr = win_diff;
                uint8_t poly_twr = poly_diff;
#endif
                ret = resolve_ambiguity_dual_channel_ts(s_twr, s_pa, s_pb,
                                                        s_d_diff_node,
                                                        initiator_id, responder_id,
                                                        channel_a, channel_b,
                                                        win_phase, poly_phase,
                                                        win_twr, poly_twr,
                                                        win_diff, poly_diff,
                                                        &mm_result);
            }
            if (ret < 0) {
                LOG_ERR("Ambiguity resolution failed for pair %u-%u: %d",
                       initiator_id, responder_id, ret);
                continue;
            }

            // Validate the result
            if (!validate_mm_distance_result(&mm_result, coarse_distance,
                                           0.1)) {
                LOG_WRN("MM distance result validation failed for pair %u-%u",
                       initiator_id, responder_id);
                // Zero out the MM distance so consumers know it's invalid
                mm_result.distance_mm = 0.0f;
            }

            // Store the measurement
            if (measurement_count >= max_measurements) {
                LOG_WRN("Not enough space to store all measurements");
                return measurement_count;
            }

            struct measurement_mm *m = &measurements[measurement_count];
            m->ranging_initiator_id = initiator_id;
            m->ranging_responder_id = responder_id;
            m->tof = prop_ch_a.tof;
            m->cfo = prop_ch_a.cfo;
            m->tof_quality = 0;

            // Store phase information
            m->phase_init = twr_ch_a.phase_init;
            m->phase_resp = twr_ch_a.phase_resp;
            m->phase_final = twr_ch_a.phase_final;
            m->phase_post_final = twr_ch_a.phase_post_final;
            m->d_diff = mm_result.d_diff_smoothed;
            m->d_diff_minus = mm_result.d_diff_minus;
            m->d_diff_plus = mm_result.d_diff_plus;

            // Store dual-channel phase data (filtered from ambiguity resolution)
            m->phase_chan_5 = mm_result.phase_a_filtered;
            m->phase_chan_3 = mm_result.phase_b_filtered;
            m->delta_phi = mm_result.phase_diff_smoothed;

            // Store processed phase data - per channel
            m->coarse_phase_ch5 = phases.coarse_phase_ch5;
            m->fine_phase_ch5 = phases.fine_phase_ch5;
            m->residual_phase_raw_ch5 = phases.residual_phase_raw_ch5;
            m->coarse_phase_ch3 = phases.coarse_phase_ch3;
            m->fine_phase_ch3 = phases.fine_phase_ch3;
            m->residual_phase_raw_ch3 = phases.residual_phase_raw_ch3;

            // Backward compatibility (using ch5 values)
            m->coarse_phase = phases.coarse_phase_ch5;
            m->fine_phase = phases.fine_phase_ch5;
            m->residual_phase_raw = phases.residual_phase_raw_ch5;
            m->filtered_phase = mm_result.phase_diff_smoothed;

            // Store distance results (both raw and filtered)
            m->distance_twr = mm_result.distance_twr_smoothed;  // Use smoothed TWR distance from ambiguity resolution
            m->distance_mm = mm_result.distance_mm;
            m->distance_twr_raw = (float)uncorrected_distance;
            m->range_bias_m = (float)applied_bias_m;
            // Store raw TWR distance for comparison (if needed in measurement struct)
            // m->raw_distance_twr = raw_coarse_distance;  // Uncomment if field exists

            // Store channel information (actual channel numbers)
            m->channel_5 = (channel_a == 5);  // Boolean: true if channel A is channel 5
            m->channel_3 = (channel_b == 3);  // Boolean: true if channel B is channel 3

            measurement_count++;

            LOG_INF("Dual-channel measurement %u-%u: twr_raw=%.6f, twr_corrected=%.6f, bias=%.6f, twr_smooth=%.6f, mm=%.6f, delta_phi=%.6f",
                   initiator_id, responder_id, uncorrected_distance, raw_coarse_distance, applied_bias_m,
                   mm_result.distance_twr_smoothed, (double)mm_result.distance_mm,
                   mm_result.phase_diff_smoothed);
        }
    }

    return measurement_count;
}
