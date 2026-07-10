#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/lib/ranging/twr.h>
#include <app/lib/node_table/node_table.h>
#include <app/lib/blocks/mtm.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>

LOG_MODULE_REGISTER(ranging_engine);

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


struct dstwr_template
{
    uint16_t ranging_initiator_id, ranging_responder_id;
    uint64_t tx_init, rx_init, tx_resp, rx_resp, tx_final, rx_final;

    // these are only relevant if ranging_initiator_id and ranging_responder_id are both not our own ranging id
    uint64_t local_rx_init, local_rx_resp, local_rx_final;
};

struct transmission_event
{
    int seq;
    uwb_ts_t transmission_timestamp, reception_timestamp;
    uwb_ts_t passive_reception_timestamp;
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

int32_t compute_propagation_time(int32_t initiator_roundtrip, int32_t initiator_reply,
				 int32_t replier_roundtrip, int32_t replier_reply)
{

    return (int32_t)(( ((int64_t) initiator_roundtrip * replier_roundtrip)
            - ((int64_t) initiator_reply * replier_reply))
        /
        ((int64_t) initiator_roundtrip
            +  replier_roundtrip
            +  initiator_reply
            +  replier_reply));
}

float time_to_dist(float tof)
{
    return (float)tof * SPEED_OF_LIGHT_M_PER_UWB_TU;
}

/* struct propagation_time defined in twr.h */

struct propagation_time compute_propagation_time_symmetric(struct dstwr_template *twr)
{
    struct propagation_time prop;

    int64_t initiator_roundtrip = correct_overflow(twr->rx_resp, twr->tx_init);
    int64_t responder_reply_delay = correct_overflow(twr->tx_resp, twr->rx_init);
    int64_t responder_roundtrip = correct_overflow(twr->rx_final, twr->tx_resp);
    int64_t initiator_reply_delay = correct_overflow(twr->tx_final, twr->rx_resp);

    /* Classic symmetric DS-TWR formula:
     * ToF = (R_a * R_b - D_a * D_b) / (R_a + R_b + D_a + D_b) */
    int32_t tof_raw = (int32_t)((initiator_roundtrip * responder_roundtrip
                                  - responder_reply_delay * initiator_reply_delay)
                                 / (initiator_roundtrip + responder_roundtrip
                                    + responder_reply_delay + initiator_reply_delay));

    uint64_t initiator_duration = correct_overflow(twr->tx_final, twr->tx_init);
    uint64_t responder_duration = correct_overflow(twr->rx_final, twr->rx_init);
    double drift_offset = (double)initiator_duration / (double)responder_duration;

    prop.tof = (float)tof_raw;
    prop.cfo = drift_offset;

    return prop;
}

struct propagation_time calculate_propagation_time(struct dstwr_template *twr)
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

struct propagation_time calculate_propagation_time_cfo(struct dstwr_template *twr, double cfo)
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

int get_first_matching_transmission_event_from_digest(void *context, uint16_t sender, uint16_t receiver, int seq, struct transmission_event *transmission)
{
    struct deca_ranging_digest *digest = (struct deca_ranging_digest *)context;
    int transmissionSeq = INT_MAX, receptionSeq = INT_MAX;
    dwt_ts_t transmission_timestamp = 0, reception_timestamp = 0;

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
        struct deca_tagged_timestamp *frame_timestamps = NULL;
        int rx_ts_count;

        if (frame == NULL || container->status == DECA_FRAME_REJECTED) {
		continue;
        }

        // first check if we have a received frame from which we can directly take the reception timestamp
        if(container->type == DECA_RECEIVED && container->slot == transmissionSeq && frame->addr == sender) {
            receptionSeq = container->slot;
            reception_timestamp = container->timestamp;
            break;
        }

        rx_ts_count = deca_ranging_frame_get_tagged_timestamps(frame, &frame_timestamps);

        if (frame->addr != receiver) {
            continue;
	}

        for (size_t j = 0; j < rx_ts_count; j++) {
            struct deca_tagged_timestamp ts = frame_timestamps[j];

            uint16_t slot = ts.slot;
            if (slot < seq) {
                continue;
            }
            if (ts.addr == sender && slot == transmissionSeq) {
                receptionSeq = slot;
		reception_timestamp = from_packed_uwb_ts(ts.ts);
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
    transmission->passive_reception_timestamp = UINT64_MAX; // TODO, we need to get our uwb address to do this into this module

    return transmission->seq;
}

// in memory lookup
pair_view_t createDigestPairView(const struct deca_ranging_digest *digest)
{
    pair_view_t view;
    view.context = (void *)digest;
    view.getFirstMatchingTransmissionEventFrom = get_first_matching_transmission_event_from_digest;

    return view;
}

bool add_address_if_not_in_list(deca_short_addr_t *addrs, int length, deca_short_addr_t addr)
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
        struct deca_tagged_timestamp *frame_timestamps = NULL;
        int rx_ts_count;

        if(frame == NULL || container->status == DECA_FRAME_REJECTED) {
            continue;
        }

	deca_short_addr_t frame_id = frame->addr;
        rx_ts_count = deca_ranging_frame_get_tagged_timestamps(frame, &frame_timestamps);

	length += add_address_if_not_in_list(addrs, length, frame_id);
	if (length >= max_length) {
            LOG_WRN("Ranging digest contains more than %d unique addresses", max_length);
            return length;
        }

        int err = 0;
        for (int j = 0; j < rx_ts_count; j++) {
            struct deca_tagged_timestamp ts = frame_timestamps[j];
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

bool single_sided_complete(const struct dstwr_template *twr_template)
{
    return twr_template->tx_init != UINT64_MAX && twr_template->rx_init != UINT64_MAX &&
        twr_template->tx_resp != UINT64_MAX && twr_template->rx_resp != UINT64_MAX;
}

bool double_sided_complete(const struct dstwr_template *twr_template)
{
    return single_sided_complete(twr_template) && twr_template->tx_final != UINT64_MAX && twr_template->rx_final != UINT64_MAX;
}

struct dstwr_template create_dstwr_template(deca_short_addr_t initiator_id, deca_short_addr_t responder_id)
{
    struct dstwr_template twr_template;
    twr_template.ranging_initiator_id = initiator_id;
    twr_template.ranging_responder_id = responder_id;
    twr_template.local_rx_init = UINT64_MAX;
    twr_template.local_rx_resp = UINT64_MAX;
    twr_template.local_rx_final = UINT64_MAX;
    twr_template.tx_init = UINT64_MAX;
    twr_template.rx_init = UINT64_MAX;
    twr_template.tx_resp = UINT64_MAX;
    twr_template.rx_resp = UINT64_MAX;
    twr_template.tx_final = UINT64_MAX;
    twr_template.rx_final = UINT64_MAX;

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

    if ((slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, initiator_id, responder_id, 0, &initiation)) >= 0) {
        DSTWRTemplate.tx_init = initiation.transmission_timestamp;
        DSTWRTemplate.rx_init = initiation.reception_timestamp;
	DSTWRTemplate.local_rx_init = initiation.passive_reception_timestamp;
    }

    if (slot >= 0 && (slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, responder_id, initiator_id, slot + 1, &response)) >= 0) {
        DSTWRTemplate.tx_resp = response.transmission_timestamp;
        DSTWRTemplate.rx_resp = response.reception_timestamp;
	DSTWRTemplate.local_rx_resp = response.passive_reception_timestamp;
    }

    if (slot >= 0 && (slot = pairs.getFirstMatchingTransmissionEventFrom(pairs.context, initiator_id, responder_id, slot + 1, &finalization)) >= 0) {
        DSTWRTemplate.tx_final = finalization.transmission_timestamp;
        DSTWRTemplate.rx_final = finalization.reception_timestamp;
	DSTWRTemplate.local_rx_final = finalization.passive_reception_timestamp;
    }

    return DSTWRTemplate;
}

double frequencyOffsetFromDSTWR(struct dstwr_template *twr)
{
    uint64_t initiator_duration   = correct_overflow(twr->tx_final, twr->tx_init);
    uint64_t responder_duration = correct_overflow(twr->rx_final, twr->rx_init);
    return (double)((int64_t)initiator_duration - (int64_t)responder_duration) /
                (double)(responder_duration);
}

int twr_extract_timestamps(const struct deca_ranging_digest *digest,
                           uint16_t initiator_id, uint16_t responder_id,
                           struct node_twr_timestamps *out)
{
    struct dstwr_template twr = get_dstwr_for_node_pair(digest, initiator_id, responder_id);

    if (!double_sided_complete(&twr)) {
        return -1;
    }

    out->initiator_id = initiator_id;
    out->responder_id = responder_id;
    out->tx_init  = (int64_t)twr.tx_init;
    out->rx_init  = (int64_t)twr.rx_init;
    out->tx_resp  = (int64_t)twr.tx_resp;
    out->rx_resp  = (int64_t)twr.rx_resp;
    out->tx_final = (int64_t)twr.tx_final;
    out->rx_final = (int64_t)twr.rx_final;

    return 0;
}

int frequencyOffsetFromCFO(const struct deca_ranging_digest *digest, deca_short_addr_t transmitter, bool average, double *offset)
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

int estimate_distances(const struct device *dev, const struct deca_ranging_digest *digest, uint8_t channel, bool estimateOnlyLocal,
			deca_short_addr_t local_addr, struct measurement *measurements,
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
                prop = compute_propagation_time_symmetric(&twr);
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

                struct measurement m;
                m.ranging_initiator_id = twr.ranging_initiator_id;
                m.ranging_responder_id = twr.ranging_responder_id;

                /* Apply distance-level range bias only if mode is DISTANCE */
                float corrected_tof = prop.tof;
                if (mtm_get_bias_correction_mode() == BIAS_CORRECTION_DISTANCE) {
                    int prf = get_prf_from_device(dev);
                    if (prf > 0) {
                        const uwb_driver_t *driver = uwb_driver_get(dev);
                        if (driver && driver->get_range_bias) {
                            double raw_distance = prop.tof * SPEED_OF_LIGHT_M_PER_UWB_TU;
                            double bias = driver->get_range_bias(dev, channel, raw_distance, prf);
                            double corrected_distance = raw_distance - bias;
                            corrected_tof = ((float)corrected_distance / SPEED_OF_LIGHT_M_PER_UWB_TU);
                            LOG_INF("Range bias correction [twr]: raw=%.3fm, bias=%.3fm, corrected=%.3fm",
                                   raw_distance, bias, corrected_distance);
                        }
                    }
                }
                m.tof = corrected_tof;
                m.cfo = prop.cfo;
                m.tof_quality = 0;

                measurements[measurement_count++] = m;
            }


        }
    }

    return measurement_count;
}
