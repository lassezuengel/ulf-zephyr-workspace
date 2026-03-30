#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/ieee802154/uwb_frame_utils.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */

#include <math.h>
#include <zephyr/logging/log.h>
#ifdef CONFIG_SEGGER_SYSTEMVIEW
#include <SEGGER_SYSVIEW.h>

// MTM Protocol SystemView Markers (0x7200-0x721F range to avoid collision with MM)
#define MTM_SYSVIEW_MARKER_SLOT_RX              0x7200
#define MTM_SYSVIEW_MARKER_SLOT_TX              0x7201
#define MTM_SYSVIEW_MARKER_SLOT_LOAD            0x7202
#define MTM_SYSVIEW_MARKER_SLOT_IDLE            0x7203
#define MTM_SYSVIEW_MARKER_PHY_PROGRAM_TX       0x7204
#define MTM_SYSVIEW_MARKER_PHY_PROGRAM_RX       0x7205
#define MTM_SYSVIEW_MARKER_RX_FRAME_HANDLING    0x7206
#define MTM_SYSVIEW_MARKER_RX_READ_DIAGNOSTICS  0x7207
#define MTM_SYSVIEW_MARKER_RX_BUSY_WAIT         0x7208
#define MTM_SYSVIEW_MARKER_RX_READ_FRAME        0x7209
#define MTM_SYSVIEW_MARKER_RX_BUFFER_SWITCH     0x720A
#define MTM_SYSVIEW_MARKER_RX_CHECKSUM          0x720B
#define MTM_SYSVIEW_MARKER_RX_STORE_TIMESTAMP   0x720C
#define MTM_SYSVIEW_MARKER_RX_CIR_READ          0x720D
#define MTM_SYSVIEW_MARKER_RX_CIR_HANDLER       0x720E
#define MTM_SYSVIEW_MARKER_TX_PREPARE           0x720F
#define MTM_SYSVIEW_MARKER_TX_SEEK_SLOT         0x7210
#define MTM_SYSVIEW_MARKER_TX_CALC_TIMESTAMP    0x7211
#define MTM_SYSVIEW_MARKER_TX_BUILD_FRAME       0x7212
#define MTM_SYSVIEW_MARKER_TX_LOAD_PAYLOAD      0x7213
#define MTM_SYSVIEW_MARKER_TX_LOAD_TIMESTAMPS   0x7214
#define MTM_SYSVIEW_MARKER_TX_CHECKSUM          0x7215
#define MTM_SYSVIEW_MARKER_TX_SETUP_FRAME       0x7216
#define MTM_SYSVIEW_MARKER_IRQ_WAIT             0x7217
#define MTM_SYSVIEW_MARKER_IRQ_RX_PROCESS       0x7218
#define MTM_SYSVIEW_MARKER_IRQ_TX_PROCESS       0x7219
#define MTM_SYSVIEW_MARKER_IRQ_TIMEOUT          0x721A

static void mtm_sysview_ensure_named(void)
{
    static bool named;

    if (!named) {
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_SLOT_RX, "mtm_slot_rx");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_SLOT_TX, "mtm_slot_tx");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_SLOT_LOAD, "mtm_slot_load");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_SLOT_IDLE, "mtm_slot_idle");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_PHY_PROGRAM_TX, "mtm_phy_program_tx");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_PHY_PROGRAM_RX, "mtm_phy_program_rx");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_RX_FRAME_HANDLING, "mtm_rx_frame_handling");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_RX_READ_DIAGNOSTICS, "mtm_rx_read_diagnostics");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_RX_BUSY_WAIT, "mtm_rx_busy_wait");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_RX_READ_FRAME, "mtm_rx_read_frame");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_RX_BUFFER_SWITCH, "mtm_rx_buffer_switch");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_RX_CHECKSUM, "mtm_rx_checksum");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_RX_STORE_TIMESTAMP, "mtm_rx_store_timestamp");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_RX_CIR_READ, "mtm_rx_cir_read");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_RX_CIR_HANDLER, "mtm_rx_cir_handler");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_TX_PREPARE, "mtm_tx_prepare");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_TX_SEEK_SLOT, "mtm_tx_seek_slot");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_TX_CALC_TIMESTAMP, "mtm_tx_calc_timestamp");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_TX_BUILD_FRAME, "mtm_tx_build_frame");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_TX_LOAD_PAYLOAD, "mtm_tx_load_payload");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_TX_LOAD_TIMESTAMPS, "mtm_tx_load_timestamps");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_TX_CHECKSUM, "mtm_tx_checksum");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_TX_SETUP_FRAME, "mtm_tx_setup_frame");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_IRQ_WAIT, "mtm_irq_wait");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_IRQ_RX_PROCESS, "mtm_irq_rx_process");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_IRQ_TX_PROCESS, "mtm_irq_tx_process");
        SEGGER_SYSVIEW_NameMarker(MTM_SYSVIEW_MARKER_IRQ_TIMEOUT, "mtm_irq_timeout");
        named = true;
    }
}
#else
static inline void mtm_sysview_ensure_named(void) {}
#endif

LOG_MODULE_REGISTER(ranging_mtm, LOG_LEVEL_INF);

// Debug logging removed to prevent buffer overflows

#define RANGE_CORR_MAX_RSSI (-61)
#define RANGE_CORR_MIN_RSSI (-93)

static int8_t range_bias_by_rssi[RANGE_CORR_MAX_RSSI-RANGE_CORR_MIN_RSSI+1] = {
    -23, // -61dBm (-11 cm)
    -23, // -62dBm (-10.75 cm)
    -22, // -63dBm (-10.5 cm)
    -22, // -64dBm (-10.25 cm)
    -21, // -65dBm (-10.0 cm)
    -21, // -66dBm (-9.65 cm)
    -20, // -67dBm (-9.3 cm)
    -19, // -68dBm (-8.75 cm)
    -17, // -69dBm (-8.2 cm)
    -16, // -70dBm (-7.55 cm)
    -15, // -71dBm (-6.9 cm)
    -13, // -72dBm (-6.0 cm)
    -11, // -73dBm (-5.1 cm)
    -8, // -74dBm (-3.9 cm)
    -6, // -75dBm (-2.7 cm)
    -3, // -76dBm (-1.35 cm)
    0, // -77dBm (0.0 cm)
    2, // -78dBm (1.05 cm)
    4, // -79dBm (2.1 cm)
    6, // -80dBm (2.8 cm)
    7, // -81dBm (3.5 cm)
    8, // -82dBm (3.85 cm)
    9, // -83dBm (4.2 cm)
    10, // -84dBm (4.55 cm)
    10, // -85dBm (4.9 cm)
    12, // -86dBm (5.55 cm)
    13, // -87dBm (6.2 cm)
    14, // -88dBm (6.65 cm)
    15, // -89dBm (7.1 cm)
    16, // -90dBm (7.35 cm)
    16, // -91dBm (7.6 cm)
    17, // -92dBm (7.85 cm)
    17, // -93dBm (8.1 cm)
};

// we will directly insert the header into the buffer, because of the limitiations of using nrfx spi
// directly .Write into this buffer starting from offset 0, but read into it simultanously from
// offset 1
static uint8_t cir_acc_mem[CONFIG_DWT_MTM_CIR_BUFFER_SIZE];


struct uwb_ranging_timing mtm_ranging_conf = {
	.phy_activate_rx_delay_us = 128 + 36, //+36 // give receiver some more time to start up
	.phase_setup_delay_us = 200,
	.round_setup_delay_us = 200,
	.preamble_timeout = (128/8),
	/* .preamble_timeout = (64/8), */
	.preamble_chunk_duration_us = 8, // 8 symbols per cross-correlated chunk
};

struct mtm_round_timing {
	uint32_t round_init_us, initiation_frame_us, init_round_setup_us,
		prepare_tx_us, prog_rx_ts_us, frame_handling_base_us, frame_handling_per_timestamp_us, irq_handling_us,
		voodo_per_timestamp_us, voodo_base_us;

};

// TODO: initiation frame is for now only calculated in case it is not send, why is it actually so long then??
struct mtm_round_timing timing = {
	.round_init_us = 85,
	.initiation_frame_us = 20,
	.init_round_setup_us = 68,
	.prepare_tx_us = 130, // DEPENDENCY ON NODES
	.prog_rx_ts_us = 41,
	.frame_handling_base_us = 180 + 100, // per timestamp
	.voodo_per_timestamp_us = 15, /* voodo time period since we for some reason read garbage if immeditiay read from the double buffered swing set (WHY oh god WHY the receiver tells me the frame is ready, why are you lying to me) */
	.voodo_base_us = 30 + 20, /* voodo time period since we for some reason read garbage if immeditiay read from the double buffered swing set (WHY oh god WHY the receiver tells me the frame is ready, why are you lying to me) */
	.frame_handling_per_timestamp_us = 12,
	.irq_handling_us = 122, /* old value 91 */
};

int dwt_calculate_slot_duration(const struct device *dev, int timestamps_to_load, int payload_size, int guard_us) {
	struct mtm_round_timing *t = &timing;

	int psdu_len = offsetof(struct deca_ranging_frame, payload) + (timestamps_to_load * sizeof(struct deca_tagged_timestamp)) + payload_size;
	// TODO: Add get_packet_duration_ns to UWB driver API
	int tx_duration_us = dwt_get_pkt_duration_ns(dev, psdu_len)/1000;// + 36;
	int frame_handling_duration_us = t->frame_handling_base_us + t->frame_handling_per_timestamp_us*timestamps_to_load;

	int slot_length_us =  t->prog_rx_ts_us + MAX(tx_duration_us, frame_handling_duration_us) + t->irq_handling_us;

	return slot_length_us + guard_us;
}


static int8_t get_range_bias_by_rssi(int8_t rssi) {
    rssi = MAX(MIN(RANGE_CORR_MAX_RSSI, rssi), RANGE_CORR_MIN_RSSI);
    return range_bias_by_rssi[-(rssi-RANGE_CORR_MAX_RSSI)];
}


// Moved to uwb_utils/uwb_frame_utils.c

static uint16_t calculate_checksum(const struct deca_ranging_frame *frame, size_t frame_size) {
    uint16_t checksum = 0;
    const uint8_t *data = (const uint8_t *)frame;
    size_t checksum_offset = offsetof(struct deca_ranging_frame, checksum);

    // Calculate checksum for data before the checksum field
    for (size_t i = 0; i < checksum_offset; i++) {
        checksum += data[i];
    }

    // Calculate checksum for data after the checksum field
    for (size_t i = checksum_offset + sizeof(uint16_t); i < frame_size; i++) {
        checksum += data[i];
    }

    return checksum;
}

static bool verify_checksum(const struct deca_ranging_frame *frame, size_t frame_size) {
    uint16_t computed_checksum = calculate_checksum(frame, frame_size);
    return computed_checksum == frame->checksum;
}


int deca_ranging(const struct device *dev,
	const struct  deca_ranging_configuration *conf,
	struct deca_ranging_digest *digest)
{
	static struct deca_tagged_timestamp stored_timestamps[UWB_MTM_MAX_FRAMES];


	int ret = 0;
	uwb_irq_state_e irq_state;
	struct uwb_ranging_timing *ranging_conf = &mtm_ranging_conf;
	struct deca_schedule *schedule = conf->schedule;

	uwb_ts_t round_start_dw_ts, slot_start_ts;
	uwb_antenna_delay_t antenna_delay;
	int stored_timestamp_count;


#if ANALYZE_DWT_TIMING
	SW_RESET(TIME_TILL_FIRST_FRAME);
	reset_timing_logs();
	timing_start();

	SW_START_ONCE(TIME_TILL_FIRST_FRAME);
#endif
	const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
	if (!uwb_driver) {
		LOG_ERR("No UWB driver found for device");
		return -ENODEV;
	}

	// --- Prevent execution of multiple ranging tasks
	if (uwb_driver->acquire_device(dev) != 0) {
		LOG_ERR("MTM: Transceiver busy, cannot start ranging");
		return -EBUSY;
	}


	// Get antenna delay values
	uwb_driver->get_antenna_delay(dev, &antenna_delay);

	uwb_driver->disable_txrx(dev);
	uwb_driver->set_frame_filter(dev, 0, 0);
	uwb_driver->align_double_buffering(dev); // for the following execution we require that host and receiver side are aligned

	/* SW_END(ROUND_INIT); */

	/* SW_START(INIT_ROUND_SETUP); */
	// --- PHY setup for ranging round ---
	uwb_driver->setup_frame_timeout(dev, 1200); // max packet transmission will take about 700us
	// !!frame timeouts are expensive as they require a full receiver soft reset, thus we setup a preamble timeout as well!!

	uwb_driver->setup_preamble_timeout(dev, ranging_conf->preamble_timeout + conf->guard_period_us/8);

	/* SW_START(INITIATION_FRAME); */
	// --- Optional: Round Initiation ---
	if (conf->deca_clock_synchronization_instance != NULL) {
		/* round_start_dw_ts = conf->deca_clock_synchronization_instance->local + US_TO_DWT_TS(conf->round_start_offset_us); */
		round_start_dw_ts = conf->deca_round_start_ts;
	} else {
		// use current transmission timestamp and configuration with guard periods
		round_start_dw_ts = uwb_driver->system_timestamp(dev) + uwb_driver->us_to_timestamp(dev, 500);
	}
	/* SW_END(INITIATION_FRAME); */

	// --- lock execute ranging round ---
	slot_start_ts = round_start_dw_ts & UWB_TS_MASK;

	struct deca_ranging_frame *outgoing_frame = NULL;

        stored_timestamp_count = 0;
	uint8_t have_frame = 0, finished_frame = 0;

	uint16_t pkt_len;
	int cfo = 0;

#ifdef CONFIG_SEGGER_SYSTEMVIEW
	// Active marker tracking for error path cleanup
	bool slot_marker_active = false;
	bool rx_frame_handling_active = false;
	bool tx_prepare_active = false;
	bool irq_rx_process_active = false;
	bool rx_cir_read_active = false;
	bool irq_timeout_active = false;
	enum slot_type active_slot_type = DENSE_IDLE_SLOT;
#endif

	// -- do one more iteration because of double buffered operation --
	/* SW_END(INIT_ROUND_SETUP); */
	for(size_t s = 0; s < schedule->slot_count + 1; s++) {
		struct deca_slot *current_slot = NULL;
		enum slot_type type;

		if(s < schedule->slot_count) {
			current_slot = &schedule->slots[s];
			type = current_slot->type;
		} else {
			type = DENSE_IDLE_SLOT;
		}

#ifdef CONFIG_SEGGER_SYSTEMVIEW
		// Start slot-level marker based on type
		mtm_sysview_ensure_named();
		slot_marker_active = true;
		active_slot_type = type;
		switch (type) {
			case DENSE_RX_SLOT:
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_SLOT_RX);
				break;
			case DENSE_TX_SLOT:
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_SLOT_TX);
				break;
			case DENSE_LOAD_TX_BUFFER:
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_SLOT_LOAD);
				break;
			case DENSE_IDLE_SLOT:
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_SLOT_IDLE);
				break;
			default:
				break;
		}
#endif

		// ---- I) kicking of next PHY action ----
		if ((type == DENSE_RX_SLOT || type == DENSE_TX_SLOT)) {
			// --- Decide the PHY action to execute ---
			// --- schedule next PHY action ---
			if(type == DENSE_TX_SLOT) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_PHY_PROGRAM_TX);
#endif
                            uint64_t tx_timestamp = (slot_start_ts
                                + uwb_driver->us_to_timestamp(dev, conf->micro_slot_offset_ns / 1000
                                    + ranging_conf->phy_activate_rx_delay_us
                                    + conf->guard_period_us/2)) & UWB_TS_MASK;
				uwb_driver->start_tx(dev, tx_timestamp);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_PHY_PROGRAM_TX);
#endif
			} else if(type == DENSE_RX_SLOT) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_PHY_PROGRAM_RX);
#endif
				uwb_driver->enable_rx(dev, 0, slot_start_ts & UWB_TS_MASK);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_PHY_PROGRAM_RX);
#endif
			}
			/* SW_END(PROG_RX_TX); */
		}

		// ---- II) double buffered operation -----
		if(have_frame) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_RX_FRAME_HANDLING);
			rx_frame_handling_active = true;
#endif
			// get current system timestamp
			static uint8_t rx_buf[DECA_RANGING_FRAME_MAX_FRAME_SIZE];
			/* SW_START(FRAME_HANDLING); */
			// Note: in slot N we process the frame of slot N-1
			// -- grab frame and frame info struct for storing the received data --
			// Check bounds before accessing frames array
			if (digest->length >= digest->capacity) {
				LOG_ERR("MTM: Frame buffer overflow at slot %u, length=%d, max=%d",
					s - 1, digest->length, digest->capacity);
				ret = -EOVERFLOW;
				goto cleanup;
			}

			struct deca_ranging_frame *incoming_frame = digest->frames[digest->length].frame;
			struct deca_ranging_frame_container *incoming_frame_info = &digest->frames[digest->length];
			digest->length++;

			int8_t bias_correction;
			uwb_rx_diagnostics_t rx_diag;

#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_RX_READ_DIAGNOSTICS);
#endif
			// Get frame length using UWB driver API
			pkt_len = uwb_driver->get_rx_frame_length(dev);

			// Get timestamp and fill diagnostics in one call
			uwb_ts_t reception_ts_raw = uwb_driver->read_rx_timestamp(dev, &rx_diag);

			bias_correction = conf->correct_timestamp_bias ? get_range_bias_by_rssi(rx_diag.rx_level) : 0;
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_READ_DIAGNOSTICS);
#endif

			// read sys stat and save
			/* SAVE_UINT32_TO_LOG(dwt_reg_read_u32(dev, DWT_SYS_STATUS_ID, 0)); */

			// --- read incoming frame and check for validity
			uwb_ts_t current_ts = uwb_driver->system_timestamp(dev);
			uwb_ts_t time_until_reception = uwb_driver->timestamp_to_us(dev, correct_overflow(slot_start_ts, current_ts)) + 30;

			/* SAVE_UINT32_TO_LOG((uint32_t)time_until_reception); */
			if(time_until_reception < 1000) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_RX_BUSY_WAIT);
#endif
				k_busy_wait(time_until_reception);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_BUSY_WAIT);
#endif
			}
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_RX_READ_FRAME);
#endif
			uwb_driver->read_rx_frame(dev, rx_buf, pkt_len, 0);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_READ_FRAME);
			SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_RX_BUFFER_SWITCH);
#endif
			uwb_driver->switch_buffers(dev);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_BUFFER_SWITCH);
#endif

			// --- retrieve incoming frame ---
			// Only check minimum frame size (header + CRC)
			size_t min_frame_size = offsetof(struct deca_ranging_frame, payload) + FRAME_LENGTH_ADDITIONAL;
			if(pkt_len < min_frame_size) {
				LOG_ERR("Frame too small: %u bytes (need at least %u)", pkt_len, (unsigned)min_frame_size);
				ret = -EIO;
				goto cleanup;
			}

			memcpy(incoming_frame, rx_buf, pkt_len-FRAME_LENGTH_ADDITIONAL);
			/* SAVE_UINT32_TO_LOG(pkt_len); */

			if(incoming_frame->msg_id != UWB_MTM_RANGING_FRAME_ID || pkt_len <= offsetof(struct deca_ranging_frame, payload)) {
				// Only log on error paths to avoid timing disruption
				LOG_ERR("Invalid ranging frame: msg_id=%02x (expected %02x), frame_size=%u, addr=%04x",
					incoming_frame->msg_id, UWB_MTM_RANGING_FRAME_ID, pkt_len, incoming_frame->addr);
				incoming_frame_info->status = DECA_FRAME_REJECTED;
				incoming_frame_info->type = DECA_RECEIVED;
			} else {
				uwb_ts_t reception_ts = reception_ts_raw;
				// frame and container already allocated together above

				incoming_frame_info->status = DECA_FRAME_OKAY;
				incoming_frame_info->timestamp = reception_ts - bias_correction;
				incoming_frame_info->type = DECA_RECEIVED;
				incoming_frame_info->slot = s - 1;
				incoming_frame_info->cir_pwr = rx_diag.cir_pwr;
				incoming_frame_info->rx_pacc = rx_diag.rx_pacc;
				incoming_frame_info->fp_index = rx_diag.fp_index;
				incoming_frame_info->fp_ampl1 = rx_diag.fp_ampl1;
				incoming_frame_info->fp_ampl2 = rx_diag.fp_ampl2;
				incoming_frame_info->fp_ampl3 = rx_diag.fp_ampl3;
				incoming_frame_info->std_noise = rx_diag.std_noise;
				incoming_frame_info->rx_level = rx_diag.rx_level;

				if(conf->cfo) {
                                    // warning this has to be adjusted for other data rates. -1e6 * (((((float) cfo)) / (((float) (2 * 1024) / 998.4e6) * 2^17)) / 6489.6e6)
                                    incoming_frame_info->cfo_ppm = (float)cfo * -0.000573121584378756f;
				}

#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_RX_CHECKSUM);
#endif
				/* verify checksum */
				bool frame_valid = verify_checksum(incoming_frame, pkt_len-FRAME_LENGTH_ADDITIONAL);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_CHECKSUM);
#endif

				// --- store reception timestamp ---
				if(!frame_valid || (conf->reject_frames && ((int) rx_diag.fp_index >> 6) <= conf->fp_index_threshold)) {
					// this informs the upper layer that the frame was rejected and not included
					incoming_frame_info->status = DECA_FRAME_REJECTED;
				} else {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
					SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_RX_STORE_TIMESTAMP);
#endif
					struct deca_tagged_timestamp *curr_rx_ts = &stored_timestamps[stored_timestamp_count];
					to_packed_uwb_ts(curr_rx_ts->ts, reception_ts - bias_correction);
					curr_rx_ts->addr = incoming_frame->addr;
					curr_rx_ts->slot = s - 1;

					stored_timestamp_count++;
#ifdef CONFIG_SEGGER_SYSTEMVIEW
					SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_STORE_TIMESTAMP);
#endif
				}
			}

			/* k_sem_take(&ctx->dev_lock, K_FOREVER); */
			/* dwt_switch_buffers(dev); */
			/* k_sem_give(&ctx->dev_lock); */
			have_frame = 0;
			finished_frame = 1;
			/* SW_END(FRAME_HANDLING); */
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_FRAME_HANDLING);
			rx_frame_handling_active = false;
#endif
		}

		// -- has to happen after frame handling, since the frame handler during double buffered operation may still add to the outgoing frame --
		if(type == DENSE_LOAD_TX_BUFFER) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_TX_PREPARE);
			tx_prepare_active = true;
#endif
			/* SW_START(PREPARE_TX); */
			uwb_ts_t current_frame_transmission_ts = slot_start_ts;
			uint16_t future_tx_slot = UINT16_MAX;

#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_TX_SEEK_SLOT);
#endif
			// seek next transmission slot
			for(size_t i = s; i < schedule->slot_count; i++) {
				struct deca_slot *slot = &schedule->slots[i];

				if(slot->type == DENSE_TX_SLOT) {
					future_tx_slot = i;
					break;
				}

				// later in case we want to have differently sized slots, we can further distinguish here
				current_frame_transmission_ts += uwb_driver->us_to_timestamp(dev, slot->duration_us);
			}
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_TX_SEEK_SLOT);
#endif

			// --- in the following phase we will send data that we collected throughout the round
			if(future_tx_slot != UINT16_MAX) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_TX_CALC_TIMESTAMP);
#endif
				/* --- store planned transmission timestamp in frame --- */
				current_frame_transmission_ts =
                                    ((current_frame_transmission_ts + uwb_driver->us_to_timestamp(dev, ranging_conf->phy_activate_rx_delay_us
                                        + conf->guard_period_us/2
                                        + conf->micro_slot_offset_ns / 1000 // convert ns to us
                                        )) & ( (uint64_t) 0xFFFFFFFE00ULL )) + antenna_delay.tx_ant_dly;
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_TX_CALC_TIMESTAMP);
#endif

				// -- already grab a frame for the next upcoming transmission --
				// Check bounds before accessing frames array
				if (digest->length >= digest->capacity) {
					LOG_ERR("MTM: Frame buffer overflow during TX prep at slot %u, length=%d, max=%d",
						future_tx_slot, digest->length, digest->capacity);
					ret = -EOVERFLOW;
					goto cleanup;
				}
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_TX_BUILD_FRAME);
#endif
				outgoing_frame = digest->frames[digest->length].frame;
				struct deca_ranging_frame_container *outgoing_frame_info = &digest->frames[digest->length];
				digest->length++;

				outgoing_frame->msg_id  = UWB_MTM_RANGING_FRAME_ID;
				outgoing_frame->addr = conf->addr;
				outgoing_frame->payload_size = 0;
				outgoing_frame->rx_ts_count = 0;
				to_packed_uwb_ts(outgoing_frame->tx_ts, current_frame_transmission_ts);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_TX_BUILD_FRAME);
#endif

				// check if we have a payload which we should include in this frame
				if(current_slot->meta.payload != NULL && current_slot->meta.payload_size > 0) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
					SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_TX_LOAD_PAYLOAD);
#endif
					// check how much space is available in frame for additional data
					if (current_slot->meta.payload_size > DWT_MTM_MAX_PAYLOAD) {
						LOG_ERR("payload size too high");
					}

					outgoing_frame->payload_size = current_slot->meta.payload_size;
					memcpy(outgoing_frame->payload, current_slot->meta.payload, current_slot->meta.payload_size);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
					SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_TX_LOAD_PAYLOAD);
#endif
				}

				// load timestamps
				if(current_slot->meta.load_stored_timestamps) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
					SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_TX_LOAD_TIMESTAMPS);
#endif
					int remaining_fitting_timestamps = (sizeof(outgoing_frame->payload)-outgoing_frame->payload_size)/sizeof(struct deca_tagged_timestamp);
					remaining_fitting_timestamps = MIN(remaining_fitting_timestamps, stored_timestamp_count);
					memcpy(outgoing_frame->payload + outgoing_frame->payload_size,
						&stored_timestamps[stored_timestamp_count - remaining_fitting_timestamps],
						remaining_fitting_timestamps * sizeof(struct deca_tagged_timestamp));
					outgoing_frame->rx_ts_count = remaining_fitting_timestamps;
					stored_timestamp_count -= remaining_fitting_timestamps;
#ifdef CONFIG_SEGGER_SYSTEMVIEW
					SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_TX_LOAD_TIMESTAMPS);
#endif
				}

#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_TX_CHECKSUM);
#endif
				/* calculate checksum */
				size_t frame_size = offsetof(struct deca_ranging_frame, payload)
					+ outgoing_frame->payload_size + (outgoing_frame->rx_ts_count * sizeof(struct deca_tagged_timestamp));
				uint16_t checksum = calculate_checksum(outgoing_frame, frame_size);
				outgoing_frame->checksum = checksum;
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_TX_CHECKSUM);
#endif

				// -- populate frame info struct (already allocated above) --
				outgoing_frame_info->status = DECA_FRAME_OKAY;
				outgoing_frame_info->timestamp = current_frame_transmission_ts;
				outgoing_frame_info->slot = future_tx_slot;
				outgoing_frame_info->type = DECA_TRANSMITTED;

#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_TX_SETUP_FRAME);
#endif
				// --- Finally load frame into transmission buffer ---
				uwb_driver->setup_tx_frame(dev, (uint8_t*) outgoing_frame, frame_size);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_TX_SETUP_FRAME);
#endif
			}

			/* SW_END(PREPARE_TX); */
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_TX_PREPARE);
			tx_prepare_active = false;
#endif
		}

		// ---- II) Joining with PHY (only relevant if our current slot is a rx or tx operation) -----
		if ((type == DENSE_RX_SLOT || type == DENSE_TX_SLOT)) {
			/* SW_START(IRQ_WAIT_DELAY); */
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_IRQ_WAIT);
#endif
			// Release device lock before waiting for IRQ to avoid deadlock
			uwb_driver->release_device(dev);
			irq_state = uwb_driver->wait_for_irq(dev);
			// Reacquire device lock after IRQ
			uwb_driver->acquire_device(dev);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
			SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_IRQ_WAIT);
#endif
			/* SW_END(IRQ_WAIT_DELAY); */

			if(irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_IRQ_TIMEOUT);
				irq_timeout_active = true;
#endif
				printk("MTM: Frame wait timeout at slot %u (type=%u), round_start_ts=%llu, slot_start_ts=%llu\n",
					s, type, round_start_dw_ts, slot_start_ts);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_IRQ_TIMEOUT);
				irq_timeout_active = false;
#endif
				ret = -ETIMEDOUT;
				goto cleanup;
			} else if( irq_state == UWB_IRQ_PREAMBLE_DETECT_TIMEOUT) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_IRQ_TIMEOUT);
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_IRQ_TIMEOUT);
#endif
				/* SAVE_UINT32_TO_LOG(s | 0xA000); // | sys_state */
			} else if(irq_state == UWB_IRQ_RX) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_IRQ_RX_PROCESS);
				irq_rx_process_active = true;
#endif
				have_frame = 1;

				// ----- NON DOUBLE BUFFERED REGS ------
				// Read registers not in double buffere swinging register set

				if(conf->cfo) {
                                    cfo = uwb_driver->read_carrier_integrator(dev);
				}

				if(false && current_slot->meta.with_cir_handler && conf->cir_handler != NULL) {
                                    // first we do a bulk extract of the impulse memory
                                    /* dwt_register_read(dev, DWT_ACC_MEM_ID, 0, sizeof(cir_acc_mem), cir_acc_mem); */

                                    uint16_t to_index, from_index;
                                    if(current_slot->meta.only_first_path) {
                                        uwb_rx_diagnostics_t diag;
                                        uwb_driver->read_diagnostics(dev, &diag);
                                        int fp_index = (int) (diag.fp_index >> 6);
                                        from_index = fp_index - current_slot->meta.from_index;
                                        to_index   = fp_index + current_slot->meta.to_index;
                                    } else {
                                        from_index = current_slot->meta.from_index;
                                        to_index = current_slot->meta.to_index;
                                    }

#ifdef CONFIG_SEGGER_SYSTEMVIEW
                                    SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_RX_CIR_READ);
                                    rx_cir_read_active = true;
#endif
                                    uwb_driver->enable_cir_access(dev);
                                    uint16_t offset = from_index * 4;
                                    uint16_t length = (to_index - from_index + 1) * 4;

                                    if(to_index - from_index > CONFIG_DWT_MTM_CIR_BUFFER_SIZE) {
                                        LOG_ERR("MTM: Too many CIR samples requested at slot %u: from=%u, to=%u (max=%d)",
                                            s, from_index, to_index, CONFIG_DWT_MTM_CIR_BUFFER_SIZE);
                                        ret = -EIO;
                                        goto cleanup;
                                    }

                                    // Use abstracted CIR access instead of direct SPI
                                    uwb_driver->read_cir_data(dev, cir_acc_mem + 4, offset, length);

                                    /*
                                      We are not able to buffer the CIR for every reception in
                                      the round. Therefore, we directly call a upper-layer
                                      handler for the cir This might also be useful in the
                                      future since the upper layer might want to decide whether
                                      to throw away the transmission based on the cir.
                                    */
                                    uwb_driver->disable_cir_access(dev);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
                                    SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_CIR_READ);
                                    rx_cir_read_active = false;
                                    SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_RX_CIR_HANDLER);
#endif
                                    conf->cir_handler(s, cir_acc_mem+4, (to_index - from_index + 1)*4);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
                                    SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_CIR_HANDLER);
#endif
				}
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_IRQ_RX_PROCESS);
				irq_rx_process_active = false;
#endif
			} else if(irq_state == UWB_IRQ_TX) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
				SEGGER_SYSVIEW_OnUserStart(MTM_SYSVIEW_MARKER_IRQ_TX_PROCESS);
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_IRQ_TX_PROCESS);
#endif
			} else if(irq_state == UWB_IRQ_ERR) {
				/* SW_END(IRQ_WORK_HANDLER); */
				/* handling rx errors takes a longer time than the other
				   events, since a receiver reset has to be performed.  thus
				   if we don't return out of this round after a error event,
				   we should add about 90 microseconds to the slot duration,
				   in order to not miss subsequent frames. */
				/* SAVE_UINT32_TO_LOG(s | 0xE000); */
				/* SAVE_UINT32_TO_LOG(last_rx_err_stat); */
			} else if(irq_state == UWB_IRQ_HALF_DELAY_WARNING) {
				uwb_ts_t curr_ts = uwb_driver->system_timestamp(dev);
				uint32_t elapsed_us = uwb_driver->timestamp_to_us(dev, curr_ts - slot_start_ts);
				printk("MTM: Half delay warning at slot %u (type=%u), elapsed=%u us, slot_duration=%u us\n",
					s, type, elapsed_us, current_slot->duration_us);
				ret = -EOVERFLOW;
				goto cleanup;
			} else if(irq_state != UWB_IRQ_NONE) {
				printk("MTM: Unexpected IRQ state %d at slot %u (type=%u)\n",
					irq_state, s, type);
			}
			/* SW_END(IRQ_HANDLING); */
		}

#ifdef CONFIG_SEGGER_SYSTEMVIEW
		// End slot-level marker based on type
		switch (type) {
			case DENSE_RX_SLOT:
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_SLOT_RX);
				break;
			case DENSE_TX_SLOT:
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_SLOT_TX);
				break;
			case DENSE_LOAD_TX_BUFFER:
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_SLOT_LOAD);
				break;
			case DENSE_IDLE_SLOT:
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_SLOT_IDLE);
				break;
			default:
				break;
		}
		slot_marker_active = false;
#endif

		// -- Currently always iterate by slot_duration --
		if (current_slot) {
			slot_start_ts = (slot_start_ts + uwb_driver->us_to_timestamp(dev, current_slot->duration_us)) & UWB_TS_MASK;
		}
	}

	// digest->length is updated as frames are added

  cleanup:
#ifdef CONFIG_SEGGER_SYSTEMVIEW
	// Clean up any active markers before returning
	if (rx_cir_read_active) {
		SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_CIR_READ);
		rx_cir_read_active = false;
	}
	if (irq_rx_process_active) {
		SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_IRQ_RX_PROCESS);
		irq_rx_process_active = false;
	}
	if (irq_timeout_active) {
		SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_IRQ_TIMEOUT);
		irq_timeout_active = false;
	}
	if (rx_frame_handling_active) {
		SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_RX_FRAME_HANDLING);
		rx_frame_handling_active = false;
	}
	if (tx_prepare_active) {
		SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_TX_PREPARE);
		tx_prepare_active = false;
	}
	if (slot_marker_active) {
		switch (active_slot_type) {
			case DENSE_RX_SLOT:
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_SLOT_RX);
				break;
			case DENSE_TX_SLOT:
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_SLOT_TX);
				break;
			case DENSE_LOAD_TX_BUFFER:
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_SLOT_LOAD);
				break;
			case DENSE_IDLE_SLOT:
				SEGGER_SYSVIEW_OnUserStop(MTM_SYSVIEW_MARKER_SLOT_IDLE);
				break;
			default:
				break;
		}
		slot_marker_active = false;
	}
#endif

	// --- cleanup and release device ---
	uwb_driver->clear_timeouts(dev);
	uwb_driver->release_device(dev);

	// Report preamble timeouts at end to avoid disrupting timing

#if ANALYZE_DWT_TIMING
	timing_stop();
	output_timing_logs();
#endif


/* TODO hmm this is not really nice solution to a deeper problem. In some cases, for instance if we
   are finishing our ranging schedule on a reception, another unexpected interrupt is
   triggered. However, in the last slot, we are not yielding the execution of the current thread (in
   this case the thread which called this function) since we do not call wait_for_phy
   anymore. Normally this is not a problem as no interrupt should arrive, but in cases where one
   arrives the interrupt work item will stall until the calling thread is suspended (for example by
   going to sleep). This than causes the interrupt worker thread to be invoked when the receiver is
   already in sleep, causing various other problems.
 */
	k_yield();

	return ret;
}
