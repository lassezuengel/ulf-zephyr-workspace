#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/ieee802154/uwb_frame_utils.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */

#include <math.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#ifdef CONFIG_SEGGER_SYSTEMVIEW
#include <SEGGER_SYSVIEW.h>
#define MM_SYSVIEW_MARKER_TX_STEP           0x7100
#define MM_SYSVIEW_MARKER_RX_STEP           0x7101
#define MM_SYSVIEW_MARKER_WAIT_IRQ          0x7102
#define MM_SYSVIEW_MARKER_RX_READ_INFO      0x7103
#define MM_SYSVIEW_MARKER_RX_TIMESTAMP      0x7104
#define MM_SYSVIEW_MARKER_RX_DISPATCH       0x7105
#define MM_SYSVIEW_MARKER_RX_FRAME_LENGTH   0x7106
#define MM_SYSVIEW_MARKER_RX_HEADER_COPY    0x7107
#define MM_SYSVIEW_MARKER_RX_PAYLOAD_COPY   0x7108
#define MM_SYSVIEW_MARKER_TX_BUILD_FRAME    0x7109
#define MM_SYSVIEW_MARKER_TX_TIMESTAMP      0x710A
#define MM_SYSVIEW_MARKER_TX_LOAD_FRAME     0x710B
#define MM_SYSVIEW_MARKER_TX_RECORD_INFO    0x710C

static void mm_sysview_ensure_named(void)
{
    static bool named;

    if (!named) {
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_TX_STEP, "mm_tx_step");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_RX_STEP, "mm_rx_step");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_WAIT_IRQ, "mm_wait_irq");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_RX_READ_INFO, "mm_rx_read_info");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_RX_TIMESTAMP, "mm_rx_timestamp");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_RX_DISPATCH, "mm_rx_dispatch");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_RX_FRAME_LENGTH, "mm_rx_frame_len");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_RX_HEADER_COPY, "mm_rx_header_copy");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_RX_PAYLOAD_COPY, "mm_rx_payload_copy");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_TX_BUILD_FRAME, "mm_tx_build");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_TX_TIMESTAMP, "mm_tx_timestamp");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_TX_LOAD_FRAME, "mm_tx_load");
        SEGGER_SYSVIEW_NameMarker(MM_SYSVIEW_MARKER_TX_RECORD_INFO, "mm_tx_record");
        named = true;
    }
}
#else
static inline void mm_sysview_ensure_named(void) {}
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(ranging_mm, LOG_LEVEL_INF);

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


static struct uwb_ranging_timing mtm_ranging_conf = {
	.min_slot_length_us = 0,
	.phy_activate_rx_delay_us = 128 + 36, //+36 // give receiver some more time to start up
	.phase_setup_delay_us = 200,
	.round_setup_delay_us = 200,
	.preamble_timeout = (128/8),
	/* .preamble_timeout = (64/8), */
	.preamble_chunk_duration_us = 8, // 8 symbols per cross-correlated chunk
	.frame_timeout_period = 1200
};

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


/**
 * Helper function to read CIR values at the first path index
 * @param dev Device pointer
 * @param fp_re Pointer to store real part of first path
 * @param fp_im Pointer to store imaginary part of first path
 * @return 0 on success, negative error code on failure
 */
static int read_cir_first_path(const struct device *dev,
                               const uwb_rx_diagnostics_t *diag,
                               int16_t *fp_re, int16_t *fp_im)
{
    const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
    if (!uwb_driver) {
        return -ENODEV;
    }

    if (!diag) {
        return -EINVAL;
    }

    // Use fp_index from diagnostics captured alongside the RX timestamp
    uint16_t fp_index = diag->fp_index >> 6;

    // CIR access is held active by the calling protocol; just compute offsets and fetch
    // Calculate offset for the first path sample
    uint16_t offset = fp_index * 4;

    // Buffer: 1 garbage byte + 4 bytes for one complex sample (2 bytes real + 2 bytes imaginary)
    uint8_t cir_buffer[5];

    // Read 5 bytes total (1 garbage byte + 4 data bytes)
    uwb_driver->read_cir_data(dev, cir_buffer, offset, 5);

    // Skip the first garbage byte
    uint8_t *valid_cir_data = cir_buffer + 1;

    // Extract real and imaginary parts (16-bit signed values)
    *fp_re = ((int16_t *)valid_cir_data)[0];
    *fp_im = ((int16_t *)valid_cir_data)[1];

    return 0;
}

/**
 * Apply RCPHASE correction to CIR real/imaginary values
 * RCPHASE is in units of 360°/(2^7) = 2.8125° per LSB
 * @param fp_re Pointer to real part (will be modified)
 * @param fp_im Pointer to imaginary part (will be modified)
 * @param rcphase RCPHASE value (7-bit unsigned, 0-127)
 */
static void apply_rcphase_correction(int16_t *fp_re, int16_t *fp_im, uint8_t rcphase)
{
#ifdef CONFIG_IEEE802154_DSKIEL_DW1000_RCPHASE_CORRECTION
    // Convert RCPHASE to radians: 360°/(2^7) degrees per LSB
    // RCPHASE is 7-bit unsigned (0-127), each LSB = 2.8125°
    double phase_correction = rcphase*(2*M_PI / 128.0); // Convert to radians

    // Apply rotation: [re', im'] = [re*cos(θ) - im*sin(θ), re*sin(θ) + im*cos(θ)]
    double cos_theta = cos(phase_correction);
    double sin_theta = sin(phase_correction);

    int16_t new_re = (int16_t)((*fp_re * cos_theta) - (*fp_im * sin_theta));
    int16_t new_im = (int16_t)((*fp_re * sin_theta) + (*fp_im * cos_theta));

    *fp_re = new_re;
    *fp_im = new_im;
#endif
}

int deca_ranging_mm(const struct device *dev,
	const struct  deca_ranging_configuration *conf,
	struct deca_ranging_digest *digest)
{
	static struct deca_tagged_mm_timestamp stored_timestamps[UWB_MTM_MAX_FRAMES];


	int ret = 0;
	uwb_irq_state_e irq_state;
	struct uwb_ranging_timing *ranging_conf = &mtm_ranging_conf;
	struct deca_schedule *schedule = conf->schedule;

	const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
	if (!uwb_driver) {
		LOG_ERR("No UWB driver found for device");
		return -ENODEV;
	}

	uwb_ts_t round_start_dw_ts, slot_start_ts;
	uwb_antenna_delay_t antenna_delay;
	uwb_driver->get_antenna_delay(dev, &antenna_delay);
	int stored_timestamp_count;


	// Acquire device access for ranging operation
	ret = uwb_driver->acquire_device(dev);
	if (ret != 0) {
		LOG_ERR("Failed to acquire device for ranging");
		return ret;
	}


	uwb_driver->disable_txrx(dev);
	uwb_driver->set_frame_filter(dev, 0, 0);
	uwb_driver->align_double_buffering(dev); // for the following execution we require that host and receiver side are aligned
	uwb_driver->enable_cir_access(dev); // keep CIR memory mapped for the entire round

	/* SW_END(ROUND_INIT); */

	/* SW_START(INIT_ROUND_SETUP); */
	// --- PHY setup for ranging round ---
	uwb_driver->setup_frame_timeout(dev, 1200); // max packet transmission will take about 700us
	// !!frame timeouts are expensive as they require a full receiver soft reset, thus we setup a preamble timeout as well!!

	uwb_driver->setup_preamble_timeout(dev, ranging_conf->preamble_timeout + conf->guard_period_us/8);

	/* SW_START(INITIATION_FRAME); */
	// --- Optional: Round Initiation ---
	if (conf->deca_clock_synchronization_instance != NULL) {
		/* round_start_dw_ts = conf->deca_clock_synchronization_instance->local + uwb_driver->us_to_timestamp(conf->round_start_offset_us); */
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
        int fp_re, fp_im;

	uint16_t pkt_len;
	int cfo = 0;
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

		// ---- I) kicking of next PHY action ----
		if ((type == DENSE_RX_SLOT || type == DENSE_TX_SLOT)) {
			// --- Decide the PHY action to execute ---
			// --- schedule next PHY action ---
			if(type == DENSE_TX_SLOT) {
				uwb_driver->start_tx(dev, (slot_start_ts
						+ uwb_driver->us_to_timestamp(dev, conf->micro_slot_offset_ns / 1000)
						+ uwb_driver->us_to_timestamp(dev, ranging_conf->phy_activate_rx_delay_us)
						+ uwb_driver->us_to_timestamp(dev, conf->guard_period_us)/2) & UWB_TS_MASK);
			} else if(type == DENSE_RX_SLOT) {
				uwb_driver->enable_rx(dev, 0, slot_start_ts & UWB_TS_MASK);
			}
			/* SW_END(PROG_RX_TX); */
		}

		// ---- II) double buffered operation -----
		if(have_frame) {
			// get current system timestamp
			static uint8_t rx_buf[DECA_RANGING_FRAME_MAX_FRAME_SIZE];
			/* SW_START(FRAME_HANDLING); */
			// Note: in slot N we process the frame of slot N-1
			// -- grab frame and frame info struct for storing the received data --
			if (digest->length >= digest->capacity) {
				LOG_ERR("MM: Frame buffer overflow at slot %u, length=%d, max=%d",
					s - 1, digest->length, digest->capacity);
				ret = -EOVERFLOW;
				goto cleanup;
			}

			struct deca_ranging_frame *incoming_frame = digest->frames[digest->length].frame;
			struct deca_ranging_frame_container *incoming_frame_info = &digest->frames[digest->length];
			digest->length++;

			int8_t rx_level = INT8_MIN, bias_correction;
			uint32_t rx_pacc, cir_pwr;
			uint16_t fp_index;
			float a_const;

			// Read frame info and diagnostics through UWB driver API
			uwb_rx_diagnostics_t rx_diag;
			uwb_driver->read_rx_timestamp(dev, &rx_diag);

			// Get frame length using UWB driver API
			pkt_len = uwb_driver->get_rx_frame_length(dev);

			// --- Use diagnostics from UWB driver API ---
			cir_pwr = rx_diag.cir_pwr;
			rx_pacc = rx_diag.rx_pacc;
			fp_index = rx_diag.fp_index;

			// Use radio constants from UWB driver API for RSSI calculation
			const uwb_radio_constants_t *radio_constants = uwb_driver->get_radio_constants(dev);
			// For now, assume PRF64 (this should be made configurable or detected)
			a_const = radio_constants->rssi_constants.prf64;

			rx_level = 10.0f * log10f(cir_pwr * BIT(17) /
				(rx_pacc * rx_pacc)) - a_const;

			bias_correction = conf->correct_timestamp_bias ? get_range_bias_by_rssi(rx_level) : 0;

			// read sys stat and save
			/* SAVE_UINT32_TO_LOG(dwt_reg_read_u32(dev, DWT_SYS_STATUS_ID, 0)); */

			// --- read incoming frame and check for validity
			uwb_ts_t current_ts = uwb_driver->system_timestamp(dev);
			uwb_ts_t time_until_reception = uwb_driver->timestamp_to_us(dev, correct_overflow(slot_start_ts, current_ts)) + 30;

			/* SAVE_UINT32_TO_LOG((uint32_t)time_until_reception); */
			/* otherwise the reception already begin everything good */
			if(time_until_reception < 1000) {
				k_busy_wait(time_until_reception);
			}
			uwb_driver->read_rx_frame(dev, rx_buf, pkt_len, 0);
			uwb_driver->switch_buffers(dev);

			// --- retrieve incoming frame ---
			if(pkt_len >= sizeof(struct deca_ranging_frame)+FRAME_LENGTH_ADDITIONAL) {
				LOG_ERR("unexpected frame size\n");
				ret = -EIO;
				goto cleanup;
			}

			memcpy(incoming_frame, rx_buf, pkt_len-FRAME_LENGTH_ADDITIONAL);
			/* SAVE_UINT32_TO_LOG(pkt_len); */

			if(incoming_frame->msg_id != DWT_MTM_RANGIN_FRAME_ID || pkt_len <= offsetof(struct deca_ranging_frame, payload)) {
				LOG_ERR("invalid ranging frame: msg_id=%02x, pkt_len=%u", incoming_frame->msg_id, pkt_len);
				incoming_frame_info->status = DECA_FRAME_REJECTED;
				incoming_frame_info->type = DECA_RECEIVED;
			} else {
				uwb_rx_diagnostics_t rx_diag;
				uwb_ts_t reception_ts = uwb_driver->read_rx_timestamp(dev, &rx_diag);
				// frame and container already allocated together above

				// store frame into frame_container
				// frame pointer already set during allocation

				incoming_frame_info->status = DECA_FRAME_OKAY;
				incoming_frame_info->timestamp = reception_ts - bias_correction;
				incoming_frame_info->type = DECA_RECEIVED;
				incoming_frame_info->slot = s - 1;
				incoming_frame_info->cir_pwr = cir_pwr;
				incoming_frame_info->rx_pacc = rx_pacc;
				incoming_frame_info->fp_index = fp_index;
				incoming_frame_info->fp_ampl1 = rx_diag.fp_ampl1;
				incoming_frame_info->fp_ampl2 = rx_diag.fp_ampl2;
				incoming_frame_info->fp_ampl3 = rx_diag.fp_ampl3;
				incoming_frame_info->std_noise = rx_diag.std_noise;
				incoming_frame_info->rx_level = rx_level;
				incoming_frame_info->fp_re = fp_re;
                                incoming_frame_info->fp_im = fp_im;
				incoming_frame_info->rx_phase = rx_diag.rx_phase;

				if(conf->cfo) {
					// warning this has to be adjusted for other data rates. -1e6 * (((((float) cfo)) / (((float) (2 * 1024) / 998.4e6) * 2^17)) / 6489.6e6)
					incoming_frame_info->cfo_ppm = (float)cfo * -0.000573121584378756f;
				}

				/* verify checksum */
				bool frame_valid = verify_checksum(incoming_frame, pkt_len-FRAME_LENGTH_ADDITIONAL);

				// --- store reception timestamp ---
				if(!frame_valid || (conf->reject_frames && ((int) fp_index >> 6) <= conf->fp_index_threshold)) {
					// this informs the upper layer that the frame was rejected and not included
					incoming_frame_info->status = DECA_FRAME_REJECTED;
				} else {
					struct deca_tagged_mm_timestamp *curr_rx_ts = &stored_timestamps[stored_timestamp_count];
					to_packed_uwb_ts(curr_rx_ts->ts, reception_ts - bias_correction);
					curr_rx_ts->addr = incoming_frame->addr;
					curr_rx_ts->slot = s - 1;

					// Apply RCPHASE correction before storing
					int16_t corrected_re = fp_re;
					int16_t corrected_im = fp_im;
					apply_rcphase_correction(&corrected_re, &corrected_im, rx_diag.rx_phase);

					curr_rx_ts->re = corrected_re;
                                        curr_rx_ts->im = corrected_im;

					stored_timestamp_count++;
				}
			}

			/* k_sem_take(&ctx->dev_lock, K_FOREVER); */
			/* uwb_driver->switch_buffers(dev); */
			/* k_sem_give(&ctx->dev_lock); */
			have_frame = 0;
			finished_frame = 1;
			/* SW_END(FRAME_HANDLING); */
		}

		// -- has to happen after frame handling, since the frame handler during double buffered operation may still add to the outgoing frame --
		if(type == DENSE_LOAD_TX_BUFFER) {
			/* SW_START(PREPARE_TX); */
			uwb_ts_t current_frame_transmission_ts = slot_start_ts;
			uint16_t future_tx_slot = UINT16_MAX;

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

			// --- in the following phase we will send data that we collected throughout the round
			if(future_tx_slot != UINT16_MAX) {
				/* --- store planned transmission timestamp in frame --- */
				current_frame_transmission_ts =
					((current_frame_transmission_ts + uwb_driver->us_to_timestamp(dev, ranging_conf->phy_activate_rx_delay_us)
						+ uwb_driver->us_to_timestamp(dev, conf->guard_period_us)/2
						+ uwb_driver->us_to_timestamp(dev, conf->micro_slot_offset_ns / 1000)
						) & ( (uint64_t) 0xFFFFFFFE00ULL )) + antenna_delay.tx_ant_dly;

				// -- already grab a frame for the next upcoming transmission --
				if (digest->length >= digest->capacity) {
					LOG_ERR("MM: Frame buffer overflow during TX prep, length=%d, max=%d",
						digest->length, digest->capacity);
					ret = -EOVERFLOW;
					goto cleanup;
				}
				outgoing_frame = digest->frames[digest->length].frame;
				struct deca_ranging_frame_container *outgoing_frame_info = &digest->frames[digest->length];
				digest->length++;

				outgoing_frame->msg_id  = DWT_MTM_RANGIN_FRAME_ID;
				outgoing_frame->addr = conf->addr;
				outgoing_frame->payload_size = 0;
				outgoing_frame->rx_ts_count = 0;
				to_packed_uwb_ts(outgoing_frame->tx_ts, current_frame_transmission_ts);

				// check if we have a payload which we should include in this frame
				if(current_slot->meta.payload != NULL && current_slot->meta.payload_size > 0) {
					// check how much space is available in frame for additional data
					if (current_slot->meta.payload_size > DWT_MTM_MAX_PAYLOAD) {
						LOG_ERR("payload size too high");
					}

					outgoing_frame->payload_size = current_slot->meta.payload_size;
					memcpy(outgoing_frame->payload, current_slot->meta.payload, current_slot->meta.payload_size);
				}

				// load timestamps
				if(current_slot->meta.load_stored_timestamps) {
					int remaining_fitting_timestamps = (sizeof(outgoing_frame->payload)-outgoing_frame->payload_size)/sizeof(struct deca_tagged_mm_timestamp);
					remaining_fitting_timestamps = MIN(remaining_fitting_timestamps, stored_timestamp_count);
					memcpy(outgoing_frame->payload + outgoing_frame->payload_size,
						&stored_timestamps[stored_timestamp_count - remaining_fitting_timestamps],
						remaining_fitting_timestamps * sizeof(struct deca_tagged_mm_timestamp));
					outgoing_frame->rx_ts_count = remaining_fitting_timestamps;
					stored_timestamp_count -= remaining_fitting_timestamps;
				}

				/* calculate checksum */
				size_t frame_size = offsetof(struct deca_ranging_frame, payload)
					+ outgoing_frame->payload_size + (outgoing_frame->rx_ts_count * sizeof(struct deca_tagged_mm_timestamp));
				uint16_t checksum = calculate_checksum(outgoing_frame, frame_size);
				outgoing_frame->checksum = checksum;

				// -- populate frame info struct (already allocated above) --
				outgoing_frame_info->status = DECA_FRAME_OKAY;
				outgoing_frame_info->timestamp = current_frame_transmission_ts;
				outgoing_frame_info->slot = future_tx_slot;
				outgoing_frame_info->type = DECA_TRANSMITTED;

				// --- Finally load frame into transmission buffer ---
					uwb_driver->setup_tx_frame(dev, (uint8_t*) outgoing_frame, frame_size);
				}

			/* SW_END(PREPARE_TX); */
		}

		// ---- II) Joining with PHY (only relevant if our current slot is a rx or tx operation) -----
		if ((type == DENSE_RX_SLOT || type == DENSE_TX_SLOT)) {
			/* SW_START(IRQ_WAIT_DELAY); */
			// Release device lock before waiting for IRQ to avoid deadlock
			uwb_driver->release_device(dev);
			irq_state = uwb_driver->wait_for_irq(dev);
			// Reacquire device lock after IRQ
			uwb_driver->acquire_device(dev);
			/* SW_END(IRQ_WAIT_DELAY); */

			if(irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT) {
				ret = -ETIMEDOUT;
				goto cleanup;
			} else if( irq_state == UWB_IRQ_PREAMBLE_DETECT_TIMEOUT) {
				/* uint32_t sys_status = uwb_driver->read_reg_u32(dev, DWT_SYS_STATUS_ID, 0); */
				/* SAVE_UINT32_TO_LOG(s | 0xA000); // | sys_state */
				/* SAVE_UINT32_TO_LOG(sys_status); // | sys_state */
			} else if(irq_state == UWB_IRQ_RX) {
                            have_frame = 1;

                            // ----- NON DOUBLE BUFFERED REGS ------
                            // Read registers not in double buffere swinging register set

                            if(conf->cfo) {
                                cfo = uwb_driver->read_carrier_integrator(dev);
                            }

                            if(current_slot->meta.with_cir_handler && conf->cir_handler != NULL) {
                                // first we do a bulk extract of the impulse memory
                                /* dwt_register_read(dev, DWT_ACC_MEM_ID, 0, sizeof(cir_acc_mem), cir_acc_mem); */

                                uint16_t fp_index, to_index, from_index;
                                if(current_slot->meta.only_first_path) {
                                    uwb_rx_diagnostics_t diag;
                                    uwb_driver->read_rx_timestamp(dev, &diag);
                                    fp_index = diag.fp_index >> 6;
                                    from_index = fp_index - current_slot->meta.from_index;
                                    to_index   = fp_index + current_slot->meta.to_index;
                                } else {
                                    from_index = current_slot->meta.from_index;
                                    to_index = current_slot->meta.to_index;
                                }

                                uint16_t offset = from_index * 4;
                                uint16_t length = (to_index - from_index + 1) * 4;

                                if(to_index - from_index > CONFIG_DWT_MTM_CIR_BUFFER_SIZE) {
                                    LOG_ERR("Too many samples requested");
                                    ret = -EIO;
                                    goto cleanup;
                                }

                                // Use UWB driver API for CIR data access
                                uwb_driver->read_cir_data(dev, cir_acc_mem + 1, offset, length);

                                /*
                                  We are not able to buffer the CIR for every reception in
                                  the round. Therefore, we directly call a upper-layer
                                  handler for the cir This might also be useful in the
                                  future since the upper layer might want to decide whether
                                  to throw away the transmission based on the cir.
                                */
                                // store re and im from fp_index as well for this
                                // transmission (+1 -> garbage byte)
                                uint8_t *valid_cir_mem = cir_acc_mem+1;
				fp_re = ((int16_t *)valid_cir_mem)[current_slot->meta.from_index*2];
                                fp_im = ((int16_t *)valid_cir_mem)[current_slot->meta.from_index*2+1];

                                conf->cir_handler(s, valid_cir_mem, (to_index - from_index + 1)*4);
                            }
			} else if(irq_state == UWB_IRQ_TX) {
			} else if(irq_state == UWB_IRQ_ERR) {
				if(s >= 2) {
				}

				/* SW_END(IRQ_WORK_HANDLER); */
				/* handling rx errors takes a longer time than the other
				   events, since a receiver reset has to be performed.  thus
				   if we don't return out of this round after a error event,
				   we should add about 90 microseconds to the slot duration,
				   in order to not miss subsequent frames. */
				/* SAVE_UINT32_TO_LOG(s | 0xE000); */
				/* SAVE_UINT32_TO_LOG(last_rx_err_stat); */
			} else if(irq_state == UWB_IRQ_HALF_DELAY_WARNING) {
				uwb_ts_t curr_ts __attribute__((unused)) = uwb_driver->system_timestamp(dev);
				ret = -EOVERFLOW;
				goto cleanup;
			}
			/* SW_END(IRQ_HANDLING); */
		}

		// -- Currently always iterate by slot_duration --
		if (current_slot) {
			slot_start_ts = (slot_start_ts + uwb_driver->us_to_timestamp(dev, current_slot->duration_us)) & UWB_TS_MASK;
		}
	}

	// digest->length is updated as frames are added

cleanup:
	uwb_driver->disable_cir_access(dev);
	// Release device access
	uwb_driver->setup_preamble_timeout(dev, 0);
	uwb_driver->release_device(dev);


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


/* MM Reference Protocol State Machine Implementation */

enum mm_protocol_state {
    MM_STATE_IDLE,
    MM_STATE_SETUP,
    MM_STATE_WAIT_POLL,      // Responder waits for poll
    MM_STATE_SEND_POLL,      // Initiator sends poll
    MM_STATE_WAIT_RESPONSE,  // Initiator waits for response
    MM_STATE_SEND_RESPONSE,  // Responder sends response
    MM_STATE_WAIT_FINAL,     // Responder waits for final
    MM_STATE_SEND_FINAL,     // Initiator sends final
    MM_STATE_SEND_POST_FINAL, // Initiator sends post-final
    MM_STATE_WAIT_POST_FINAL, // Responder waits for post-final
    MM_STATE_SEND_MEASUREMENT_EXCHANGE, // Responder sends measurement exchange
    MM_STATE_WAIT_MEASUREMENT_EXCHANGE, // Initiator waits for measurement exchange
    MM_STATE_SEND_MEASUREMENT_EXCHANGE_ACK, // Initiator sends measurement exchange ACK
    MM_STATE_WAIT_MEASUREMENT_EXCHANGE_ACK, // Responder waits for measurement exchange ACK
    MM_STATE_COMPLETE,
    MM_STATE_ERROR
};

static const char* mm_state_names[] = {
    "IDLE",
    "SETUP",
    "WAIT_POLL",
    "SEND_POLL",
    "WAIT_RESPONSE",
    "SEND_RESPONSE",
    "WAIT_FINAL",
    "SEND_FINAL",
    "SEND_POST_FINAL",
    "WAIT_POST_FINAL",
    "SEND_MEASUREMENT_EXCHANGE",
    "WAIT_MEASUREMENT_EXCHANGE",
    "SEND_MEASUREMENT_EXCHANGE_ACK",
    "WAIT_MEASUREMENT_EXCHANGE_ACK",
    "COMPLETE",
    "ERROR"
};

static const char* mm_state_to_string(enum mm_protocol_state state) {
    if (state >= 0 && state < ARRAY_SIZE(mm_state_names)) {
        return mm_state_names[state];
    }
    return "UNKNOWN";
}


struct mm_protocol_context;

struct mm_pending_rx {
    bool valid;
    uint16_t pkt_len;
    uint16_t header_len;
    struct deca_ranging_frame_container *container;
};

// IRQ state to string conversion moved to uwb_driver_api.h

struct mm_protocol_step {
    enum mm_protocol_state state;
    uint8_t msg_id;
    bool is_tx;               // true for TX, false for RX
    enum mm_protocol_state next_state_success;
    enum mm_protocol_state next_state_timeout;
    enum mm_protocol_state next_state_error;
};

struct mm_protocol_context {
    enum mm_protocol_state current_state;
    const struct device *dev;
    struct deca_mm_reference_config *config;
    struct deca_ranging_digest *digest;  // Added digest pointer

    // Runtime state
    int frame_counter;
    int frame_container_counter;
    uint16_t antenna_delay;

    // Timing data
    uwb_ts_t poll_tx_ts;
    uwb_ts_t poll_rx_ts;
    uwb_ts_t response_tx_ts;
    uwb_ts_t response_rx_ts;
    uwb_ts_t final_tx_ts;
    uwb_ts_t final_rx_ts;
    uwb_ts_t post_final_tx_ts;
    uwb_ts_t post_final_rx_ts;
    uwb_ts_t measurement_exchange_tx_ts;
    uwb_ts_t measurement_exchange_rx_ts;
    uwb_ts_t measurement_exchange_ack_tx_ts;
    uwb_ts_t measurement_exchange_ack_rx_ts;

    // CIR data
    int16_t poll_fp_re, poll_fp_im;
    int16_t response_fp_re, response_fp_im;
    int16_t final_fp_re, final_fp_im;
    int16_t post_final_fp_re, post_final_fp_im;
    int16_t measurement_exchange_fp_re, measurement_exchange_fp_im;
    int16_t measurement_exchange_ack_fp_re, measurement_exchange_ack_fp_im;

    // Frame addresses
    deca_short_addr_t poll_addr;
    deca_short_addr_t response_addr;
    deca_short_addr_t final_addr;

    struct mm_pending_rx pending_rx;
};

static void mm_flush_pending_rx(struct mm_protocol_context *ctx)
{
    struct mm_pending_rx *pending = &ctx->pending_rx;

    if (!pending->valid) {
        return;
    }

    const uwb_driver_t *uwb_driver = uwb_driver_get(ctx->dev);
    if (!uwb_driver) {
        pending->valid = false;
        return;
    }

    size_t frame_size = pending->pkt_len;
    size_t header_len = MIN(pending->header_len, frame_size);

#ifdef CONFIG_SEGGER_SYSTEMVIEW
    mm_sysview_ensure_named();
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_RX_PAYLOAD_COPY);
#endif
    if (pending->container && pending->container->frame) {
        struct deca_ranging_frame *frame = pending->container->frame;

        if (frame_size > header_len) {
            uwb_driver->read_rx_frame(ctx->dev,
                                      ((uint8_t *)frame) + header_len,
                                      frame_size - header_len,
                                      header_len);
        }
    }

    uwb_driver->switch_buffers(ctx->dev);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_PAYLOAD_COPY);
#endif

    pending->valid = false;
    pending->container = NULL;
    pending->pkt_len = 0U;
    pending->header_len = 0U;
}

// State transition tables for initiator and responder
static const struct mm_protocol_step initiator_steps[] = {
    {
        .state = MM_STATE_SEND_POLL,
        .msg_id = UWB_MTM_REF_POLL,
        .is_tx = true,
        .next_state_success = MM_STATE_WAIT_RESPONSE,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_WAIT_RESPONSE,
        .msg_id = UWB_MTM_REF_RESPONSE,
        .is_tx = false,
        .next_state_success = MM_STATE_SEND_FINAL,
        .next_state_timeout = MM_STATE_ERROR,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_SEND_FINAL,
        .msg_id = UWB_MTM_REF_FINAL,
        .is_tx = true,
        .next_state_success = MM_STATE_SEND_POST_FINAL,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_SEND_POST_FINAL,
        .msg_id = UWB_MTM_REF_POST_FINAL,
        .is_tx = true,
        .next_state_success = MM_STATE_WAIT_MEASUREMENT_EXCHANGE,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_WAIT_MEASUREMENT_EXCHANGE,
        .msg_id = UWB_MTM_REF_MEASUREMENT_EXCHANGE,
        .is_tx = false,
        .next_state_success = MM_STATE_SEND_MEASUREMENT_EXCHANGE_ACK,
        .next_state_timeout = MM_STATE_ERROR,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_SEND_MEASUREMENT_EXCHANGE_ACK,
        .msg_id = UWB_MTM_REF_MEASUREMENT_EXCHANGE_ACK,
        .is_tx = true,
        .next_state_success = MM_STATE_COMPLETE,
        .next_state_error = MM_STATE_ERROR
    }
};

static const struct mm_protocol_step responder_steps[] = {
    {
        .state = MM_STATE_WAIT_POLL,
        .msg_id = UWB_MTM_REF_POLL,
        .is_tx = false,
        .next_state_success = MM_STATE_SEND_RESPONSE,
        .next_state_timeout = MM_STATE_ERROR,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_SEND_RESPONSE,
        .msg_id = UWB_MTM_REF_RESPONSE,
        .is_tx = true,
        .next_state_success = MM_STATE_WAIT_FINAL,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_WAIT_FINAL,
        .msg_id = UWB_MTM_REF_FINAL,
        .is_tx = false,
        .next_state_success = MM_STATE_WAIT_POST_FINAL,
        .next_state_timeout = MM_STATE_ERROR,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_WAIT_POST_FINAL,
        .msg_id = UWB_MTM_REF_POST_FINAL,
        .is_tx = false,
        .next_state_success = MM_STATE_SEND_MEASUREMENT_EXCHANGE,
        .next_state_timeout = MM_STATE_ERROR,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_SEND_MEASUREMENT_EXCHANGE,
        .msg_id = UWB_MTM_REF_MEASUREMENT_EXCHANGE,
        .is_tx = true,
        .next_state_success = MM_STATE_WAIT_MEASUREMENT_EXCHANGE_ACK,
        .next_state_error = MM_STATE_ERROR
    },
    {
        .state = MM_STATE_WAIT_MEASUREMENT_EXCHANGE_ACK,
        .msg_id = UWB_MTM_REF_MEASUREMENT_EXCHANGE_ACK,
        .is_tx = false,
        .next_state_success = MM_STATE_COMPLETE,
        .next_state_timeout = MM_STATE_ERROR,
        .next_state_error = MM_STATE_ERROR
    }
};

static const struct mm_protocol_step* find_step_by_state(const struct mm_protocol_step *steps,
                                                         size_t step_count,
                                                         enum mm_protocol_state state) {
    for (size_t i = 0; i < step_count; i++) {
        if (steps[i].state == state) {
            return &steps[i];
        }
    }
    return NULL;
}

static void create_tagged_timestamp(struct deca_tagged_mm_timestamp *ts,
                                   uwb_ts_t timestamp, deca_short_addr_t addr,
                                   uint16_t slot, int16_t fp_re, int16_t fp_im) {
    to_packed_uwb_ts(ts->ts, timestamp);
    ts->addr = addr;
    ts->slot = slot;
    ts->re = fp_re;
    ts->im = fp_im;
}

static size_t get_frame_size(struct deca_ranging_frame *frame) {
    return offsetof(struct deca_ranging_frame, payload) +
           frame->payload_size +
           (frame->rx_ts_count * sizeof(struct deca_tagged_mm_timestamp));
}

static uwb_ts_t calculate_tx_timestamp(struct mm_protocol_context *ctx,
                                       const struct mm_protocol_step *step) {
    const uwb_driver_t *uwb_driver = uwb_driver_get(ctx->dev);
    uwb_ts_t tx_ts = 0;

    switch (step->state) {
        case MM_STATE_SEND_POLL:
            // Poll is sent at round start time with fixed PHY delay
            tx_ts = ctx->config->deca_round_start_ts +
                    uwb_driver->us_to_timestamp(ctx->dev, 164) + // 128 + 36 us PHY activate delay
                    uwb_driver->us_to_timestamp(ctx->dev, ctx->config->guard_period_us) / 2;
            break;

        case MM_STATE_SEND_RESPONSE:
            // Response is sent after respond_interval from poll reception
            tx_ts = ctx->poll_rx_ts +
                    uwb_driver->us_to_timestamp(ctx->dev, ctx->config->respond_interval_us);
            break;

        case MM_STATE_SEND_FINAL:
            // Final is sent after respond_interval from response reception
            tx_ts = ctx->response_rx_ts +
                    uwb_driver->us_to_timestamp(ctx->dev, ctx->config->respond_interval_us);
            break;

        case MM_STATE_SEND_POST_FINAL:
            // Post-final timing matches poll-to-response duration
            {
                uwb_ts_t poll_to_response_duration = ctx->response_rx_ts -
                    ((ctx->poll_tx_ts & ((uint64_t) 0xFFFFFFFE00ULL)) + ctx->antenna_delay);

                /* uwb_ts_t poll_to_response_duration = ctx->response_tx_ts - */
                /*     ((ctx->poll_rx_ts & ((uint64_t) 0xFFFFFFFE00ULL)) + ctx->antenna_delay); */

                tx_ts = ctx->final_tx_ts + poll_to_response_duration;
            }
            break;

        case MM_STATE_SEND_MEASUREMENT_EXCHANGE:
            // Measurement exchange sent after post-final reception with respond_interval delay
            tx_ts = ctx->post_final_rx_ts +
                    uwb_driver->us_to_timestamp(ctx->dev, ctx->config->respond_interval_us);
            break;

        case MM_STATE_SEND_MEASUREMENT_EXCHANGE_ACK:
            // Measurement exchange ACK sent after measurement exchange reception with respond_interval delay
            tx_ts = ctx->measurement_exchange_rx_ts +
                    uwb_driver->us_to_timestamp(ctx->dev, ctx->config->respond_interval_us);
            break;

        default:
            LOG_ERR("Invalid TX state: %d", step->state);
            break;
    }

    return tx_ts & UWB_TS_MASK;
}

static struct deca_ranging_frame_container* build_frame_for_step(struct mm_protocol_context *ctx,
                                                                    const struct mm_protocol_step *step) {
    if (ctx->digest->length >= ctx->digest->capacity) {
        LOG_ERR("MM State Machine: Frame buffer overflow, length=%d, max=%d",
                ctx->digest->length, ctx->digest->capacity);
        return NULL;
    }
    struct deca_ranging_frame_container *container = &ctx->digest->frames[ctx->digest->length];
    struct deca_ranging_frame *frame = container->frame;
    ctx->digest->length++;

    // Common frame setup
    frame->msg_id = step->msg_id;
    frame->addr = ctx->config->addr;
    frame->payload_size = 0;
    frame->rx_ts_count = 0;

    switch (step->state) {
        case MM_STATE_SEND_POLL:
            // Poll frame has no special payload
            break;

        case MM_STATE_SEND_RESPONSE:
            // Response includes poll reception timestamp
            frame->rx_ts_count = 1;
            create_tagged_timestamp(
                (struct deca_tagged_mm_timestamp *)frame->payload,
                ctx->poll_rx_ts, ctx->poll_addr, 1,
                ctx->poll_fp_re, ctx->poll_fp_im);
            break;

        case MM_STATE_SEND_FINAL:
            // Final includes response reception timestamp
            frame->rx_ts_count = 1;
            create_tagged_timestamp(
                (struct deca_tagged_mm_timestamp *)frame->payload,
                ctx->response_rx_ts, ctx->response_addr, 2,
                ctx->response_fp_re, ctx->response_fp_im);
            break;

        case MM_STATE_SEND_POST_FINAL:
            // no special content in post final
            break;

        case MM_STATE_SEND_MEASUREMENT_EXCHANGE:
            // Measurement exchange includes both final and post-final reception timestamps
            frame->rx_ts_count = 2;

            // First timestamp: final reception (slot 4)
            create_tagged_timestamp(
                (struct deca_tagged_mm_timestamp *)frame->payload,
                ctx->final_rx_ts, ctx->final_addr, 4,
                ctx->final_fp_re, ctx->final_fp_im);

            // Second timestamp: post-final reception (slot 5)
            create_tagged_timestamp(
                ((struct deca_tagged_mm_timestamp *)frame->payload) + 1,
                ctx->post_final_rx_ts, ctx->final_addr, 5,
                ctx->post_final_fp_re, ctx->post_final_fp_im);
            break;

        case MM_STATE_SEND_MEASUREMENT_EXCHANGE_ACK:
            // Measurement exchange ACK includes measurement exchange reception timestamp
            frame->rx_ts_count = 1;
            create_tagged_timestamp(
                (struct deca_tagged_mm_timestamp *)frame->payload,
                ctx->measurement_exchange_rx_ts, ctx->response_addr, 7,
                ctx->measurement_exchange_fp_re, ctx->measurement_exchange_fp_im);
            break;

        default:
            return NULL;
    }

    return container;
}

static int store_tx_frame_info(struct mm_protocol_context *ctx,
                              struct deca_ranging_frame_container *container,
                              uwb_ts_t tx_ts,
                              const struct mm_protocol_step *step) {

    // frame pointer already set during allocation
    container->status = DECA_FRAME_OKAY;
    container->timestamp = (tx_ts & ((uint64_t) 0xFFFFFFFE00ULL)) + ctx->antenna_delay;
    container->type = DECA_TRANSMITTED;

    // Determine slot based on state
    switch (step->state) {
        case MM_STATE_SEND_POLL:
            container->slot = 1;
            ctx->poll_tx_ts = tx_ts;
            break;
        case MM_STATE_SEND_RESPONSE:
            container->slot = 2;
            ctx->response_tx_ts = tx_ts;
            break;
        case MM_STATE_SEND_FINAL:
            container->slot = 4;
            ctx->final_tx_ts = tx_ts;
            break;
        case MM_STATE_SEND_POST_FINAL:
            container->slot = 5;
            ctx->post_final_tx_ts = tx_ts;
            break;
        case MM_STATE_SEND_MEASUREMENT_EXCHANGE:
            container->slot = 7;
            ctx->measurement_exchange_tx_ts = tx_ts;
            break;
        case MM_STATE_SEND_MEASUREMENT_EXCHANGE_ACK:
            container->slot = 8;
            ctx->measurement_exchange_ack_tx_ts = tx_ts;
            break;
        default:
            return -EINVAL;
    }

    return 0;
}

static int process_rx_frame(struct mm_protocol_context *ctx,
                           const struct mm_protocol_step *step,
                           uwb_ts_t rx_ts,
                           int16_t fp_re, int16_t fp_im,
                           uwb_rx_diagnostics_t *rx_diag,
                           struct deca_ranging_frame_container *container) {

    // frame pointer already set during allocation
    container->status = DECA_FRAME_OKAY;
    container->timestamp = rx_ts;
    container->type = DECA_RECEIVED;
    container->fp_re = fp_re;
    container->fp_im = fp_im;
    container->rx_phase = rx_diag->rx_phase;
    container->rx_pacc = rx_diag->rx_pacc;

    // Store reception data based on state
    switch (step->state) {
        case MM_STATE_WAIT_POLL:
            container->slot = 1;
            ctx->poll_rx_ts = rx_ts;
            ctx->poll_fp_re = fp_re;
            ctx->poll_fp_im = fp_im;
            ctx->poll_addr = container->frame->addr;
            break;
        case MM_STATE_WAIT_RESPONSE:
            container->slot = 2;
            ctx->response_rx_ts = rx_ts;
            ctx->response_fp_re = fp_re;
            ctx->response_fp_im = fp_im;
            ctx->response_addr = container->frame->addr;
            break;
        case MM_STATE_WAIT_FINAL:
            container->slot = 4;
            ctx->final_rx_ts = rx_ts;
            ctx->final_fp_re = fp_re;
            ctx->final_fp_im = fp_im;
            ctx->final_addr = container->frame->addr;
            break;
        case MM_STATE_WAIT_POST_FINAL:
            container->slot = 5;
            ctx->post_final_rx_ts = rx_ts;
            ctx->post_final_fp_re = fp_re;
            ctx->post_final_fp_im = fp_im;
            break;
        case MM_STATE_WAIT_MEASUREMENT_EXCHANGE:
            container->slot = 7;
            ctx->measurement_exchange_rx_ts = rx_ts;
            ctx->measurement_exchange_fp_re = fp_re;
            ctx->measurement_exchange_fp_im = fp_im;
            break;
        case MM_STATE_WAIT_MEASUREMENT_EXCHANGE_ACK:
            container->slot = 8;
            ctx->measurement_exchange_ack_rx_ts = rx_ts;
            ctx->measurement_exchange_ack_fp_re = fp_re;
            ctx->measurement_exchange_ack_fp_im = fp_im;
            break;
        default:
            return -EINVAL;
    }

    return 0;
}

static int execute_tx_step(struct mm_protocol_context *ctx,
                          const struct mm_protocol_step *step) {
    int ret = 0;
    const uwb_driver_t *uwb_driver = uwb_driver_get(ctx->dev);

#ifdef CONFIG_SEGGER_SYSTEMVIEW
    bool build_active = false;
    bool timestamp_active = false;
    bool load_active = false;
    bool record_active = false;

    mm_sysview_ensure_named();
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_TX_STEP);
#endif

    // Build frame based on step type
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    build_active = true;
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_TX_BUILD_FRAME);
#endif
    struct deca_ranging_frame_container *container = build_frame_for_step(ctx, step);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_TX_BUILD_FRAME);
    build_active = false;
#endif
    if (!container) {
        ret = -ENOMEM;
        goto exit;
    }
    struct deca_ranging_frame *frame = container->frame;

    // Calculate transmission timestamp
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    timestamp_active = true;
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_TX_TIMESTAMP);
#endif
    uwb_ts_t tx_ts = calculate_tx_timestamp(ctx, step);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_TX_TIMESTAMP);
    timestamp_active = false;
#endif

    // Store transmission timestamp in frame
    to_packed_uwb_ts(frame->tx_ts, (tx_ts & ((uint64_t) 0xFFFFFFFE00ULL)) + ctx->antenna_delay);

    // Setup and transmit
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    load_active = true;
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_TX_LOAD_FRAME);
#endif
    size_t frame_size = get_frame_size(frame);
    uwb_driver->setup_tx_frame(ctx->dev, (uint8_t*)frame, frame_size);
    uwb_driver->start_tx(ctx->dev, tx_ts & UWB_TS_MASK);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_TX_LOAD_FRAME);
    load_active = false;
#endif

    mm_flush_pending_rx(ctx);

    // Wait for transmission completion
    uwb_driver->release_device(ctx->dev);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_WAIT_IRQ);
#endif
    uwb_irq_state_e irq_state = uwb_driver->wait_for_irq(ctx->dev);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_WAIT_IRQ);
#endif
    uwb_driver->acquire_device(ctx->dev);
    if (irq_state != UWB_IRQ_TX) {
        printk("MM_REF: ERROR - TX frame failed (state=%s, irq_state=%s)\n",
               mm_state_to_string(step->state), irq_state_to_string(irq_state));
        ret = -EIO;
        goto exit;
    }

    // Store transmission info
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    record_active = true;
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_TX_RECORD_INFO);
#endif
    ret = store_tx_frame_info(ctx, container, tx_ts, step);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_TX_RECORD_INFO);
    record_active = false;
#endif

exit:
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    if (record_active) {
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_TX_RECORD_INFO);
    }
    if (load_active) {
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_TX_LOAD_FRAME);
    }
    if (timestamp_active) {
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_TX_TIMESTAMP);
    }
    if (build_active) {
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_TX_BUILD_FRAME);
    }
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_TX_STEP);
#endif
    return ret;
}

static int execute_rx_step(struct mm_protocol_context *ctx,
                          const struct mm_protocol_step *step) {
    int ret = 0;
    const uwb_driver_t *uwb_driver = uwb_driver_get(ctx->dev);

#ifdef CONFIG_SEGGER_SYSTEMVIEW
    bool read_info_active = false;
    bool frame_len_active = false;
    bool header_copy_active = false;
    bool timestamp_active = false;
    bool dispatch_active = false;

    mm_sysview_ensure_named();
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_RX_STEP);
#endif

    mm_flush_pending_rx(ctx);

    // Enable RX at appropriate time
    uwb_ts_t rx_enable_ts = 0;
    if (step->state == MM_STATE_WAIT_POLL) {
        // For poll, enable RX at round start time
        rx_enable_ts = ctx->config->deca_round_start_ts & UWB_TS_MASK;
    }
    uwb_driver->enable_rx(ctx->dev, 0, rx_enable_ts);

    // Wait for reception
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_WAIT_IRQ);
#endif
    uwb_driver->release_device(ctx->dev);
    uwb_irq_state_e irq_state = uwb_driver->wait_for_irq(ctx->dev);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_WAIT_IRQ);
#endif
    uwb_driver->acquire_device(ctx->dev);
    if (irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT ||
        irq_state == UWB_IRQ_PREAMBLE_DETECT_TIMEOUT) {
        printk("MM_REF: TIMEOUT - RX frame timed out (state=%s)\n",
               mm_state_to_string(step->state));
        ret = -ETIMEDOUT;
        goto exit_step;
    } else if (irq_state != UWB_IRQ_RX) {
        printk("MM_REF: ERROR - RX frame failed (state=%s, irq_state=%s)\n",
               mm_state_to_string(step->state), irq_state_to_string(irq_state));
        ret = -EIO;
        goto exit_step;
    }

    // Stage 1: read frame metadata/header
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    read_info_active = true;
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_RX_READ_INFO);
#endif
    if (ctx->digest->length >= ctx->digest->capacity) {
        LOG_ERR("MM State Machine RX: Frame buffer overflow, length=%d, max=%d",
                ctx->digest->length, ctx->digest->capacity);
        ret = -EOVERFLOW;
        goto exit_step;
    }
    struct deca_ranging_frame_container *container = &ctx->digest->frames[ctx->digest->length];
    struct deca_ranging_frame *frame = container->frame;
    ctx->digest->length++;

#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_READ_INFO);
    read_info_active = false;
    frame_len_active = true;
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_RX_FRAME_LENGTH);
#endif
    uint16_t pkt_len = uwb_driver->get_rx_frame_length(ctx->dev);
    if (pkt_len >= sizeof(struct deca_ranging_frame) + FRAME_LENGTH_ADDITIONAL) {
        printk("MM_REF: ERROR - Frame too large: %u bytes\n", pkt_len);
        ret = -EIO;
        goto exit_step;
    }

    size_t frame_size = (pkt_len > FRAME_LENGTH_ADDITIONAL) ?
        (pkt_len - FRAME_LENGTH_ADDITIONAL) : 0U;
    frame_size = MIN(frame_size, sizeof(struct deca_ranging_frame));
    size_t header_len = MIN(frame_size, offsetof(struct deca_ranging_frame, payload));

#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_FRAME_LENGTH);
    frame_len_active = false;
#endif
    if (header_len > 0U) {
#ifdef CONFIG_SEGGER_SYSTEMVIEW
        header_copy_active = true;
        SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_RX_HEADER_COPY);
#endif
        uwb_driver->read_rx_frame(ctx->dev, (uint8_t *)frame, header_len, 0);
#ifdef CONFIG_SEGGER_SYSTEMVIEW
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_HEADER_COPY);
        header_copy_active = false;
#endif
    }

#ifdef CONFIG_SEGGER_SYSTEMVIEW
    timestamp_active = true;
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_RX_TIMESTAMP);
#endif

    // Stage 2: capture timestamp/diagnostics
    uwb_rx_diagnostics_t rx_diag;
    uwb_ts_t rx_ts = uwb_driver->read_rx_timestamp(ctx->dev, &rx_diag);
    int16_t fp_re = 0, fp_im = 0;
    read_cir_first_path(ctx->dev, &rx_diag, &fp_re, &fp_im);

    if (pkt_len <= offsetof(struct deca_ranging_frame, payload)) {
        printk("MM_REF: ERROR - Frame too small: %u bytes\n", pkt_len);
        uwb_driver->switch_buffers(ctx->dev);
        ret = -EIO;
        goto exit_step;
    }

    if (frame->msg_id != step->msg_id) {
        printk("MM_REF: ERROR - Wrong message ID: expected %02x, got %02x\n",
               step->msg_id, frame->msg_id);
        uwb_driver->switch_buffers(ctx->dev);
        ret = -EIO;
        goto exit_step;
    }

#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_TIMESTAMP);
    timestamp_active = false;
    dispatch_active = true;
    SEGGER_SYSVIEW_OnUserStart(MM_SYSVIEW_MARKER_RX_DISPATCH);
#endif

    struct mm_pending_rx *pending = &ctx->pending_rx;
    pending->valid = false;
    pending->pkt_len = frame_size;
    pending->header_len = header_len;
    pending->container = container;

    // Stage 3: dispatch into digest / higher layers
    ret = process_rx_frame(ctx, step, rx_ts, fp_re, fp_im, &rx_diag, container);
    if (ret < 0) {
        uwb_driver->switch_buffers(ctx->dev);
        pending->valid = false;
        pending->container = NULL;
        pending->pkt_len = 0U;
        pending->header_len = 0U;
        goto exit_step;
    }

    pending->valid = true;

#ifdef CONFIG_SEGGER_SYSTEMVIEW
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_DISPATCH);
    dispatch_active = false;
#endif
exit_step:
#ifdef CONFIG_SEGGER_SYSTEMVIEW
    if (dispatch_active) {
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_DISPATCH);
    }
    if (timestamp_active) {
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_TIMESTAMP);
    }
    if (header_copy_active) {
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_HEADER_COPY);
    }
    if (frame_len_active) {
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_FRAME_LENGTH);
    }
    if (read_info_active) {
        SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_READ_INFO);
    }
    SEGGER_SYSVIEW_OnUserStop(MM_SYSVIEW_MARKER_RX_STEP);
#endif
    return ret;
}

static int execute_protocol_step(struct mm_protocol_context *ctx,
                               const struct mm_protocol_step *step) {
    int ret = 0;

    if (step->is_tx) {
        ret = execute_tx_step(ctx, step);
    } else {
        ret = execute_rx_step(ctx, step);
    }

    // Handle state transitions based on result
    if (ret == 0) {
        ctx->current_state = step->next_state_success;
    } else if (ret == -ETIMEDOUT) {
        ctx->current_state = step->next_state_timeout;
    } else {
        ctx->current_state = step->next_state_error;
    }

    return ret;
}

static int deca_mm_reference_state_machine(const struct device *dev,
                                          struct deca_mm_reference_config *conf,
                                          struct deca_ranging_digest *digest) {
    struct mm_protocol_context ctx = {0};
    // Get antenna delay from UWB driver
    uwb_antenna_delay_t ant_delay;
    const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
    uwb_driver->get_antenna_delay(dev, &ant_delay);

    // Initialize context
    ctx.dev = dev;
    ctx.config = conf;
    ctx.digest = digest;  // Store digest pointer
    ctx.antenna_delay = ant_delay.tx_ant_dly;
    ctx.frame_counter = 0;
    ctx.frame_container_counter = 0;

    // Select step table based on role
    const struct mm_protocol_step *steps;
    size_t step_count;

    if (conf->isInitiator) {
        steps = initiator_steps;
        step_count = ARRAY_SIZE(initiator_steps);
        ctx.current_state = MM_STATE_SEND_POLL;
    } else {
        steps = responder_steps;
        step_count = ARRAY_SIZE(responder_steps);
        ctx.current_state = MM_STATE_WAIT_POLL;
    }

    int ret = 0;

    // Execute protocol steps
    while (ctx.current_state != MM_STATE_COMPLETE && ctx.current_state != MM_STATE_ERROR) {
        const struct mm_protocol_step *current_step = find_step_by_state(steps, step_count, ctx.current_state);
        if (!current_step) {
            printk("MM_REF: ERROR - No step found for state %s\n", mm_state_to_string(ctx.current_state));
            ret = -EINVAL;
            ctx.current_state = MM_STATE_ERROR;
            goto out;
        }

        ret = execute_protocol_step(&ctx, current_step);
        if (ret < 0 && ctx.current_state == MM_STATE_ERROR) {
            goto out;
        }
    }

    if (ctx.current_state == MM_STATE_COMPLETE) {
        // frames are already in digest->frames, digest->length is correctly maintained
        ret = digest->length;
        goto out;
    }

    ret = -EIO;

out:
    mm_flush_pending_rx(&ctx);
    return ret;
}

int deca_mm_reference(const struct device *dev,
	struct deca_mm_reference_config *conf, struct deca_ranging_digest *digest) {
	int ret = 0;

	const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
	if (!uwb_driver) {
		LOG_ERR("No UWB driver found for device");
		return -ENODEV;
	}

	// Acquire device access for ranging operation
	ret = uwb_driver->acquire_device(dev);
	if (ret != 0) {
		LOG_ERR("Failed to acquire device for ranging");
		return ret;
	}

	uwb_driver->disable_txrx(dev);
	uwb_driver->set_frame_filter(dev, 0, 0);
	uwb_driver->align_double_buffering(dev); // for the following execution we require that host and receiver side are aligned
	uwb_driver->enable_cir_access(dev); // keep CIR memory mapped for the full reference exchange

	// Setup generous frame timeout for MM reference protocol (about double the response interval)
	uint32_t mm_frame_timeout_us = conf->respond_interval_us * 2;
	uwb_driver->setup_frame_timeout(dev, mm_frame_timeout_us);

	uwb_driver->setup_preamble_timeout(dev, mm_frame_timeout_us); // Disable preamble timeout for MM reference

	// Use the new state machine implementation
	ret = deca_mm_reference_state_machine(dev, conf, digest);
	if(ret < 0) {
		goto cleanup;
	}

cleanup:
	// Reset timeouts
	uwb_driver->setup_preamble_timeout(dev, 0);
	uwb_driver->setup_frame_timeout(dev, 0); // Reset timeout
	uwb_driver->disable_cir_access(dev);

	// Release device access
	uwb_driver->release_device(dev);

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
