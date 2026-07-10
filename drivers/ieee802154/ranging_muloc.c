/*
 * MULoc Anchor Overhearing Ranging Driver
 *
 * Implements the MULoc protocol: N anchors broadcast sequentially, each
 * embedding CIR/phase/timestamp data (AO records) from its own receptions
 * of prior anchors.  Tags (anchor_id == 0xFF) passively overhear all
 * transmissions and collect per-anchor CIR, phase, timestamp, and carrier
 * integrator for offline multilateration.
 *
 * Wire format per TX frame (raw, NOT deca_ranging_frame):
 *   [MAC 10B] [anchor_id 1B] [AO payload (N-1)*13B] [frame_seq 1B]
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/ieee802154/uwb_frame_utils.h>
#include <app/lib/system/hw.h>

LOG_MODULE_REGISTER(muloc_ranging, LOG_LEVEL_INF);

/* --------------------------------------------------------------------
 * Protocol constants
 * -------------------------------------------------------------------- */

#define MULOC_MAX_ANCHORS  8
#define MULOC_MAX_ROUNDS   8

#define MULOC_MAC_HEADER_LEN   10
#define MULOC_AO_RECORD_SIZE   13  /* bytes per AO entry in the wire format */

/* Maximum raw frame size: header + anchor_id + (N-1)*AO + frame_seq */
#define MULOC_MAX_FRAME_SIZE  (MULOC_MAC_HEADER_LEN + 1 + \
                               (MULOC_MAX_ANCHORS - 1) * MULOC_AO_RECORD_SIZE + 1)

#define MULOC_PAN_ID  0xDECA

/* --------------------------------------------------------------------
 * Data structures
 * -------------------------------------------------------------------- */

struct muloc_ao_record {
	uint16_t cir_re;          /* CIR first-path real (little-endian on wire) */
	uint16_t cir_im;          /* CIR first-path imaginary (little-endian on wire) */
	uint8_t  rcphase;         /* RCPHASE correction value */
	uint8_t  rx_pacc;         /* Preamble accumulation count (truncated to 8 bit) */
	uint16_t max_growth_cir;  /* Max CIR growth for RSSI (big-endian on wire) */
	uint8_t  rx_ts[5];        /* 40-bit RX timestamp (little-endian on wire) */
};

struct muloc_to_record {
	struct muloc_ao_record ao;
	int32_t carrier_integrator; /* CFO for clock-drift estimation */
};

struct muloc_ranging_config {
	uint8_t  anchor_count;
	uint8_t  anchor_id;         /* 0..N-1 for anchors, 0xFF for tag/listener */
	uint8_t  num_rounds;
	uint16_t delay_time_us;     /* Inter-anchor TX delay (default 600 us) */
	uint16_t turn_delay_us;     /* Delay between rounds (default 600 us) */
	uint16_t rx_timeout_us;     /* RX timeout per slot (default 2500 us) */
	uint64_t round_start_ts;    /* DWT timestamp for first round start */
	uint8_t  start_channel;
	uint8_t  hop_channel;
};

struct muloc_round_result {
	uint8_t  round_index;
	uint8_t  channel;
	uint8_t  anchor_count;
	struct muloc_ao_record ao_records[MULOC_MAX_ANCHORS];
	uint8_t  ao_valid_mask;
	struct muloc_to_record to_records[MULOC_MAX_ANCHORS];
	uint8_t  to_valid_mask;
	uint8_t  frame_seq;
};

/* --------------------------------------------------------------------
 * CIR first-path extraction (mirrors ranging_mm.c read_cir_first_path)
 * -------------------------------------------------------------------- */

static int muloc_read_cir_first_path(const struct device *dev,
				     const uwb_rx_diagnostics_t *diag,
				     int16_t *fp_re, int16_t *fp_im)
{
	const uwb_driver_t *uwb_driver = uwb_driver_get(dev);

	if (!uwb_driver || !diag) {
		return -EINVAL;
	}

	uint16_t fp_index = diag->fp_index >> 6;
	uint16_t offset = fp_index * 4;

	/* 1 garbage byte + 4 data bytes (2B real + 2B imaginary) */
	uint8_t cir_buf[5];

	uwb_driver->read_cir_data(dev, cir_buf, offset, 5);

	uint8_t *data = cir_buf + 1; /* skip garbage byte */
	*fp_re = ((int16_t *)data)[0];
	*fp_im = ((int16_t *)data)[1];

	return 0;
}

/* --------------------------------------------------------------------
 * Wire-format helpers
 * -------------------------------------------------------------------- */

/**
 * Build the MAC header for a MULoc broadcast frame.
 *
 * Layout (10 bytes):
 *   [0] 0x41   frame control lo
 *   [1] 0x88   frame control hi
 *   [2] seq
 *   [3] PAN lo
 *   [4] PAN hi
 *   [5] dst lo (0xFF broadcast)
 *   [6] dst hi (0xFF broadcast)
 *   [7] src lo
 *   [8] src hi
 *   [9] msg_id = UWB_MULOC_BROADCAST
 */
static void build_mac_header(uint8_t *buf, uint8_t seq, uint16_t src_addr)
{
	buf[0] = 0x41;
	buf[1] = 0x88;
	buf[2] = seq;
	buf[3] = (uint8_t)(MULOC_PAN_ID & 0xFF);
	buf[4] = (uint8_t)(MULOC_PAN_ID >> 8);
	buf[5] = 0xFF; /* dst broadcast */
	buf[6] = 0xFF;
	buf[7] = (uint8_t)(src_addr & 0xFF);
	buf[8] = (uint8_t)(src_addr >> 8);
	buf[9] = UWB_MULOC_BROADCAST;
}

/**
 * Serialize one AO record into the wire buffer (13 bytes).
 *
 * Wire layout:
 *   cir_re   : 2B LE
 *   cir_im   : 2B LE
 *   rcphase  : 1B
 *   rx_pacc  : 1B
 *   max_growth_cir : 2B BE  (matches original MULoc format)
 *   rx_ts    : 5B LE
 */
static void serialize_ao_record(uint8_t *buf, const struct muloc_ao_record *rec)
{
	/* cir_re little-endian */
	buf[0] = (uint8_t)(rec->cir_re & 0xFF);
	buf[1] = (uint8_t)(rec->cir_re >> 8);
	/* cir_im little-endian */
	buf[2] = (uint8_t)(rec->cir_im & 0xFF);
	buf[3] = (uint8_t)(rec->cir_im >> 8);
	/* rcphase */
	buf[4] = rec->rcphase;
	/* rx_pacc */
	buf[5] = rec->rx_pacc;
	/* max_growth_cir big-endian */
	buf[6] = (uint8_t)(rec->max_growth_cir >> 8);
	buf[7] = (uint8_t)(rec->max_growth_cir & 0xFF);
	/* rx_ts 5 bytes little-endian */
	memcpy(&buf[8], rec->rx_ts, 5);
}

/**
 * Deserialize one AO record from the wire buffer (13 bytes).
 */
static void deserialize_ao_record(struct muloc_ao_record *rec, const uint8_t *buf)
{
	rec->cir_re = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
	rec->cir_im = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
	rec->rcphase = buf[4];
	rec->rx_pacc = buf[5];
	rec->max_growth_cir = ((uint16_t)buf[6] << 8) | (uint16_t)buf[7];
	memcpy(rec->rx_ts, &buf[8], 5);
}

/**
 * Build the complete TX frame for this anchor's broadcast slot.
 *
 * @param buf         Output buffer (must be >= MULOC_MAX_FRAME_SIZE)
 * @param anchor_id   This anchor's ID
 * @param frame_seq   Current sequence number
 * @param src_addr    Node short address (for MAC header)
 * @param ao_records  AO records collected during this round's earlier RX slots
 * @param ao_valid    Bitmask of valid AO records
 * @param anchor_count Total anchors in the round
 * @return Total frame length in bytes
 */
static uint16_t build_tx_frame(uint8_t *buf,
			       uint8_t anchor_id,
			       uint8_t frame_seq,
			       uint16_t src_addr,
			       const struct muloc_ao_record *ao_records,
			       uint8_t ao_valid,
			       uint8_t anchor_count)
{
	uint16_t pos = 0;

	build_mac_header(buf, frame_seq, src_addr);
	pos = MULOC_MAC_HEADER_LEN;

	/* anchor_id byte */
	buf[pos++] = anchor_id;

	/*
	 * AO payload: one 13-byte record per anchor, ordered by anchor_id,
	 * skipping self.  Invalid slots are zero-filled.
	 */
	for (uint8_t i = 0; i < anchor_count; i++) {
		if (i == anchor_id) {
			continue; /* skip self */
		}
		if (ao_valid & BIT(i)) {
			serialize_ao_record(&buf[pos], &ao_records[i]);
		} else {
			memset(&buf[pos], 0, MULOC_AO_RECORD_SIZE);
		}
		pos += MULOC_AO_RECORD_SIZE;
	}

	/* frame_seq trailer */
	buf[pos++] = frame_seq;

	return pos;
}

/* --------------------------------------------------------------------
 * Main ranging function
 * -------------------------------------------------------------------- */

int deca_muloc_ranging(const struct device *dev,
		       struct muloc_ranging_config *conf,
		       struct muloc_round_result *results,
		       size_t max_results)
{
	const uwb_driver_t *uwb_driver = uwb_driver_get(dev);

	if (!uwb_driver) {
		LOG_ERR("No UWB driver for device");
		return -ENODEV;
	}

	if (conf->anchor_count < 2 || conf->anchor_count > MULOC_MAX_ANCHORS) {
		LOG_ERR("anchor_count %u out of range [2,%d]",
			conf->anchor_count, MULOC_MAX_ANCHORS);
		return -EINVAL;
	}

	if (conf->num_rounds == 0 || conf->num_rounds > MULOC_MAX_ROUNDS) {
		LOG_ERR("num_rounds %u out of range [1,%d]",
			conf->num_rounds, MULOC_MAX_ROUNDS);
		return -EINVAL;
	}

	if (conf->anchor_id != 0xFF && conf->anchor_id >= conf->anchor_count) {
		LOG_ERR("anchor_id %u >= anchor_count %u",
			conf->anchor_id, conf->anchor_count);
		return -EINVAL;
	}

	/* ---- Acquire device and configure PHY ---- */
	int ret = uwb_driver->acquire_device(dev);
	if (ret != 0) {
		LOG_ERR("Failed to acquire device: %d", ret);
		return ret;
	}

	uwb_driver->disable_txrx(dev);
	uwb_driver->set_frame_filter(dev, 0, 0);
	uwb_driver->align_double_buffering(dev);
	uwb_driver->enable_cir_access(dev);
	uwb_driver->setup_frame_timeout(dev, conf->rx_timeout_us);
	uwb_driver->setup_preamble_timeout(dev, conf->rx_timeout_us);

	/* Retrieve node source address for MAC header */
	uwb_antenna_delay_t antenna_delay;
	uwb_driver->get_antenna_delay(dev, &antenna_delay);

	/* We use the lower 16 bits of the source address field.
	 * Anchors encode their anchor_id; tags encode 0xFF. */
	uint16_t src_addr = conf->anchor_id;

	uint8_t current_channel = conf->start_channel;
	uint8_t frame_seq = 0;
	uint8_t tx_buf[MULOC_MAX_FRAME_SIZE];
	uint8_t rx_buf[MULOC_MAX_FRAME_SIZE];

	const uint8_t anchor_count = conf->anchor_count;
	const uint8_t anchor_id = conf->anchor_id;
	const bool is_tag = (anchor_id == 0xFF);

	uwb_ts_t last_rx_ts = 0;
	int completed_rounds = 0;

	/* ---- Round loop ---- */
	for (uint8_t round = 0;
	     round < conf->num_rounds && round < (uint8_t)max_results;
	     round++) {

		struct muloc_round_result *res = &results[round];

		res->round_index = round;
		res->channel = current_channel;
		res->anchor_count = anchor_count;
		res->ao_valid_mask = 0;
		res->to_valid_mask = 0;

		/* ---- Slot loop (one slot per anchor) ---- */
		for (uint8_t slot = 0; slot < anchor_count; slot++) {

			if (!is_tag && slot == anchor_id) {
				/* ========== TX SLOT ========== */

				/* Calculate TX timestamp */
				uwb_ts_t tx_ts;

				if (anchor_id == 0 && round == 0) {
					/* First anchor, first round: use configured start */
					tx_ts = conf->round_start_ts;
				} else if (anchor_id == 0) {
					/* First anchor, subsequent rounds */
					tx_ts = last_rx_ts +
						uwb_driver->us_to_timestamp(dev,
							conf->turn_delay_us);
				} else {
					/* Other anchors: delay after last reception */
					tx_ts = last_rx_ts +
						uwb_driver->us_to_timestamp(dev,
							conf->delay_time_us);
				}
				tx_ts &= UWB_TS_MASK;

				/* Build frame with AO records collected so far */
				uint16_t frame_len = build_tx_frame(
					tx_buf, anchor_id, frame_seq, src_addr,
					res->ao_records, res->ao_valid_mask,
					anchor_count);

				uwb_driver->setup_tx_frame(dev, tx_buf, frame_len);
				uwb_driver->start_tx(dev, tx_ts);

				/* Wait for TX IRQ */
				uwb_driver->release_device(dev);
				uwb_irq_state_e irq = uwb_driver->wait_for_irq(dev);
				uwb_driver->acquire_device(dev);

				if (irq != UWB_IRQ_TX) {
					LOG_ERR("TX failed round=%u slot=%u irq=%s",
						round, slot,
						irq_state_to_string(irq));
					ret = -EIO;
					goto cleanup;
				}

				LOG_DBG("TX ok round=%u slot=%u", round, slot);

			} else {
				/* ========== RX SLOT ========== */

				uwb_ts_t rx_enable_ts = 0;

				/*
				 * For the very first RX in a round, schedule
				 * the receiver at round_start_ts (delayed RX)
				 * if we are not anchor 0.  After the first slot
				 * we use immediate RX (the driver already
				 * switched buffers).
				 */
				if (slot == 0 && round == 0) {
					/* First slot of first round: delayed RX
					 * at round_start_ts */
					rx_enable_ts = conf->round_start_ts
						       & UWB_TS_MASK;
				} else if (slot == 0 && round > 0 && !is_tag &&
					   anchor_id == 0) {
					/* Anchor 0 at start of subsequent round:
					 * it just TXed last slot of prev round
					 * -- impossible, anchor_id==0 means
					 * slot==0 is TX. This branch is only
					 * reachable for tags. Use immediate. */
					rx_enable_ts = 0;
				}
				/* All other cases: immediate RX (rx_enable_ts=0) */

				uwb_driver->enable_rx(dev, 0, rx_enable_ts);

				/* Wait for RX IRQ (release lock while blocking) */
				uwb_driver->release_device(dev);
				uwb_irq_state_e irq = uwb_driver->wait_for_irq(dev);
				uwb_driver->acquire_device(dev);

				if (irq == UWB_IRQ_FRAME_WAIT_TIMEOUT ||
				    irq == UWB_IRQ_PREAMBLE_DETECT_TIMEOUT) {
					LOG_WRN("RX timeout round=%u slot=%u",
						round, slot);
					/* Mark slot invalid, continue */
					goto next_slot;
				}

				if (irq == UWB_IRQ_ERR) {
					LOG_WRN("RX error round=%u slot=%u",
						round, slot);
					goto next_slot;
				}

				if (irq != UWB_IRQ_RX) {
					LOG_ERR("Unexpected IRQ %s round=%u slot=%u",
						irq_state_to_string(irq),
						round, slot);
					ret = -EIO;
					goto cleanup;
				}

				/* -- Successfully received a frame -- */
				uint16_t pkt_len = uwb_driver->get_rx_frame_length(dev);
				uint16_t raw_len = (pkt_len > FRAME_LENGTH_ADDITIONAL) ?
					(pkt_len - FRAME_LENGTH_ADDITIONAL) : 0;

				if (raw_len < MULOC_MAC_HEADER_LEN + 1) {
					LOG_WRN("Frame too short: %u bytes", raw_len);
					uwb_driver->switch_buffers(dev);
					goto next_slot;
				}

				if (raw_len > sizeof(rx_buf)) {
					raw_len = sizeof(rx_buf);
				}

				/* Read RX timestamp and diagnostics */
				uwb_rx_diagnostics_t rx_diag;
				uwb_ts_t rx_ts = uwb_driver->read_rx_timestamp(dev, &rx_diag);

				/* Extract CIR first-path for our local reception */
				int16_t fp_re = 0, fp_im = 0;
				muloc_read_cir_first_path(dev, &rx_diag, &fp_re, &fp_im);

				/* Read carrier integrator (for tag mode CFO) */
				int32_t ci = 0;
				if (is_tag) {
					ci = uwb_driver->read_carrier_integrator(dev);
				}

				/* Read the raw frame */
				uwb_driver->read_rx_frame(dev, rx_buf, raw_len, 0);
				uwb_driver->switch_buffers(dev);

				/* Validate msg_id */
				if (rx_buf[9] != UWB_MULOC_BROADCAST) {
					LOG_WRN("Bad msg_id 0x%02x round=%u slot=%u",
						rx_buf[9], round, slot);
					goto next_slot;
				}

				/* Store reception data in result */
				struct muloc_ao_record *ao = &res->ao_records[slot];
				ao->cir_re = (uint16_t)fp_re;
				ao->cir_im = (uint16_t)fp_im;
				ao->rcphase = rx_diag.rx_phase;
				ao->rx_pacc = (uint8_t)(rx_diag.rx_pacc & 0xFF);
				/* Use cir_pwr as max_growth_cir proxy */
				ao->max_growth_cir = (uint16_t)(rx_diag.cir_pwr & 0xFFFF);
				to_packed_uwb_ts(ao->rx_ts, rx_ts);

				res->ao_valid_mask |= BIT(slot);

				if (is_tag) {
					struct muloc_to_record *to = &res->to_records[slot];
					to->ao = *ao;
					to->carrier_integrator = ci;
					res->to_valid_mask |= BIT(slot);
				}

				last_rx_ts = rx_ts;

				LOG_DBG("RX ok round=%u slot=%u ts=0x%010llx",
					round, slot, rx_ts);
			}

next_slot:
			; /* continue to next slot */
		}

		res->frame_seq = frame_seq;

		/* Frequency hopping: switch channel after every 2 rounds */
		if ((round % 2) == 1 && (round + 1) < conf->num_rounds) {
			if (current_channel == conf->start_channel) {
				current_channel = conf->hop_channel;
			} else {
				current_channel = conf->start_channel;
			}
			uwb_driver->set_channel(dev, current_channel);
		}

		frame_seq++;
		completed_rounds++;
	}

	ret = completed_rounds;

cleanup:
	uwb_driver->setup_preamble_timeout(dev, 0);
	uwb_driver->setup_frame_timeout(dev, 0);
	uwb_driver->disable_cir_access(dev);
	uwb_driver->release_device(dev);

	/*
	 * Yield so that any stale IRQ work item left over from the last
	 * slot can drain before the caller re-uses the device.
	 * (Same workaround as ranging_mm.c / deca_mm_reference.)
	 */
	k_yield();

	return ret;
}
