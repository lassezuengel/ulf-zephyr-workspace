/**
 * @file block_cir_read.c
 * @brief CIR read block handler
 *
 * Each node is configured as sender or receiver. Senders transmit a probe
 * frame, receivers listen and read the DW1000 accumulator memory. Timing
 * is derived from glossy time synchronization with a fixed setup guard.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/lib/system/node.h>
#include <app/lib/system/hw.h>
#include <app/lib/blocks/cir_read.h>
#include <app/lib/timesync/time_synchronization.h>
#include <app/lib/system/block_heap.h>

LOG_MODULE_REGISTER(cir_read_block, CONFIG_LOG_DEFAULT_LEVEL);

/** Fixed setup guard to cover device acquisition, channel config, frame setup */
#define CIR_SETUP_GUARD_US 1000

void cir_read_block_handler(uint64_t event_time, void *user_data)
{
	struct cir_read_block_config *cfg = (struct cir_read_block_config *)user_data;

	const uwb_driver_t *drv = uwb_driver_get(ieee802154_dev);
	if (!drv) {
		LOG_ERR("CIR read: no UWB driver");
		return;
	}

	if (cfg->mode != CIR_MODE_SENDER && cfg->mode != CIR_MODE_RECEIVER) {
		LOG_ERR("CIR read: invalid mode %u", cfg->mode);
		return;
	}

	int ret = drv->acquire_device(ieee802154_dev);
	if (ret != 0) {
		LOG_ERR("CIR read: failed to acquire device: %d", ret);
		return;
	}

	/* Channel setup */
	uint8_t ch = cfg->channel;
	if (ch && drv->set_channel) {
		drv->set_channel(ieee802154_dev, ch);
	}

	/* Compute synchronized delayed timestamp from glossy time sync */
	uint64_t ref_deca_ts;
	uint64_t local_ts;
	bool have_sync = false;

	if (ref_rtc_to_deca(event_time, &ref_deca_ts) == 0 &&
	    get_deca_local_timestamp(ref_deca_ts + US_TO_DWT_TS(CIR_SETUP_GUARD_US),
				     &local_ts) >= 0) {
		have_sync = true;
	}

	if (cfg->mode == CIR_MODE_SENDER) {
		/* === SENDER: transmit CIR probe frame === */
		uint8_t frame[] = { CIR_PROBE_FRAME_ID };

		drv->disable_txrx(ieee802154_dev);
		drv->setup_tx_frame(ieee802154_dev, frame, sizeof(frame));

		uint64_t tx_ts = 0;
		if (have_sync) {
			tx_ts = (local_ts + ((uint64_t)cfg->tx_delay_dtu << 8))
				& UWB_TS_MASK;
		}
		drv->start_tx(ieee802154_dev, tx_ts);

		drv->release_device(ieee802154_dev);
		uwb_irq_state_e irq = drv->wait_for_irq(ieee802154_dev);
		drv->acquire_device(ieee802154_dev);

		if (irq != UWB_IRQ_TX) {
			LOG_WRN("CIR sender: unexpected IRQ %d", irq);
		}

		LOG_DBG("CIR sender: probe sent (delay_dtu=%u, sync=%d)",
			cfg->tx_delay_dtu, have_sync);
		goto cleanup;
	}

	/* === RECEIVER: listen + read CIR accumulator === */
	drv->disable_txrx(ieee802154_dev);
	drv->enable_cir_access(ieee802154_dev);
	drv->set_frame_filter(ieee802154_dev, 0, 0); /* accept any frame */
	drv->setup_frame_timeout(ieee802154_dev, 3000); /* 3ms timeout */

	uint64_t rx_ts = 0;
	if (have_sync) {
		rx_ts = local_ts & UWB_TS_MASK;
	}
	drv->enable_rx(ieee802154_dev, 3000, rx_ts);

	drv->release_device(ieee802154_dev);
	uwb_irq_state_e irq = drv->wait_for_irq(ieee802154_dev);
	drv->acquire_device(ieee802154_dev);

	if (irq != UWB_IRQ_RX) {
		LOG_WRN("CIR receiver: no frame received (irq=%d)", irq);
		if (cfg->cir_cb) {
			cfg->cir_cb(CIR_READ_STATUS_ERROR, NULL, cfg->cb_user_data);
		}
		goto cleanup_cir;
	}

	/* Read diagnostics for first path index */
	uwb_rx_diagnostics_t diag;
	drv->read_rx_timestamp(ieee802154_dev, &diag);
	uint16_t fp_idx = diag.fp_index >> 6;

	/* Calculate actual sample range */
	uint16_t from, to;
	if (cfg->only_first_path) {
		from = (fp_idx > cfg->from_index) ? (fp_idx - cfg->from_index) : 0;
		to = fp_idx + cfg->to_index;
	} else {
		from = cfg->from_index;
		to = cfg->to_index;
	}

	/* Clamp to accumulator bounds */
	if (to >= CIR_READ_MAX_SAMPLES) {
		to = CIR_READ_MAX_SAMPLES - 1;
	}
	if (from > to) {
		from = to;
	}

	uint16_t sample_count = to - from + 1;
	uint16_t byte_length = sample_count * 4;
	uint16_t byte_offset = from * 4;

	/* Heap-allocate CIR buffer: +1 for DW1000 SPI garbage byte */
	uint8_t *cir_buffer = block_malloc(1 + byte_length);
	if (!cir_buffer) {
		LOG_ERR("CIR read: failed to allocate %u bytes", 1 + byte_length);
		if (cfg->cir_cb) {
			cfg->cir_cb(CIR_READ_STATUS_ERROR, NULL, cfg->cb_user_data);
		}
		goto cleanup_cir;
	}

	/* Read CIR data (+1 for garbage byte, same pattern as ranging_mm.c) */
	drv->read_cir_data(ieee802154_dev, cir_buffer + 1, byte_offset, byte_length);

	LOG_INF("CIR read: %u samples [%u..%u], fp_idx=%u", sample_count, from, to, fp_idx);

	/* Fire callback with result */
	struct cir_read_block_result result = {
		.cir_data = cir_buffer + 1, /* skip garbage byte */
		.sample_count = sample_count,
		.first_path_index = fp_idx,
		.from_index = from,
		.channel = ch ? ch : 5,
	};

	if (cfg->cir_cb) {
		cfg->cir_cb(CIR_READ_STATUS_SUCCESS, &result, cfg->cb_user_data);
	}

	block_free(cir_buffer);

cleanup_cir:
	drv->disable_cir_access(ieee802154_dev);

cleanup:
	drv->release_device(ieee802154_dev);

	/* Restore channel to default (5) for glossy time synchronization */
	if (ch && drv->set_channel) {
		drv->set_channel(ieee802154_dev, 5);
	}
}
