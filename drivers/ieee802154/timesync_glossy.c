#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/ieee802154/uwb_frame_utils.h>
#include <app/drivers/debug/timesync_debug_gpio.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */

#include <math.h>
#include <zephyr/drivers/timer/nrf_rtc_timer.h>
#include <zephyr/logging/log.h>

#include <app/drivers/ieee802154/uwb_irq_broker.h>

LOG_MODULE_REGISTER(timesync_glossy, LOG_LEVEL_INF);

// TODO: we don't receive the full 128 pacc symbols, is our timing completely correct here? Maybe check phy_activate_rx_delay again

#define MAX_GLOSSY_PAYLOAD 10
struct __attribute__((__packed__)) dwt_glossy_frame_buffer {
	uint8_t msg_id;
	uint8_t hop_count;
	uint16_t root_node_id;  // Address of the node that initiated this flood round
	uint32_t rtc_initiation_timestamp;
	uwb_packed_ts_t dwt_initiation_timestamp;
	uint8_t payload_size;
	uint8_t payload[MAX_GLOSSY_PAYLOAD];
};

static uint8_t received_glossy_payload[MAX_GLOSSY_PAYLOAD];


static K_SEM_DEFINE(read_dwt_sys_clock, 0, 1);
static uint64_t dwt_start_ts = 0;
void read_deca_system_timestamp(int32_t id, uint64_t expire_time, void *user_data) {
	const struct device *dev = (const struct device *)user_data;
	const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
	if (uwb_driver) {
		dwt_start_ts = uwb_driver->system_timestamp(dev);
	}
	k_sem_give(&read_dwt_sys_clock);
}

/* ========================================================================== */
/* Generic glossy flooding primitive                                           */
/* ========================================================================== */

int uwb_glossy_flood(const struct device *dev,
	struct uwb_flood_config *conf, struct uwb_flood_result *result) {
	int ret = 0;

	uwb_irq_state_e irq_state = UWB_IRQ_ERR;
	uint64_t initiator_rtc_ts = 0;
	uwb_ts_t initiator_dwt_ts = 0;

	uint16_t timeout_us = conf->max_depth * conf->transmission_delay_us;

	/* Initialize result */
	result->measured_constant_delay_us = -1;
	result->hop_count = 0;
	result->payload_size = 0;
	result->payload = NULL;
	result->initiator_rtc_ts = 0;
	result->initiator_dwt_ts = 0;
	result->local_rtc_ts = 0;
	result->local_dwt_ts = 0;

	const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
	if (!uwb_driver) {
		LOG_ERR("No UWB driver found for device");
		return -ENODEV;
	}

	/* Prevent execution of multiple ranging tasks */
	if (uwb_driver->acquire_device(dev) != 0) {
		LOG_ERR("Transceiver busy");
		return -EBUSY;
	}

	/* Disable preamble timeout (use frame wait timeout only via enable_rx) */
	uwb_driver->setup_preamble_timeout(dev, 0);

	if (conf->payload_size > MAX_GLOSSY_PAYLOAD) {
		LOG_ERR("Flood payload too large, passed %u, max %u", conf->payload_size, MAX_GLOSSY_PAYLOAD);
		ret = -EINVAL;
		goto cleanup;
	}

	uwb_driver->disable_txrx(dev);
	uwb_driver->set_frame_filter(dev, 0, 0);
	uwb_driver->align_double_buffering(dev);

	/* --- Initiator path --- */
	if (conf->is_initiator) {
		LOG_DBG("INITIATOR: Starting flood round, frame_id=0x%02x, max_depth=%u, tx_delay=%uus, guard=%uus, timeout=%uus",
			conf->frame_id, conf->max_depth, conf->transmission_delay_us, conf->guard_period_us, timeout_us);

		initiator_rtc_ts = k_cycle_get_32() + 2;
		z_nrf_rtc_timer_set(1, initiator_rtc_ts, read_deca_system_timestamp, (void *)dev);
		if (k_sem_take(&read_dwt_sys_clock, K_MSEC(100)) != 0) {
			LOG_ERR("INITIATOR: Failed to read system timestamp");
			ret = -EIO;
			goto cleanup;
		}

		initiator_dwt_ts = (dwt_start_ts + uwb_driver->us_to_timestamp(dev, conf->transmission_delay_us + conf->guard_period_us)) & UWB_TS_MASK;
		LOG_DBG("INITIATOR: Scheduled TX at dwt_ts=0x%llx (rtc=%u)", initiator_dwt_ts, initiator_rtc_ts);

		struct dwt_glossy_frame_buffer initial_frame = {
			.msg_id = conf->frame_id,
			.hop_count = 0,
			.root_node_id = conf->node_addr,
			.rtc_initiation_timestamp = initiator_rtc_ts,
			.payload_size = 0,
		};

		to_packed_uwb_ts(initial_frame.dwt_initiation_timestamp, initiator_dwt_ts);

		/* Copy payload if provided */
		if (conf->payload && conf->payload_size > 0) {
			memcpy(initial_frame.payload, conf->payload, conf->payload_size);
			initial_frame.payload_size = conf->payload_size;
		}

		uwb_driver->setup_tx_frame(dev, (uint8_t *)&initial_frame,
			offsetof(struct dwt_glossy_frame_buffer, payload) + initial_frame.payload_size);

		uwb_driver->start_tx(dev, initiator_dwt_ts & UWB_TS_MASK);
		LOG_DBG("INITIATOR: TX started, waiting for TX IRQ...");

		/* Populate result for initiator */
		result->initiator_node_id = conf->node_addr;
		result->hop_count = 0;
		result->initiator_rtc_ts = initiator_rtc_ts;
		result->initiator_dwt_ts = initiator_dwt_ts;
		result->local_rtc_ts = initiator_rtc_ts;  /* initiator: local == initiator */
		result->local_dwt_ts = initiator_dwt_ts;

		/* Copy payload into static buffer */
		memcpy(received_glossy_payload, initial_frame.payload, initial_frame.payload_size);
		result->payload = received_glossy_payload;
		result->payload_size = conf->payload_size;

		/* Wait for TX completion */
		uwb_driver->release_device(dev);
		k_timeout_t tx_wait = K_USEC(conf->transmission_delay_us + conf->guard_period_us + 1000);
		irq_state = uwb_broker_glossy_wait(dev, tx_wait);
		LOG_DBG("glossy done waiting for irq, state=%d", irq_state);
		if (irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT) {
			LOG_WRN("glossy wait timed out waiting for TX IRQ");
		}

		uwb_driver->acquire_device(dev);
		LOG_DBG("INITIATOR: TX IRQ received, state=%d", irq_state);
	} else {
		/* --- Receiver path --- */
		LOG_DBG("RECEIVER: Starting RX attempts, frame_id=0x%02x, max_depth=%u, timeout=%uus",
			conf->frame_id, conf->max_depth, timeout_us);

		bool success = false;
		for (size_t k = 0; !success && (k < conf->max_depth || !conf->max_depth); k++) {
			uint32_t rx_timeout = timeout_us + (timeout_us > 0 ? conf->guard_period_us : 0);
			LOG_DBG("RECEIVER: RX attempt %u/%u, timeout=%uus",
				k + 1, conf->max_depth, rx_timeout);

			uwb_driver->enable_rx(dev, rx_timeout, 0);

			uwb_driver->release_device(dev);
			k_timeout_t rx_wait = K_USEC(rx_timeout + 1000);
			irq_state = uwb_broker_glossy_wait(dev, rx_wait);
			uwb_driver->acquire_device(dev);

			LOG_DBG("RECEIVER: RX attempt %u IRQ state=%d", k + 1, irq_state);
			if (irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT) {
			  LOG_DBG("RECEIVER: RX wait timed out, resetting transceiver");
				uwb_driver->force_trx_off(dev);
				uwb_driver->clear_timeouts(dev);
				continue;
			}

			if (irq_state == UWB_IRQ_RX) {
				uint32_t local_rtc_ts = k_cycle_get_32();

				struct dwt_glossy_frame_buffer glossy_frame;
				uint8_t buf[sizeof(struct dwt_glossy_frame_buffer) + FRAME_LENGTH_ADDITIONAL];
				size_t glossy_header_len = offsetof(struct dwt_glossy_frame_buffer, payload);
				size_t glossy_min_len = glossy_header_len + FRAME_LENGTH_ADDITIONAL;

				uwb_rx_diagnostics_t rx_diag;
				uint64_t local_dwt_ts = uwb_driver->read_rx_timestamp(dev, &rx_diag);

				uint16_t pkt_len = uwb_driver->get_rx_frame_length(dev);

				LOG_DBG("RECEIVER: RX diagnostics: pacc=%u cir_pwr=%u fp_index=%u pkt_len=%u",
					rx_diag.rx_pacc, rx_diag.cir_pwr, rx_diag.fp_index, pkt_len);

				if (pkt_len > sizeof(buf)) {
					uwb_driver->switch_buffers(dev);
					LOG_WRN("RECEIVER: Frame too large for glossy buffer (%u > %u), discarding",
						pkt_len, (unsigned)sizeof(buf));
					success = false;
					continue;
				}

				if (pkt_len < glossy_min_len) {
					uwb_driver->switch_buffers(dev);
					LOG_WRN("RECEIVER: Frame too short for glossy header (%u < %u), discarding",
						pkt_len, (unsigned)glossy_min_len);
					success = false;
					continue;
				}

				uwb_driver->read_rx_frame(dev, buf, pkt_len, 0);

				if (buf[0] != conf->frame_id) {
					uwb_driver->switch_buffers(dev);
					LOG_DBG("RECEIVER: Wrong frame id, expected 0x%02x, got 0x%02x (discarding, retrying)",
						conf->frame_id, buf[0]);
					success = false;
					continue;
				}

				memcpy(&glossy_frame, buf, pkt_len - FRAME_LENGTH_ADDITIONAL);

				if (glossy_frame.payload_size > MAX_GLOSSY_PAYLOAD) {
					uwb_driver->switch_buffers(dev);
					LOG_WRN("RECEIVER: Invalid payload_size=%u, discarding",
						glossy_frame.payload_size);
					success = false;
					continue;
				}

				if (pkt_len != glossy_header_len + glossy_frame.payload_size + FRAME_LENGTH_ADDITIONAL) {
					uwb_driver->switch_buffers(dev);
					LOG_WRN("RECEIVER: Length mismatch (pkt=%u, expected=%u), discarding",
						pkt_len, (unsigned)(glossy_header_len + glossy_frame.payload_size + FRAME_LENGTH_ADDITIONAL));
					success = false;
					continue;
				}

				if (conf->max_depth > 0 && glossy_frame.hop_count > conf->max_depth) {
					uwb_driver->switch_buffers(dev);
					LOG_WRN("RECEIVER: Invalid hop_count=%u (max=%u), discarding",
						glossy_frame.hop_count, conf->max_depth);
					success = false;
					continue;
				}

				success = true;
				LOG_ERR("RECEIVER: RX success, hop_count=%u, payload_size=%u, rx_ts=0x%llx",
					glossy_frame.hop_count, glossy_frame.payload_size, local_dwt_ts);

				glossy_frame.hop_count++;

				uwb_driver->switch_buffers(dev);

        // TODO: Remove
        uwb_driver->force_trx_off(dev);

				uwb_driver->setup_tx_frame(dev, (uint8_t *)&glossy_frame,
					offsetof(struct dwt_glossy_frame_buffer, payload) + glossy_frame.payload_size);

				uwb_ts_t tx_delay_dtu = uwb_driver->us_to_timestamp(dev, conf->transmission_delay_us);
				uwb_ts_t programmed_tx_ts = (local_dwt_ts + tx_delay_dtu) & UWB_TS_MASK;

				uwb_driver->start_tx(dev, programmed_tx_ts);
				/* timesync_debug_pulse(); */

				/* Populate result with raw timestamps (no sync computation) */
				result->initiator_node_id = glossy_frame.root_node_id;
				result->hop_count = glossy_frame.hop_count; /* already incremented */
				result->initiator_rtc_ts = glossy_frame.rtc_initiation_timestamp;
				result->initiator_dwt_ts = from_packed_uwb_ts(glossy_frame.dwt_initiation_timestamp);
				result->local_rtc_ts = local_rtc_ts;
				result->local_dwt_ts = local_dwt_ts;

				/* Copy payload */
				memcpy(received_glossy_payload, glossy_frame.payload, glossy_frame.payload_size);
				result->payload_size = glossy_frame.payload_size;
				result->payload = received_glossy_payload;

				/* Wait for retransmit TX completion */
				uwb_driver->release_device(dev);
				k_timeout_t tx_wait = K_USEC(conf->transmission_delay_us + conf->guard_period_us + 1000);
				irq_state = uwb_broker_glossy_wait(dev, tx_wait);
				LOG_DBG("glossy done waiting for irq, state=%d", irq_state);
				if (irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT) {
					LOG_WRN("glossy wait timed out waiting for TX IRQ");
				}
				uwb_driver->acquire_device(dev);
			}
		}

    if(!success) {
      LOG_ERR("RECEIVER: Failed to receive glossy flood after %u attempts", conf->max_depth);
      ret = -EIO;
      goto cleanup;
    }
	}

	if (irq_state == UWB_IRQ_TX) {
		/* timesync_debug_pulse(); */
	}

	/* Error handling */
	if (irq_state == UWB_IRQ_ERR) {
		LOG_ERR("Flood failed: IRQ_ERR");
		ret = -EIO;
		goto cleanup;
	} else if (irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT ||
		   irq_state == UWB_IRQ_PREAMBLE_DETECT_TIMEOUT) {
		LOG_DBG("Flood timeout: irq_state=%d, is_initiator=%d, max_depth=%u, timeout=%uus",
			irq_state, conf->is_initiator, conf->max_depth, timeout_us);
		ret = -ETIMEDOUT;
		goto cleanup;
	} else if (irq_state != UWB_IRQ_TX) {
		LOG_DBG("Flood unexpected IRQ state: %d", irq_state);
	}

	/* Constant delay measurement (initiator only) */
	if (conf->is_initiator) {
		LOG_DBG("INITIATOR: Enabling RX to measure constant delay from first retransmission");
		/* timesync_debug_pulse(); */
		uint32_t new_initiator_rtc_ts;
		uint32_t root_rx_timeout_us = conf->transmission_delay_us + conf->guard_period_us + 1000;
		LOG_DBG("INITIATOR: RX timeout set to %uus (tx_delay=%u + guard=%u + margin=1000)",
			root_rx_timeout_us, conf->transmission_delay_us, conf->guard_period_us);
		uwb_driver->enable_rx(dev, root_rx_timeout_us, 0);

		uwb_driver->release_device(dev);
    // TODO: Does K_FOREVER cause deadlock? shouldn't be possible because we specify a timeout above...
		irq_state = uwb_broker_glossy_wait(dev, K_USEC(root_rx_timeout_us));
		uwb_driver->acquire_device(dev);

		LOG_DBG("INITIATOR: RX measurement IRQ received, state=%d", irq_state);
		if (irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT) {
			LOG_DBG("INITIATOR: RX measurement wait timed out, resetting transceiver");
			uwb_driver->force_trx_off(dev);
			uwb_driver->clear_timeouts(dev);
		}

		if (irq_state == UWB_IRQ_RX) {
			new_initiator_rtc_ts = k_cycle_get_32();

			struct dwt_glossy_frame_buffer glossy_frame;
			uint8_t buf[sizeof(struct dwt_glossy_frame_buffer) + FRAME_LENGTH_ADDITIONAL];

			uint16_t pkt_len = uwb_driver->get_rx_frame_length(dev);

			if (pkt_len > sizeof(buf)) {
				uwb_driver->switch_buffers(dev);
				LOG_WRN("INITIATOR: Measurement frame too large (%u > %u), ignoring",
					pkt_len, (unsigned)sizeof(buf));
				goto cleanup;
			}

			uwb_driver->read_rx_frame(dev, buf, pkt_len, 0);
			if (buf[0] != conf->frame_id) {
				uwb_driver->switch_buffers(dev);
				LOG_ERR("INITIATOR: Wrong frame id in measurement RX, expected 0x%02x, got 0x%02x",
					conf->frame_id, buf[0]);
				ret = -EIO;
				goto cleanup;
			}

			if (pkt_len < FRAME_LENGTH_ADDITIONAL) {
				uwb_driver->switch_buffers(dev);
				LOG_ERR("INITIATOR: Measurement frame too short (%u bytes)", pkt_len);
				ret = -EIO;
				goto cleanup;
			}

			memcpy(&glossy_frame, buf, pkt_len - FRAME_LENGTH_ADDITIONAL);

			uint32_t measured_round_trip_us = (uint32_t)(((new_initiator_rtc_ts - initiator_rtc_ts) * 1000000) / CONFIG_SYS_CLOCK_TICKS_PER_SEC);
			result->measured_constant_delay_us = measured_round_trip_us - 2 * conf->transmission_delay_us;
			LOG_INF("INITIATOR: Constant delay measurement: round_trip=%uus, tx_delay=%uus, constant_delay=%dus (config=%d)",
				measured_round_trip_us, conf->transmission_delay_us, result->measured_constant_delay_us, CONFIG_SYNCHROFLY_GLOSSY_CONSTANT_DELAY_US);

			uwb_driver->switch_buffers(dev);
		} else {
			/* timesync_debug_pulse(); */
			LOG_DBG("INITIATOR: Failed to receive retransmission for constant delay measurement, irq_state=%d", irq_state);
		}
	}

cleanup:
	uwb_driver->setup_preamble_timeout(dev, 0);
	uwb_driver->release_device(dev);
	k_yield();
	return ret;
}

/* ========================================================================== */
/* Time-synchronization wrapper (backward-compatible API)                      */
/* ========================================================================== */

int deca_glossy_time_synchronization(const struct device *dev,
	struct deca_glossy_configuration *conf, struct deca_glossy_result *result) {

	/* Translate legacy config to generic flood config */
	struct uwb_flood_config flood_conf = {
		.node_addr = conf->node_addr,
		.is_initiator = conf->isRoot,
		.guard_period_us = conf->guard_period_us,
		.max_depth = conf->max_depth,
		.transmission_delay_us = conf->transmission_delay_us,
		.payload = conf->payload,
		.payload_size = conf->payload_size,
		.frame_id = UWB_MTM_GLOSSY_TX_ID,
	};

	if (conf->payload_size > 0 && !conf->isRoot) {
		LOG_WRN("Only the root node may provide a payload, ignoring");
		flood_conf.payload = NULL;
		flood_conf.payload_size = 0;
	}

	struct uwb_flood_result flood_result;
	int ret = -EBUSY;
  if (uwb_broker_acquire_lease(dev) == 0) {
      ret = uwb_glossy_flood(dev, &flood_conf, &flood_result);
      uwb_broker_release_lease(dev);
  }

	/* Always populate basic result fields */
	result->root_node_id = flood_result.initiator_node_id;
	result->dist_to_root = flood_result.hop_count;
	result->payload_size = flood_result.payload_size;
	result->payload = flood_result.payload;
	result->measured_constant_delay_us = flood_result.measured_constant_delay_us;

	if (ret < 0) {
		return ret;
	}

	/* Compute time synchronization pairs from raw timestamps */
	struct deca_glossy_time_pair *rtc_inst = &result->rtc_clock_pair;
	struct deca_glossy_time_pair *dwt_inst = &result->deca_clock_pair;

	if (conf->isRoot) {
		/* Initiator: local == reference */
		rtc_inst->ref   = (int64_t)flood_result.initiator_rtc_ts;
		rtc_inst->local = (int64_t)flood_result.initiator_rtc_ts;
		dwt_inst->ref   = (int64_t)flood_result.initiator_dwt_ts;
		dwt_inst->local = (int64_t)flood_result.initiator_dwt_ts;
	} else {
		/* Receiver: compute corrected local timestamps from raw values */
		rtc_inst->ref = (int64_t)flood_result.initiator_rtc_ts;
		dwt_inst->ref = (int64_t)flood_result.initiator_dwt_ts;

		const uwb_driver_t *uwb_driver = uwb_driver_get(dev);

		rtc_inst->local = flood_result.local_rtc_ts
			- (((flood_result.hop_count * (uint64_t)conf->transmission_delay_us)
			    + CONFIG_SYNCHROFLY_GLOSSY_CONSTANT_DELAY_US + conf->guard_period_us)
			   * CONFIG_SYS_CLOCK_TICKS_PER_SEC) / 1000000;

		/* Subtract one from hop_count since we align RMARKERS, not IRQ issuance */
		dwt_inst->local = flood_result.local_dwt_ts
			- (flood_result.hop_count - 1) * uwb_driver->us_to_timestamp(dev, conf->transmission_delay_us);
	}

	return ret;
}
