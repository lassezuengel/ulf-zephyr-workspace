#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/ieee802154/uwb_frame_utils.h>
#include <app/drivers/debug/timesync_debug_gpio.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */

#include <math.h>
#include <zephyr/drivers/timer/nrf_rtc_timer.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(timesync_glossy, LOG_LEVEL_INF);

// TODO: we don't receive the full 128 pacc symbols, is our timing completely correct here? Maybe check phy_activate_rx_delay again

#define MAX_GLOSSY_PAYLOAD 50
struct __attribute__((__packed__)) dwt_glossy_frame_buffer {
	/* uint8_t msg_id : 4;     // 4 bits for msg_id */
	/* uint8_t hop_count : 4;  // 4 bits for hop_count */
	uint8_t msg_id;
	uint8_t hop_count;
	uint16_t root_node_id;  // Address of the root node that initiated this glossy round
	uint32_t rtc_initiation_timestamp;
	uwb_packed_ts_t dwt_initiation_timestamp;
	uint8_t payload_size;
	uint8_t payload[MAX_GLOSSY_PAYLOAD]; // payload will be located AFTER reception timestamps
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

int deca_glossy_time_synchronization(const struct device *dev,
	struct deca_glossy_configuration *conf, struct deca_glossy_result *result) {
	int ret = 0;
	struct deca_glossy_time_pair *rtc_inst = &result->rtc_clock_pair;
	struct deca_glossy_time_pair *dwt_inst = &result->deca_clock_pair;

	uwb_irq_state_e irq_state = UWB_IRQ_ERR;
	uint32_t initiator_rtc_ts, local_rtc_ts;
	uwb_ts_t initiator_dwt_ts, local_dwt_ts;
	uwb_ts_t programmed_tx_ts = 0;  // Track programmed transmission timestamp

	uint16_t timeout_us = conf->max_depth * conf->transmission_delay_us;

	// Initialize measured constant delay to -1 (not measured)
	result->measured_constant_delay_us = -1;

	const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
	if (!uwb_driver) {
		LOG_ERR("No UWB driver found for device");
		return -ENODEV;
	}

	// --- Prevent execution of multiple ranging tasks
	if (uwb_driver->acquire_device(dev) != 0) {
		LOG_ERR("Transceiver busy");
		return -EBUSY;
	}

	// Disable preamble timeout for glossy (use frame wait timeout only via enable_rx)
	uwb_driver->setup_preamble_timeout(dev, 0);

	if(conf->payload_size > MAX_GLOSSY_PAYLOAD) {
		LOG_ERR("Glossy Payload too large, passed %u, max %u", conf->payload_size, MAX_GLOSSY_PAYLOAD);
		ret = -EINVAL;
                goto cleanup;
	}

	if(conf->payload_size > 0 && !conf->isRoot) {
		LOG_WRN("Only the root node may provide a payload, ignoring");
	}

	uwb_driver->disable_txrx(dev);
	uwb_driver->set_frame_filter(dev, 0, 0);
	uwb_driver->align_double_buffering(dev); // for the following execution we require that host and receiver side are aligned

	// --- Round Initiation ---
	if (conf->isRoot) {
		LOG_DBG("ROOT: Starting Glossy round, max_depth=%u, tx_delay=%uus, guard=%uus, timeout=%uus",
			conf->max_depth, conf->transmission_delay_us, conf->guard_period_us, timeout_us);

		initiator_rtc_ts = k_cycle_get_32() + 2;
		z_nrf_rtc_timer_set(1, initiator_rtc_ts, read_deca_system_timestamp, (void*)dev);
		if(k_sem_take(&read_dwt_sys_clock, K_MSEC(100)) != 0) {
			LOG_ERR("ROOT: Failed to read system timestamp");
			ret = -EIO;
			goto cleanup;
		}

		initiator_dwt_ts = (dwt_start_ts + uwb_driver->us_to_timestamp(dev, conf->transmission_delay_us + conf->guard_period_us)) % UWB_TS_MASK;
		LOG_DBG("ROOT: Scheduled TX at dwt_ts=0x%llx (rtc=%u)", initiator_dwt_ts, initiator_rtc_ts);

		struct dwt_glossy_frame_buffer initial_glossy_frame = {
			.msg_id = UWB_MTM_GLOSSY_TX_ID,
			.hop_count = 0,
			.root_node_id = conf->node_addr,  // Root includes its own address
			.rtc_initiation_timestamp = initiator_rtc_ts,
			.payload_size = 0,
		};

		to_packed_uwb_ts(initial_glossy_frame.dwt_initiation_timestamp, initiator_dwt_ts);

		// Copy payload if provided
		if (conf->payload && conf->payload_size > 0) {
			memcpy(initial_glossy_frame.payload, conf->payload, conf->payload_size);
			initial_glossy_frame.payload_size = conf->payload_size;
		}

		uwb_driver->setup_tx_frame(dev, (uint8_t *)&initial_glossy_frame, offsetof(struct dwt_glossy_frame_buffer, payload) + initial_glossy_frame.payload_size);

		uwb_driver->start_tx(dev, initiator_dwt_ts & UWB_TS_MASK);
		LOG_DBG("ROOT: TX started, waiting for TX IRQ...");

		rtc_inst->ref   = (int64_t) initiator_rtc_ts;
		rtc_inst->local = (int64_t) initiator_rtc_ts;
		dwt_inst->ref   = (int64_t) initiator_dwt_ts;
		dwt_inst->local = (int64_t) initiator_dwt_ts;
		result->root_node_id = conf->node_addr;  // Root sets its own address
		result->dist_to_root = 0; // i am groot

		// copy payload into received glossy payload
		memcpy(received_glossy_payload, initial_glossy_frame.payload, initial_glossy_frame.payload_size);
		result->payload = received_glossy_payload;
		result->payload_size = conf->payload_size;

		// Release device lock before waiting for IRQ to avoid deadlock
		uwb_driver->release_device(dev);
		irq_state = uwb_driver->wait_for_irq(dev);
		// Reacquire device lock after IRQ
		uwb_driver->acquire_device(dev);
		LOG_DBG("ROOT: TX IRQ received, state=%d", irq_state);
	} else {
		LOG_DBG("NON-ROOT: Starting RX attempts, max_depth=%u, timeout=%uus",
			conf->max_depth, timeout_us);

		// retry this section to maybe get one of the other hops if one hop fails
		bool success = false;
		for(size_t k = 0; !success && (k < conf->max_depth || !conf->max_depth); k++) {
			LOG_DBG("NON-ROOT: RX attempt %u/%u, timeout=%uus",
				k+1, conf->max_depth, timeout_us + (timeout_us > 0 ? conf->guard_period_us : 0));

			uwb_driver->enable_rx(dev, timeout_us + (timeout_us > 0 ? conf->guard_period_us : 0), 0);

			// Release device lock before waiting for IRQ to avoid deadlock
			uwb_driver->release_device(dev);
			irq_state = uwb_driver->wait_for_irq(dev);
			// Reacquire device lock after IRQ
			uwb_driver->acquire_device(dev);

			LOG_DBG("NON-ROOT: RX attempt %u IRQ state=%d", k+1, irq_state);

			if(irq_state == UWB_IRQ_RX) {
				local_rtc_ts = k_cycle_get_32();
				success = true;

				// read received packet using UWB driver API
				struct dwt_glossy_frame_buffer glossy_frame;
				uint8_t buf[sizeof(struct dwt_glossy_frame_buffer) + FRAME_LENGTH_ADDITIONAL];

				// Get RX timestamp and diagnostics
				uwb_rx_diagnostics_t rx_diag;
				local_dwt_ts = uwb_driver->read_rx_timestamp(dev, &rx_diag);

				// Get frame length first, then read exactly that many bytes
				uint16_t pkt_len = uwb_driver->get_rx_frame_length(dev);
				uwb_driver->read_rx_frame(dev, buf, pkt_len, 0);

				if(buf[0] != UWB_MTM_GLOSSY_TX_ID) {
					uwb_driver->switch_buffers(dev);
					LOG_ERR("NON-ROOT: Wrong frame id, expected %u, got %u", UWB_MTM_GLOSSY_TX_ID, buf[0]);
					ret = -EIO;
					goto cleanup;
				}

				// Copy frame (assuming standard frame format without CRC)
				memcpy(&glossy_frame, buf, pkt_len-FRAME_LENGTH_ADDITIONAL);

				LOG_DBG("NON-ROOT: RX success, hop_count=%u, payload_size=%u, rx_ts=0x%llx",
					glossy_frame.hop_count, glossy_frame.payload_size, local_dwt_ts);

				glossy_frame.hop_count++;

				uwb_driver->switch_buffers(dev);

				uwb_driver->setup_tx_frame(dev, (uint8_t *)&glossy_frame, offsetof(struct dwt_glossy_frame_buffer, payload) + glossy_frame.payload_size);

				// Calculate transmission delay in DWT time units
				uwb_ts_t tx_delay_dtu = uwb_driver->us_to_timestamp(dev, conf->transmission_delay_us);
				programmed_tx_ts = (local_dwt_ts + tx_delay_dtu) % UWB_TS_MASK;

				uwb_driver->start_tx(dev, programmed_tx_ts);
				timesync_debug_pulse();  // Debug pulse on non-root retransmit
				// now we have some time for doing further work on the mcu without affecting the timing above

				// read from glossy frame
				rtc_inst->ref = (int64_t) glossy_frame.rtc_initiation_timestamp;
				dwt_inst->ref = (int64_t) from_packed_uwb_ts(glossy_frame.dwt_initiation_timestamp);

				rtc_inst->local = local_rtc_ts - (((glossy_frame.hop_count * (uint64_t) conf->transmission_delay_us)
                                        + CONFIG_SYNCHROFLY_GLOSSY_CONSTANT_DELAY_US + conf->guard_period_us) * CONFIG_SYS_CLOCK_TICKS_PER_SEC) / 1000000;
				/* Attention: here we subtract one from the hop_count since we align the RMARKERS, not the point
				 * where the reception/transmission commands are issued */
				// TODO check whether we want to offset by the guard period, in theory theres no strict requirement to perfectly align mcu and DWT time
				dwt_inst->local = local_dwt_ts - (glossy_frame.hop_count-1) * uwb_driver->us_to_timestamp(dev, conf->transmission_delay_us);

				// memcpy payload to received_glossy_payload
				memcpy(received_glossy_payload, glossy_frame.payload, glossy_frame.payload_size);
				result->root_node_id = glossy_frame.root_node_id;  // Extract root address from frame
				result->dist_to_root = glossy_frame.hop_count;
				result->payload_size = glossy_frame.payload_size;
				result->payload = received_glossy_payload;

                                // Release device lock before waiting for IRQ to avoid deadlock
                                uwb_driver->release_device(dev);
                                irq_state = uwb_driver->wait_for_irq(dev);
                                // Reacquire device lock after IRQ
                                uwb_driver->acquire_device(dev);

			}
		}
	}

        if(irq_state == UWB_IRQ_TX) {
            timesync_debug_pulse();  // Debug pulse when non-root TX completes
        }

	// transmission handling
	if(irq_state == UWB_IRQ_ERR) {
		LOG_ERR("Glossy failed: IRQ_ERR");
		ret = -EIO;
		goto cleanup;
	} else if (irq_state == UWB_IRQ_FRAME_WAIT_TIMEOUT ||
		   irq_state == UWB_IRQ_PREAMBLE_DETECT_TIMEOUT) {
		LOG_WRN("Glossy timeout: irq_state=%d, isRoot=%d, max_depth=%u, timeout=%uus",
			irq_state, conf->isRoot, conf->max_depth, timeout_us);
		ret = -ETIMEDOUT;
		goto cleanup;
	} else if (irq_state != UWB_IRQ_TX) {
		LOG_WRN("Glossy unexpected IRQ state: %d", irq_state);
	}

	// lets do something unconventional here and reenable the initiator again for rx, this allows us to measure the otherwise unmeasureable delay that is induced by
	// the time duration between reception of the frame and the processing of the subsequent interrupt. We can then, since we know the exact re-transmission duration
	// calculate this delay by comparing our initial local rtc timestamp and the one of the captured retransmission of the other node
	if (conf->isRoot)  {
		LOG_DBG("ROOT: Enabling RX to measure constant delay from first retransmission");
		timesync_debug_pulse();  // Debug pulse when root starts RX for constant delay measurement
		uint32_t new_initiator_rtc_ts;
		// Timeout must be long enough to receive first retransmission: transmission_delay + guard period + safety margin
		// The first retransmission arrives at ~transmission_delay_us after root TX
		uint32_t root_rx_timeout_us = conf->transmission_delay_us + conf->guard_period_us + 1000;
		LOG_DBG("ROOT: RX timeout set to %uus (tx_delay=%u + guard=%u + margin=1000)",
			root_rx_timeout_us, conf->transmission_delay_us, conf->guard_period_us);
		uwb_driver->enable_rx(dev, root_rx_timeout_us, 0);
		// Release device lock before waiting for IRQ to avoid deadlock
		uwb_driver->release_device(dev);
		irq_state = uwb_driver->wait_for_irq(dev);
		// Reacquire device lock after IRQ
		uwb_driver->acquire_device(dev);

		LOG_DBG("ROOT: RX measurement IRQ received, state=%d", irq_state);

		if(irq_state == UWB_IRQ_RX) {
			new_initiator_rtc_ts = k_cycle_get_32();

			// read received packet using UWB driver API
			struct dwt_glossy_frame_buffer glossy_frame;
			uint8_t buf[sizeof(struct dwt_glossy_frame_buffer) + FRAME_LENGTH_ADDITIONAL];

			// Get frame length first, then read exactly that many bytes
			uint16_t pkt_len = uwb_driver->get_rx_frame_length(dev);
			uwb_driver->read_rx_frame(dev, buf, pkt_len, 0);
			if(buf[0] != UWB_MTM_GLOSSY_TX_ID) {
				uwb_driver->switch_buffers(dev);
				LOG_ERR("ROOT: Wrong frame id in measurement RX, expected %u, got %u", UWB_MTM_GLOSSY_TX_ID, buf[0]);
				ret = -EIO;
				goto cleanup;
			}

			// Copy frame (assuming standard frame format without CRC)
			memcpy(&glossy_frame, buf, pkt_len-FRAME_LENGTH_ADDITIONAL);

			// Calculate measured constant delay: time from TX to receiving first retransmission minus transmission_delay
			uint32_t measured_round_trip_us = (uint32_t) (((new_initiator_rtc_ts - initiator_rtc_ts) * 1000000) / CONFIG_SYS_CLOCK_TICKS_PER_SEC);
			result->measured_constant_delay_us = measured_round_trip_us - 2*conf->transmission_delay_us;
			LOG_INF("ROOT: Constant delay measurement: round_trip=%uus, tx_delay=%uus, constant_delay=%dus (config=%d)",
				measured_round_trip_us, conf->transmission_delay_us, result->measured_constant_delay_us, CONFIG_SYNCHROFLY_GLOSSY_CONSTANT_DELAY_US);

			uwb_driver->switch_buffers(dev);
		} else {
			timesync_debug_pulse();  // Debug pulse on root RX timeout
			LOG_WRN("ROOT: Failed to receive retransmission for constant delay measurement, irq_state=%d", irq_state);
		}
	}

  cleanup:
	// Reset preamble timeout (restore default behavior)
	uwb_driver->setup_preamble_timeout(dev, 0);

	uwb_driver->release_device(dev);

	k_yield();

	return ret;
}
