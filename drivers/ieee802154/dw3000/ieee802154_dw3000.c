/*
 * Copyright (c) 2026 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ieee802154.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>

#include "dw3000.h"

LOG_MODULE_REGISTER(ieee802154_dw3000, LOG_LEVEL_WRN);

#define DW3000_FCS_LEN 2U
#define DW3000_MAX_PHY_PACKET_SIZE 127U
#define DW3000_TX_WAIT_MS 6
#define DW3000_TX_POLL_US 250
#define DW3000_CSMA_MAX_BACKOFFS 4U
#define DW3000_CSMA_MIN_BE 3U
#define DW3000_CSMA_MAX_BE 5U
#define DW3000_UNIT_BACKOFF_US 320U
#define DW3000_CCA_PTO_SYMBOLS 16U
#define DW3000_ACK_PKT_LEN 3U
#define DW3000_DUP_TRACK_SLOTS 16U
#define DW3000_TX_PG_DELAY 0x34U
#define DW3000_TX_POWER_CH5 0xFDFDFDFDUL

#define IEEE802154_FCF_FRAME_TYPE_MASK 0x0007U
#define IEEE802154_FCF_FRAME_TYPE_DATA 0x0001U
#define IEEE802154_FCF_PAN_ID_COMP 0x0040U
#define IEEE802154_FCF_DST_ADDR_MODE_MASK 0x0C00U
#define IEEE802154_FCF_DST_ADDR_MODE_SHIFT 10U
#define IEEE802154_FCF_SRC_ADDR_MODE_MASK 0xC000U
#define IEEE802154_FCF_SRC_ADDR_MODE_SHIFT 14U
#define IEEE802154_ADDR_MODE_NONE 0U
#define IEEE802154_ADDR_MODE_SHORT 2U
#define IEEE802154_ADDR_MODE_EXTENDED 3U

struct dw3000_dup_entry {
	uint32_t src_sig;
	uint8_t seq;
	uint32_t payload_hash;
	uint16_t payload_len;
	bool valid;
};

struct dw3000_data {
	struct net_if *iface;
	const uwb_driver_t *uwb;
	struct k_mutex lock;
	struct k_thread rx_thread;
	struct k_sem rx_irq_sem;
	atomic_t started;
	uint8_t mac_addr[8];
	uint16_t pan_id;
	uint16_t short_addr;
	uint16_t channel;
	bool promiscuous;
	uint32_t rx_ok_cnt;
	uint32_t rx_err_cnt;
	uint32_t tx_ok_cnt;
	uint32_t tx_err_cnt;
	uint32_t tx_attempt_cnt;
	uint32_t tx_entry_cnt;
	uint32_t rx_nobuf_cnt;
	bool rx_wait_logged;
	uint32_t rx_pkt_alloc_cnt;
	uint32_t rx_pkt_unref_cnt;
	uint32_t rx_frames_per_wake_cnt;
	uint32_t rx_fast_ack_cnt;
	uint32_t rx_capture_cnt;
	uint32_t rx_submit_cnt;
	uint32_t rx_same_payload_submit_cnt;
	uint32_t rx_last_payload_hash;
	uint16_t rx_last_payload_len;
	uint32_t rx_dup_drop_cnt;
	uint8_t rx_dup_next_slot;
	uint64_t rx_last_warn_ts;
	uint8_t rx_stage[DW3000_MAX_PHY_PACKET_SIZE];
	struct dw3000_dup_entry dup_tbl[DW3000_DUP_TRACK_SLOTS];
};

static const struct device *dw3000_irq_dev;

/*
 * Static ACK packet wrapper used for fast ACK handling without consuming
 * entries from the net_pkt/net_buf RX pools.
 */
static uint8_t dw3000_ack_psdu[DW3000_ACK_PKT_LEN];
static struct net_buf dw3000_ack_frame = {
	.data = dw3000_ack_psdu,
	.size = DW3000_ACK_PKT_LEN,
	.len = DW3000_ACK_PKT_LEN,
	.__buf = dw3000_ack_psdu,
	.frags = NULL,
};
static struct net_pkt dw3000_ack_pkt = {
	.buffer = &dw3000_ack_frame,
};

/**
 * Handler of the DWM_IRQ GPIO interrupt, which is triggered on RX events.
 * Used to wake the RX thread to process received packets.
 *
 * TODO: The existing UWB layer also uses this interrupt for its own purposes.
 * If we want to use both functionalities simultaneously, we may need to split
 * the interrupt handling so that the UWB layer can process the interrupt and
 * then signal the DW3000 driver to wake the RX thread, instead of having the
 * DW3000 driver directly handle the GPIO interrupt.
 *
 * -> Right now, the drivers will conflict!
 */
static void dw3000_irq_cb(void)
{
	const struct device *dev = dw3000_irq_dev;
	struct dw3000_data *data;

	if (dev == NULL) {
		return;
	}

	data = dev->data;
	k_sem_give(&data->rx_irq_sem);
}

struct dw3000_config {
	k_thread_stack_t *rx_stack;
	size_t rx_stack_size;
};

static int dw3000_start(const struct device *dev);

static int dw3000_apply_phy_config_locked(struct dw3000_data *data)
{
	dwt_txconfig_t tx_cfg;
	uint32_t tx_power;
	dwt_config_t phy_cfg = {
		.chan = 5U,
		.txPreambLength = DWT_PLEN_128,
		.rxPAC = DWT_PAC8,
		.txCode = 10,
		.rxCode = 10,
		.sfdType = DWT_SFD_IEEE_4A,
		.dataRate = DWT_BR_6M8,
		/* Match the existing DW3000/DW1000 profile. */
		.phrMode = DWT_PHRMODE_EXT,
		.phrRate = DWT_PHRRATE_STD,
		.sfdTO = 128 + 1 + 8 - 8,
		.stsMode = DWT_STS_MODE_OFF,
		.stsLength = DWT_STS_LEN_64,
		.pdoaMode = DWT_PDOA_M0,
	};

	if (dwt_configure(&phy_cfg) != DWT_SUCCESS) {
		return -EIO;
	}

	/*
	 * Ensure channel-dependent MRX LUT is refreshed after PHY reconfiguration.
	 */
	dwt_configmrxlut(5);

	/*
	 * Explicitly configure TX RF on every PHY/channel switch.
	 */
	tx_power = DW3000_TX_POWER_CH5;
	tx_cfg.PGdly = DW3000_TX_PG_DELAY;
	tx_cfg.power = tx_power;
	tx_cfg.PGcount = 0U;
	dwt_configuretxrf(&tx_cfg);

	LOG_INF("Applied PHY: ch=5 code=10 tx_pwr=0x%08x", tx_power);

	return 0;
}

static void dw3000_iface_api_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct dw3000_data *data = dev->data;
	int ret;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_IEEE802154);
	data->iface = iface;
	ieee802154_init(iface);

	/*
	 * Some integration flows don't trigger the radio start callback in time.
	 * Start RX proactively once the net interface is initialized.
	 *
	 * TODO: This is probably not needed because the driver should be started
	 * by Zephyr via the ieee802154 API.
	 */
	if (!atomic_get(&data->started)) {
		ret = dw3000_start(dev);
		if (ret) {
			LOG_WRN("Auto-start from iface init failed: %d", ret);
		} else {
			LOG_INF("Auto-started radio from iface init");
		}
	}
}

static void dw3000_generate_mac(uint8_t *mac)
{
	uint32_t lo = sys_rand32_get();
	uint32_t hi = sys_rand32_get();

	UNALIGNED_PUT(lo, (uint32_t *)&mac[0]);
	UNALIGNED_PUT(hi, (uint32_t *)&mac[4]);
	mac[0] = (mac[0] & ~0x01U) | 0x02U;
}

static int dw3000_apply_filter_hw(struct dw3000_data *data)
{
	uint16_t ff = DWT_FF_BEACON_EN | DWT_FF_DATA_EN | DWT_FF_ACK_EN |
		     DWT_FF_MAC_EN | DWT_FF_MULTI_EN;

	if (data->promiscuous) {
		dwt_configureframefilter(0, 0);
		return 0;
	}

	dwt_setpanid(data->pan_id);
	dwt_setaddress16(data->short_addr);
	dwt_seteui(data->mac_addr);
	dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, ff);

	return 0;
}

static inline void dw3000_clear_status_all(void);

static inline void dw3000_restart_rx_locked(void)
{
	dwt_forcetrxoff();
	dw3000_clear_status_all();
	dwt_setrxtimeout(0);
	dwt_setpreambledetecttimeout(0);
	(void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static inline void dw3000_clear_status_all(void)
{
	dwt_writesysstatuslo(DWT_INT_ALL_LO);
	dwt_writesysstatushi(DWT_INT_ALL_HI);
}

static uint32_t dw3000_payload_hash(const uint8_t *buf, uint16_t len)
{
	/* FNV-1a over up to 32 bytes keeps cost bounded while still being useful. */
	uint32_t h = 2166136261U;
	uint16_t n = MIN(len, 32U);
	uint16_t i;

	for (i = 0U; i < n; i++) {
		h ^= buf[i];
		h *= 16777619U;
	}

	h ^= len;
	h *= 16777619U;

	return h;
}

static bool dw3000_rx_is_duplicate(struct dw3000_data *data, const uint8_t *buf, uint16_t len,
				   bool commit)
{
	uint16_t fcf;
	uint8_t seq;
	uint8_t dst_mode;
	uint8_t src_mode;
	bool pan_comp;
	uint16_t off = 3U;
	uint16_t src_len;
	uint32_t src_sig = 2166136261U;
	uint32_t payload_hash;
	uint8_t i;
	int slot_same = -1;
	int slot_free = -1;
	uint8_t slot;

	if (len < 3U) {
		return false;
	}

	fcf = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
	if ((fcf & IEEE802154_FCF_FRAME_TYPE_MASK) != IEEE802154_FCF_FRAME_TYPE_DATA) {
		return false;
	}

	seq = buf[2];
	dst_mode = (uint8_t)((fcf & IEEE802154_FCF_DST_ADDR_MODE_MASK) >> IEEE802154_FCF_DST_ADDR_MODE_SHIFT);
	src_mode = (uint8_t)((fcf & IEEE802154_FCF_SRC_ADDR_MODE_MASK) >> IEEE802154_FCF_SRC_ADDR_MODE_SHIFT);
	pan_comp = (fcf & IEEE802154_FCF_PAN_ID_COMP) != 0U;

	if (dst_mode == IEEE802154_ADDR_MODE_SHORT) {
		off += 2U + 2U;
	} else if (dst_mode == IEEE802154_ADDR_MODE_EXTENDED) {
		off += 2U + 8U;
	}

	if (src_mode == IEEE802154_ADDR_MODE_NONE) {
		return false;
	}

	if (!pan_comp) {
		off += 2U;
	}

	if (src_mode == IEEE802154_ADDR_MODE_SHORT) {
		src_len = 2U;
	} else if (src_mode == IEEE802154_ADDR_MODE_EXTENDED) {
		src_len = 8U;
	} else {
		return false;
	}

	if ((off + src_len) > len) {
		return false;
	}

	payload_hash = dw3000_payload_hash(buf, len);

	for (i = 0U; i < src_len; i++) {
		src_sig ^= buf[off + i];
		src_sig *= 16777619U;
	}
	src_sig ^= src_mode;
	src_sig *= 16777619U;

	for (slot = 0U; slot < DW3000_DUP_TRACK_SLOTS; slot++) {
		if (!data->dup_tbl[slot].valid) {
			if (slot_free < 0) {
				slot_free = (int)slot;
			}
			continue;
		}

		if (data->dup_tbl[slot].src_sig == src_sig) {
			slot_same = (int)slot;
			break;
		}
	}

	if (slot_same >= 0) {
		if (data->dup_tbl[slot_same].seq == seq &&
		    data->dup_tbl[slot_same].payload_len == len &&
		    data->dup_tbl[slot_same].payload_hash == payload_hash) {
			return true;
		}
		if (commit) {
			data->dup_tbl[slot_same].seq = seq;
			data->dup_tbl[slot_same].payload_hash = payload_hash;
			data->dup_tbl[slot_same].payload_len = len;
		}
		return false;
	}

	if (!commit) {
		return false;
	}

	if (slot_free >= 0) {
		slot = (uint8_t)slot_free;
	} else {
		slot = data->rx_dup_next_slot;
		data->rx_dup_next_slot = (uint8_t)((data->rx_dup_next_slot + 1U) % DW3000_DUP_TRACK_SLOTS);
	}

	data->dup_tbl[slot].src_sig = src_sig;
	data->dup_tbl[slot].seq = seq;
	data->dup_tbl[slot].payload_hash = payload_hash;
	data->dup_tbl[slot].payload_len = len;
	data->dup_tbl[slot].valid = true;

	return false;
}

static int dw3000_rx_capture_locked(const struct device *dev, bool *ack_handled,
					    uint16_t *pkt_len_out)
{
	struct dw3000_data *data = dev->data;
	enum net_verdict ack_verdict;
	uint8_t rng = 0;
	uint16_t phy_len = 0;
	uint16_t pkt_len;

	if (ack_handled != NULL) {
		*ack_handled = false;
	}

	if (pkt_len_out != NULL) {
		*pkt_len_out = 0U;
	}

	if (data->uwb && data->uwb->get_rx_frame_length) {
		phy_len = data->uwb->get_rx_frame_length(dev);
	} else {
		phy_len = dwt_getframelength(&rng);
	}
	pkt_len = phy_len;

	if (phy_len < DW3000_FCS_LEN || phy_len > DW3000_MAX_PHY_PACKET_SIZE) {
		LOG_DBG("Drop invalid frame len=%u", phy_len);
		dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
		dw3000_restart_rx_locked();
		return 0;
	}

#if !defined(CONFIG_IEEE802154_RAW_MODE)
	pkt_len -= DW3000_FCS_LEN;
#endif

	/*
	 * Fast-path minimal ACK frames through a static wrapper packet so we do not
	 * consume net_pkt pool entries for control traffic during bursts.
	 */
	if (pkt_len == DW3000_ACK_PKT_LEN) {
		if (data->uwb && data->uwb->read_rx_frame) {
			data->uwb->read_rx_frame(dev, dw3000_ack_psdu, DW3000_ACK_PKT_LEN, 0);
		} else {
			dwt_readrxdata(dw3000_ack_psdu, DW3000_ACK_PKT_LEN, 0);
		}

		dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
		dw3000_restart_rx_locked();

		net_pkt_cursor_init(&dw3000_ack_pkt);
		ack_verdict = ieee802154_handle_ack(data->iface, &dw3000_ack_pkt);
		if (ack_handled != NULL && ack_verdict == NET_OK) {
			*ack_handled = true;
			data->rx_fast_ack_cnt++;
		}

		return 0;
	}

	if (data->uwb && data->uwb->read_rx_frame) {
		data->uwb->read_rx_frame(dev, data->rx_stage, pkt_len, 0);
	} else {
		dwt_readrxdata(data->rx_stage, pkt_len, 0);
	}

	dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
	dw3000_restart_rx_locked();

	if (pkt_len_out != NULL) {
		*pkt_len_out = pkt_len;
	}
	data->rx_capture_cnt++;

	return 1;
}

static void dw3000_rx_thread_fn(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct dw3000_data *data = dev->data;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		struct net_pkt *pkt = NULL;
		uint32_t status;
		uint16_t pkt_len = 0U;
		int rx_capture = 0;
		bool ack_handled = false;
		bool more_work_pending = false;

		if (!atomic_get(&data->started) || data->iface == NULL) {
			if (!data->rx_wait_logged) {
				LOG_INF("RX thread waiting (started=%ld iface=%p)",
					atomic_get(&data->started), data->iface);
				data->rx_wait_logged = true;
			}
			k_usleep(5000);
			continue;
		}

		data->rx_wait_logged = false;

#if defined(CONFIG_IEEE802154_DW3000_RX_POLLING_MODE)
		k_usleep(1000);
#else
		/*
		 * Do not block forever on IRQ semaphore: if IRQ edges are dropped while
		 * RX status remains pending, periodic wakeups allow recovery.
		 */
		(void)k_sem_take(&data->rx_irq_sem, K_MSEC(1));
#endif

		k_mutex_lock(&data->lock, K_FOREVER);
		status = dwt_readsysstatuslo();

		if (status & DWT_INT_RXFCG_BIT_MASK) {
			rx_capture = dw3000_rx_capture_locked(dev, &ack_handled, &pkt_len);
		} else if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
			data->rx_err_cnt++;
			if ((data->rx_err_cnt % 64U) == 0U) {
				LOG_INF("RX errors/timeouts=%u lo=0x%08x hi=0x%08x tx_entry=%u tx_attempt=%u tx_ok=%u tx_err=%u",
					data->rx_err_cnt, status, dwt_readsysstatushi(),
					data->tx_entry_cnt, data->tx_attempt_cnt,
					data->tx_ok_cnt, data->tx_err_cnt);
			}
			dwt_writesysstatuslo(status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));
			dw3000_restart_rx_locked();
		}

		status = dwt_readsysstatuslo();
		more_work_pending = (status & (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
					      SYS_STATUS_ALL_RX_ERR)) != 0U;

		k_mutex_unlock(&data->lock);

		if (more_work_pending) {
			k_sem_give(&data->rx_irq_sem);
		}

		if (ack_handled) {
			continue;
		}

		if (rx_capture <= 0 || pkt_len == 0U) {
			continue;
		}


    if (dw3000_rx_is_duplicate(data, data->rx_stage, pkt_len, false)) {
			data->rx_dup_drop_cnt++;
			if ((data->rx_dup_drop_cnt % 32U) == 0U) {
				LOG_WRN("RX duplicate drops=%u submit=%u capture=%u", data->rx_dup_drop_cnt,
					data->rx_submit_cnt, data->rx_capture_cnt);
			}
			continue;
		}

		/*
		 * Allocate outside the radio lock so RX restart is not blocked by
		 * transient net pool pressure.
		 */
		pkt = net_pkt_rx_alloc_with_buffer(data->iface, pkt_len, AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			int32_t driver_held = (int32_t)data->rx_pkt_alloc_cnt -
				(int32_t)data->rx_pkt_unref_cnt - (int32_t)data->rx_ok_cnt;

			data->rx_nobuf_cnt++;
			if ((data->rx_nobuf_cnt % 32U) == 0U) {
				uint64_t now_ts = k_uptime_get();
				LOG_WRN("RX dropped (no net_pkt): cnt=%u len=%u alloc=%u rx_ok=%u unref=%u held=%d fastack=%u",
					data->rx_nobuf_cnt, pkt_len, data->rx_pkt_alloc_cnt,
					data->rx_ok_cnt, data->rx_pkt_unref_cnt, driver_held,
					data->rx_fast_ack_cnt);
				data->rx_last_warn_ts = now_ts;
			}
			continue;
		}

		data->rx_pkt_alloc_cnt++;
		net_pkt_cursor_init(pkt);
		if (net_pkt_write(pkt, data->rx_stage, pkt_len) < 0) {
			net_pkt_unref(pkt);
			data->rx_pkt_unref_cnt++;
			continue;
		}
		net_pkt_cursor_init(pkt);

		if (ieee802154_handle_ack(data->iface, pkt) == NET_OK) {
			net_pkt_unref(pkt);
			data->rx_pkt_unref_cnt++;
			continue;
		}

		data->rx_submit_cnt++;
		{
			uint32_t payload_hash = dw3000_payload_hash(data->rx_stage, pkt_len);

			if (payload_hash == data->rx_last_payload_hash && pkt_len == data->rx_last_payload_len) {
				data->rx_same_payload_submit_cnt++;
				if ((data->rx_same_payload_submit_cnt % 64U) == 0U) {
					LOG_WRN("RX repeated payload submits=%u capture=%u submit=%u len=%u",
						data->rx_same_payload_submit_cnt,
						data->rx_capture_cnt,
						data->rx_submit_cnt,
						pkt_len);
				}
			}

			data->rx_last_payload_hash = payload_hash;
			data->rx_last_payload_len = pkt_len;
		}

		if (net_recv_data(data->iface, pkt) != NET_OK) {
			net_pkt_unref(pkt);
			data->rx_pkt_unref_cnt++;
		} else {
			(void)dw3000_rx_is_duplicate(data, data->rx_stage, pkt_len, true);
			data->rx_ok_cnt++;
			if ((data->rx_ok_cnt % 16U) == 0U) {
				LOG_INF("RX packets=%u capture=%u submit=%u", data->rx_ok_cnt,
					data->rx_capture_cnt, data->rx_submit_cnt);
			}
		}
	}
}

static enum ieee802154_hw_caps dw3000_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IEEE802154_HW_FCS | IEEE802154_HW_FILTER | IEEE802154_HW_TXTIME |
		IEEE802154_HW_CSMA;
}

static int dw3000_cca(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int dw3000_set_channel(const struct device *dev, uint16_t channel)
{
	struct dw3000_data *data = dev->data;
	int ret;

	if (channel == 9U) {
		LOG_ERR("Channel 9 is disabled in ieee802154_dw3000: no RX preamble detection observed in validation tests");
		return -ENOTSUP;
	}

	if (channel != 5U) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->channel = channel;
	ret = dw3000_apply_phy_config_locked(data);
	if (ret != 0) {
		LOG_WRN("PHY reconfigure failed for channel=%u", channel);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	ret = dwt_setchannel(DWT_CH5);
	if (ret != DWT_SUCCESS) {
		LOG_WRN("dwt_setchannel failed channel=%u ret=%d", channel, ret);
		k_mutex_unlock(&data->lock);
		return -EIO;
	}

	LOG_WRN("Channel set to %u", channel);
	k_mutex_unlock(&data->lock);

	return 0;
}

static int dw3000_filter(const struct device *dev,
			 bool set,
			 enum ieee802154_filter_type type,
			 const struct ieee802154_filter *filter)
{
	struct dw3000_data *data = dev->data;

	if (!filter) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	switch (type) {
	case IEEE802154_FILTER_TYPE_IEEE_ADDR:
		if (set) {
			memcpy(data->mac_addr, filter->ieee_addr, sizeof(data->mac_addr));
		}
		LOG_INF("Filter IEEE addr %s", set ? "set" : "clear");
		break;

	case IEEE802154_FILTER_TYPE_SHORT_ADDR:
		data->short_addr = set ? filter->short_addr : 0xffffU;
		LOG_INF("Filter short %s: 0x%04x", set ? "set" : "clear", data->short_addr);
		break;

	case IEEE802154_FILTER_TYPE_PAN_ID:
		data->pan_id = set ? filter->pan_id : 0xffffU;
		LOG_INF("Filter PAN %s: 0x%04x", set ? "set" : "clear", data->pan_id);
		break;

	default:
		k_mutex_unlock(&data->lock);
		return -ENOTSUP;
	}

	(void)dw3000_apply_filter_hw(data);
	k_mutex_unlock(&data->lock);

	return 0;
}

static int dw3000_set_txpower(const struct device *dev, int16_t dbm)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dbm);
	return 0;
}

static int dw3000_tx(const struct device *dev,
		     enum ieee802154_tx_mode mode,
		     struct net_pkt *pkt,
		     struct net_buf *frag)
{
	struct dw3000_data *data = dev->data;
	int64_t timeout_end;
	uint8_t start_mode = DWT_START_TX_IMMEDIATE;
	bool use_csma = false;
	uint8_t be = DW3000_CSMA_MIN_BE;
	uint8_t max_tries = 1U;
	uint8_t tx_try;
	int ret;

	ARG_UNUSED(pkt);

	data->tx_entry_cnt++;
	if (data->tx_entry_cnt <= 4U || (data->tx_entry_cnt % 256U) == 0U) {
		LOG_INF("TX entry=%u mode=%d frag=%p len=%u started=%ld",
			data->tx_entry_cnt, mode, frag,
			frag ? frag->len : 0U, atomic_get(&data->started));
	}

	if (!frag || frag->len == 0U) {
		LOG_WRN("TX reject: empty fragment");
		return -EINVAL;
	}

	if (!atomic_get(&data->started)) {
		LOG_WRN("TX reject: radio not started");
		return -ENETDOWN;
	}

	if ((frag->len + DW3000_FCS_LEN) > DW3000_MAX_PHY_PACKET_SIZE) {
		LOG_WRN("TX reject: frame too large len=%u", frag->len);
		return -EMSGSIZE;
	}

	if (mode == IEEE802154_TX_MODE_DIRECT || mode == IEEE802154_TX_MODE_TXTIME) {
		start_mode = DWT_START_TX_IMMEDIATE;
	} else if (mode == IEEE802154_TX_MODE_CCA || mode == IEEE802154_TX_MODE_TXTIME_CCA) {
		start_mode = DWT_START_TX_CCA;
	} else if (mode == IEEE802154_TX_MODE_CSMA_CA) {
		start_mode = DWT_START_TX_CCA;
		use_csma = true;
		max_tries = DW3000_CSMA_MAX_BACKOFFS + 1U;
	} else {
		LOG_WRN("Unsupported TX mode=%d", mode);
		return -ENOTSUP;
	}

	data->tx_attempt_cnt++;
	if (data->tx_attempt_cnt <= 8U || (data->tx_attempt_cnt % 64U) == 0U) {
		LOG_INF("TX attempt=%u len=%u mode=%d", data->tx_attempt_cnt, frag->len, mode);
	}

	for (tx_try = 0U; tx_try < max_tries; tx_try++) {
		if (use_csma && tx_try > 0U) {
			uint32_t bo_slots = 1U << MIN(be, DW3000_CSMA_MAX_BE);
			uint32_t rand_slots = sys_rand32_get() % bo_slots;
			k_usleep(rand_slots * DW3000_UNIT_BACKOFF_US);
			if (be < DW3000_CSMA_MAX_BE) {
				be++;
			}
		}

		k_mutex_lock(&data->lock, K_FOREVER);

		if (data->uwb && data->uwb->force_trx_off) {
			data->uwb->force_trx_off(dev);
		} else {
			dwt_forcetrxoff();
		}
		dw3000_clear_status_all();
		dwt_setpreambledetecttimeout((start_mode == DWT_START_TX_CCA) ? DW3000_CCA_PTO_SYMBOLS : 0U);

		if (data->uwb && data->uwb->setup_tx_frame && data->uwb->start_tx) {
			/* The UWB abstraction exposes immediate TX only, use direct starttx for CCA mode. */
			data->uwb->setup_tx_frame(dev, frag->data, (uint16_t)frag->len);
			if (start_mode == DWT_START_TX_IMMEDIATE) {
				ret = data->uwb->start_tx(dev, 0);
			} else {
				ret = dwt_starttx(start_mode);
			}
		} else {
			if (data->tx_entry_cnt <= 4U || (data->tx_entry_cnt % 256U) == 0U) {
				LOG_WRN("TX using legacy direct dwt_* path");
			}
			ret = dwt_writetxdata((uint16_t)frag->len, frag->data, 0);
			if (ret != DWT_SUCCESS) {
				data->tx_err_cnt++;
				LOG_WRN("TX data write failed ret=%d tx_err=%u", ret, data->tx_err_cnt);
				k_mutex_unlock(&data->lock);
				return -EIO;
			}

			dwt_writetxfctrl((uint16_t)frag->len + DW3000_FCS_LEN, 0, 0);
			ret = dwt_starttx(start_mode);
		}
		if (ret != DWT_SUCCESS) {
			uint32_t status_lo = dwt_readsysstatuslo();
			uint32_t status_hi = dwt_readsysstatushi();
			dw3000_restart_rx_locked();
			k_mutex_unlock(&data->lock);

			if (use_csma && (tx_try + 1U) < max_tries) {
				continue;
			}

			data->tx_err_cnt++;
			LOG_WRN("TX start failed ret=%d mode=%d lo=0x%08x hi=0x%08x tx_err=%u",
				ret, mode, status_lo, status_hi, data->tx_err_cnt);
			return -EIO;
		}

		timeout_end = k_uptime_get() + DW3000_TX_WAIT_MS;
		while (k_uptime_get() < timeout_end) {
			uint32_t status_lo = dwt_readsysstatuslo();
			uint32_t status_hi = dwt_readsysstatushi();

			if (status_lo & DWT_INT_TXFRS_BIT_MASK) {
				dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
				data->tx_ok_cnt++;
				if ((data->tx_ok_cnt % 16U) == 0U) {
					LOG_INF("TX packets=%u", data->tx_ok_cnt);
				}
				dw3000_restart_rx_locked();
				k_mutex_unlock(&data->lock);
				return 0;
			}

			if ((status_hi & DWT_INT_HI_CCA_FAIL_BIT_MASK) != 0U && start_mode == DWT_START_TX_CCA) {
				dwt_writesysstatushi(DWT_INT_HI_CCA_FAIL_BIT_MASK);
				dw3000_restart_rx_locked();
				k_mutex_unlock(&data->lock);

				if (use_csma && (tx_try + 1U) < max_tries) {
					break;
				}

				data->tx_err_cnt++;
				LOG_WRN("TX CCA busy mode=%d tries=%u tx_err=%u", mode, tx_try + 1U, data->tx_err_cnt);
				return -EBUSY;
			}

			if (status_lo & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)) {
				dwt_writesysstatuslo(status_lo & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO));
			}

			k_usleep(DW3000_TX_POLL_US);
		}

		if (k_uptime_get() >= timeout_end) {
			dw3000_restart_rx_locked();
			k_mutex_unlock(&data->lock);

			if (use_csma && (tx_try + 1U) < max_tries) {
				continue;
			}

			data->tx_err_cnt++;
			LOG_WRN("TX timeout lo=0x%08x hi=0x%08x tx_err=%u",
				dwt_readsysstatuslo(), dwt_readsysstatushi(), data->tx_err_cnt);
			return -EIO;
		}
	}

	data->tx_err_cnt++;
	LOG_WRN("TX CSMA exhausted tries=%u tx_err=%u", max_tries, data->tx_err_cnt);
	return -EBUSY;
}

static int dw3000_start(const struct device *dev)
{
	struct dw3000_data *data = dev->data;

	LOG_DBG("dw3000_start: Entry");
	k_mutex_lock(&data->lock, K_FOREVER);

	atomic_set(&data->started, 1);

	dwt_forcetrxoff();

	dw3000_clear_status_all();

	dw3000_restart_rx_locked();

	k_mutex_unlock(&data->lock);

	LOG_INF("Radio start: channel=%u pan=0x%04x short=0x%04x", data->channel, data->pan_id,
		data->short_addr);

#if !defined(CONFIG_IEEE802154_DW3000_RX_POLLING_MODE)
	dw3000_hw_interrupt_enable();
#endif

	return 0;
}

static int dw3000_stop(const struct device *dev)
{
	struct dw3000_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	atomic_set(&data->started, 0);
	dwt_forcetrxoff();
	k_mutex_unlock(&data->lock);

#if !defined(CONFIG_IEEE802154_DW3000_RX_POLLING_MODE)
	dw3000_hw_interrupt_disable();
	/* Wake RX thread so it can observe started=0 and park cleanly. */
	k_sem_give(&data->rx_irq_sem);
#endif

	return 0;
}

static int dw3000_configure(const struct device *dev,
			    enum ieee802154_config_type type,
			    const struct ieee802154_config *config)
{
	struct dw3000_data *data = dev->data;

	switch (type) {
	case IEEE802154_CONFIG_PROMISCUOUS:
		if (config == NULL) {
			return -EINVAL;
		}

		k_mutex_lock(&data->lock, K_FOREVER);
		data->promiscuous = config->promiscuous;
		(void)dw3000_apply_filter_hw(data);
		k_mutex_unlock(&data->lock);
		return 0;

	default:
		return -ENOTSUP;
	}
}

/* Driver-allocated attribute memory constant across all instances. */
static const struct {
	const struct ieee802154_phy_channel_range phy_channel_range[1];
	const struct ieee802154_phy_supported_channels phy_supported_channels;
} dw3000_drv_attr = {
	.phy_channel_range = {
		{ .from_channel = 5, .to_channel = 5 },
	},
	.phy_supported_channels = {
		.ranges = dw3000_drv_attr.phy_channel_range,
		.num_ranges = 1U,
	},
};

static int dw3000_attr_get(const struct device *dev, enum ieee802154_attr attr,
			   struct ieee802154_attr_value *value)
{
	ARG_UNUSED(dev);

	if (ieee802154_attr_get_channel_page_and_range(
		    attr, IEEE802154_ATTR_PHY_CHANNEL_PAGE_FOUR_HRP_UWB,
		    &dw3000_drv_attr.phy_supported_channels, value) == 0) {
		return 0;
	}

	return -ENOENT;
}

static const struct ieee802154_radio_api dw3000_radio_api = {
	.iface_api.init = dw3000_iface_api_init,
	.get_capabilities = dw3000_get_capabilities,
	.cca = dw3000_cca,
	.set_channel = dw3000_set_channel,
	.filter = dw3000_filter,
	.set_txpower = dw3000_set_txpower,
	.tx = dw3000_tx,
	.start = dw3000_start,
	.stop = dw3000_stop,
	.configure = dw3000_configure,
	.attr_get = dw3000_attr_get,
};

static int dw3000_init(const struct device *dev)
{
	struct dw3000_data *data = dev->data;
	const struct dw3000_config *cfg = dev->config;
	int ret;

	ret = uwb_driver_dw3000_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize DW3000 core: %d", ret);
		return ret;
	}

	/*
	 * The existing UWB layer installs its own IRQ work handler for ranging.
	 * For the IEEE802154 net path, we use a dedicated RX thread that is woken
	 * either by IRQ callbacks or by polling, depending on Kconfig.
	 */
	dw3000_hw_clear_interrupt_handler();
	dw3000_hw_interrupt_disable();

	k_mutex_init(&data->lock);
	data->uwb = uwb_driver_get(dev);
	if (data->uwb == NULL) {
		LOG_ERR("No registered UWB driver for %s", dev->name);
		return -ENODEV;
	}
	LOG_INF("UWB hooks: tx=%d rxlen=%d rxread=%d forceoff=%d",
		(data->uwb->start_tx && data->uwb->setup_tx_frame) ? 1 : 0,
		data->uwb->get_rx_frame_length ? 1 : 0,
		data->uwb->read_rx_frame ? 1 : 0,
		data->uwb->force_trx_off ? 1 : 0);
	atomic_set(&data->started, 0);
	dw3000_generate_mac(data->mac_addr);
	data->pan_id = 0xffffU;
	data->short_addr = 0xffffU;
	data->channel = 5U;
#if defined(CONFIG_NET_CONFIG_IEEE802154_CHANNEL)
	if (CONFIG_NET_CONFIG_IEEE802154_CHANNEL == 9) {
		LOG_ERR("CONFIG_NET_CONFIG_IEEE802154_CHANNEL=9 is unsupported for ieee802154_dw3000 (no RX preamble detection)");
		return -ENOTSUP;
	}
	if (CONFIG_NET_CONFIG_IEEE802154_CHANNEL == 5) {
		data->channel = CONFIG_NET_CONFIG_IEEE802154_CHANNEL;
	}
#endif
	data->promiscuous = false;
	data->rx_ok_cnt = 0U;
	data->rx_err_cnt = 0U;
	data->tx_ok_cnt = 0U;
	data->tx_err_cnt = 0U;
	data->tx_attempt_cnt = 0U;
	data->tx_entry_cnt = 0U;
	data->rx_nobuf_cnt = 0U;
	data->rx_wait_logged = false;
	data->rx_pkt_alloc_cnt = 0U;
	data->rx_pkt_unref_cnt = 0U;
	data->rx_frames_per_wake_cnt = 0U;
	data->rx_fast_ack_cnt = 0U;
	data->rx_capture_cnt = 0U;
	data->rx_submit_cnt = 0U;
	data->rx_same_payload_submit_cnt = 0U;
	data->rx_last_payload_hash = 0U;
	data->rx_last_payload_len = 0U;
	data->rx_dup_drop_cnt = 0U;
	data->rx_dup_next_slot = 0U;
	memset(data->dup_tbl, 0, sizeof(data->dup_tbl));
	data->rx_last_warn_ts = 0U;
	k_sem_init(&data->rx_irq_sem, 0, 64);

	k_mutex_lock(&data->lock, K_FOREVER);
	/*
	 * Use single-buffer RX mode for clarity and explicit control.
	 * Double-buffer mode allows hardware to switch between two RX buffers automatically,
	 * which is useful for high-speed interrupt handlers. In our case, the RX thread
	 * (whether IRQ-woken or polling) has time to process packets and explicitly restart RX
	 * via dw3000_restart_rx_locked(). Single-buffer with explicit restart is simpler and
	 * sufficient for our thread-based model, avoiding state complexity and buffer tracking.
	 */
	dwt_setdblrxbuffmode(DBL_BUF_STATE_DIS, DBL_BUF_MODE_MAN);
	ret = dw3000_apply_phy_config_locked(data);
	if (ret != 0) {
		k_mutex_unlock(&data->lock);
		LOG_ERR("Failed to configure DW3000 PHY for IEEE802154");
		return ret;
	}

	ret = dwt_setchannel(DWT_CH5);
	if (ret != DWT_SUCCESS) {
		k_mutex_unlock(&data->lock);
		LOG_ERR("dwt_setchannel failed during init: ch=%u ret=%d", data->channel, ret);
		return -EIO;
	}
	(void)dw3000_apply_filter_hw(data);
	dwt_setrxtimeout(0);
	dwt_setpreambledetecttimeout(0);
	dw3000_clear_status_all();
	dwt_forcetrxoff();
	k_mutex_unlock(&data->lock);

#if defined(CONFIG_NET_CONFIG_IEEE802154_CHANNEL)
	LOG_WRN("DW3000 IEEE802154 init channel=%u (kconfig=%d)",
		data->channel,
		CONFIG_NET_CONFIG_IEEE802154_CHANNEL);
#else
	LOG_WRN("DW3000 IEEE802154 init channel=%u (kconfig undefined)", data->channel);
#endif

#if defined(CONFIG_IEEE802154_DW3000_RX_POLLING_MODE)
	LOG_INF("DW3000 path configured for polling RX (double-buffer disabled)");
#else
	if (dw3000_hw_init_interrupt() != 0) {
		LOG_ERR("Failed to initialize DW3000 IRQ GPIO");
		return -EIO;
	}
	dw3000_irq_dev = dev;
	dw3000_hw_set_interrupt_handler(dw3000_irq_cb);
	dw3000_hw_interrupt_enable();
	LOG_INF("DW3000 path configured for IRQ-driven RX (double-buffer disabled)");
#endif

	k_thread_create(&data->rx_thread,
			cfg->rx_stack,
			cfg->rx_stack_size,
			dw3000_rx_thread_fn,
			(void *)dev,
			NULL,
			NULL,
			8,
			0,
			K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "dw3000_rx");

	LOG_INF("DW3000 IEEE802154 driver initialized");
	return 0;
}

/* Device tree configuration */
#define DT_DRV_COMPAT decawave_dw3000

#define DWT_PSDU_LENGTH (127 - DW3000_FCS_LEN)

#define DW3000_INIT(n)                                                                        \
	K_KERNEL_STACK_DEFINE(dw3000_rx_stack_##n, CONFIG_IEEE802154_DW3000_IRQ_THREAD_STACK_SIZE); \
	static struct dw3000_data dw3000_data_##n;                                                  \
	static const struct dw3000_config dw3000_config_##n = {                                     \
		.rx_stack = dw3000_rx_stack_##n,                                                          \
		.rx_stack_size = K_KERNEL_STACK_SIZEOF(dw3000_rx_stack_##n),                              \
	};                                                                                          \
	NET_DEVICE_DT_INST_DEFINE(n,                                                                \
		dw3000_init,                                                                              \
		NULL,                                                                                     \
		&dw3000_data_##n,                                                                         \
		&dw3000_config_##n,                                                                       \
		CONFIG_IEEE802154_DW3000_INIT_PRIO,                                                       \
		&dw3000_radio_api,                                                                        \
		IEEE802154_L2,                                                                            \
		NET_L2_GET_CTX_TYPE(IEEE802154_L2),                                                       \
		DWT_PSDU_LENGTH)

DT_INST_FOREACH_STATUS_OKAY(DW3000_INIT)
