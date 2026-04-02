/*
 * Copyright (c) 2024 SynchroFly Project
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

LOG_MODULE_REGISTER(ieee802154_dw3000_minimal, LOG_LEVEL_DBG);

#define DW3000_FCS_LEN 2U
#define DW3000_MAX_PHY_PACKET_SIZE 127U

struct dw3000_data {
	struct net_if *iface;
	const uwb_driver_t *uwb;
	struct k_mutex lock;
	struct k_thread rx_thread;
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
};

struct dw3000_config {
	k_thread_stack_t *rx_stack;
	size_t rx_stack_size;
};

static int dw3000_start(const struct device *dev);

static int dw3000_apply_phy_config_locked(struct dw3000_data *data)
{
	dwt_config_t phy_cfg = {
		.chan = (data->channel == 9U) ? 9U : 5U,
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
   * by zephyr via the ieee802154 api.
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

static struct net_pkt *dw3000_rx_read_pkt_locked(const struct device *dev)
{
	struct dw3000_data *data = dev->data;
	struct net_pkt *pkt;
	uint8_t rng = 0;
	uint16_t phy_len = 0;
	uint16_t pkt_len;

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
		return NULL;
	}

#if !defined(CONFIG_IEEE802154_RAW_MODE)
	pkt_len -= DW3000_FCS_LEN;
#endif

	pkt = net_pkt_rx_alloc_with_buffer(data->iface, pkt_len, AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		data->rx_nobuf_cnt++;
		if ((data->rx_nobuf_cnt % 32U) == 0U) {
			LOG_WRN("RX dropped (no net_pkt): cnt=%u len=%u", data->rx_nobuf_cnt, pkt_len);
		}
		dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
		dw3000_restart_rx_locked();
		return NULL;
	}

	if (data->uwb && data->uwb->read_rx_frame) {
		data->uwb->read_rx_frame(dev, pkt->buffer->data, pkt_len, 0);
	} else {
		dwt_readrxdata(pkt->buffer->data, pkt_len, 0);
	}
	net_buf_add(pkt->buffer, pkt_len);

	dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
	dw3000_restart_rx_locked();

	return pkt;
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

		k_mutex_lock(&data->lock, K_FOREVER);
		status = dwt_readsysstatuslo();

		if (status & DWT_INT_RXFCG_BIT_MASK) {
			pkt = dw3000_rx_read_pkt_locked(dev);
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

		k_mutex_unlock(&data->lock);

		if (!pkt) {
			k_usleep(1000);
			continue;
		}

		if (ieee802154_handle_ack(data->iface, pkt) == NET_OK) {
			net_pkt_unref(pkt);
			continue;
		}

		if (net_recv_data(data->iface, pkt) != NET_OK) {
			net_pkt_unref(pkt);
		} else {
			data->rx_ok_cnt++;
			if ((data->rx_ok_cnt % 16U) == 0U) {
				LOG_INF("RX packets=%u", data->rx_ok_cnt);
			}
		}
	}
}

static enum ieee802154_hw_caps dw3000_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IEEE802154_HW_FCS | IEEE802154_HW_FILTER | IEEE802154_HW_TXTIME;
}

static int dw3000_cca(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int dw3000_set_channel(const struct device *dev, uint16_t channel)
{
	struct dw3000_data *data = dev->data;
	dwt_pll_ch_type_e dw_ch;
	int ret;

	if (channel == 5U) {
		dw_ch = DWT_CH5;
	} else if (channel == 9U) {
		dw_ch = DWT_CH9;
	} else {
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

	ret = dwt_setchannel(dw_ch);
	if (ret == DWT_SUCCESS) {
		LOG_INF("Channel set to %u", channel);
	}
	k_mutex_unlock(&data->lock);

	return (ret == DWT_SUCCESS) ? 0 : -EIO;
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

  // TODO: Support more TX modes, including CCA/CSMA and delayed TX. For now, treat all modes as direct TX.
	if (mode == IEEE802154_TX_MODE_DIRECT || mode == IEEE802154_TX_MODE_TXTIME ||
	    mode == IEEE802154_TX_MODE_CCA || mode == IEEE802154_TX_MODE_CSMA_CA ||
	    mode == IEEE802154_TX_MODE_TXTIME_CCA) {
		start_mode = DWT_START_TX_IMMEDIATE;
		if (mode != IEEE802154_TX_MODE_DIRECT && mode != IEEE802154_TX_MODE_TXTIME) {
			LOG_DBG("TX mode=%d requested CCA/CSMA, forcing IMMEDIATE for DW3000 minimal path", mode);
		}
	} else {
		LOG_WRN("Unsupported TX mode=%d", mode);
		return -ENOTSUP;
	}

	data->tx_attempt_cnt++;
	if (data->tx_attempt_cnt <= 8U || (data->tx_attempt_cnt % 64U) == 0U) {
		LOG_INF("TX attempt=%u len=%u mode=%d", data->tx_attempt_cnt, frag->len, mode);
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->uwb && data->uwb->force_trx_off) {
		data->uwb->force_trx_off(dev);
	} else {
		dwt_forcetrxoff();
	}
	dw3000_clear_status_all();

	if (data->uwb && data->uwb->setup_tx_frame && data->uwb->start_tx) {
		data->uwb->setup_tx_frame(dev, frag->data, (uint16_t)frag->len);
		ret = data->uwb->start_tx(dev, 0);
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
		data->tx_err_cnt++;
		LOG_WRN("TX start failed ret=%d mode=%d lo=0x%08x hi=0x%08x tx_err=%u",
			ret, mode, status_lo, status_hi, data->tx_err_cnt);
		dw3000_restart_rx_locked();
		k_mutex_unlock(&data->lock);
		return -EIO;
	}

	timeout_end = k_uptime_get() + 20;
	while (k_uptime_get() < timeout_end) {
		uint32_t status = dwt_readsysstatuslo();

		if (status & DWT_INT_TXFRS_BIT_MASK) {
			dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
			data->tx_ok_cnt++;
			if ((data->tx_ok_cnt % 16U) == 0U) {
				LOG_INF("TX packets=%u", data->tx_ok_cnt);
			}
			dw3000_restart_rx_locked();
			k_mutex_unlock(&data->lock);
			return 0;
		}

		if (status & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)) {
			dwt_writesysstatuslo(status & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO));
		}

		k_usleep(1000);
	}

	dw3000_restart_rx_locked();
	data->tx_err_cnt++;
	LOG_WRN("TX timeout lo=0x%08x hi=0x%08x tx_err=%u",
		dwt_readsysstatuslo(), dwt_readsysstatushi(), data->tx_err_cnt);
	k_mutex_unlock(&data->lock);

	return -EIO;
}

static int dw3000_start(const struct device *dev)
{
	struct dw3000_data *data = dev->data;

	LOG_DBG("dw3000_start: Entry");
	k_mutex_lock(&data->lock, K_FOREVER);
	LOG_DBG("dw3000_start: Mutex acquired");

	atomic_set(&data->started, 1);
	LOG_DBG("dw3000_start: Started flag set");

	dwt_forcetrxoff();
	LOG_DBG("dw3000_start: dwt_forcetrxoff() completed");

	dw3000_clear_status_all();
	LOG_DBG("dw3000_start: clear_status_all() completed");

	dw3000_restart_rx_locked();
	LOG_DBG("dw3000_start: dw3000_restart_rx_locked() completed");

	k_mutex_unlock(&data->lock);
	LOG_DBG("dw3000_start: Mutex released");

	LOG_INF("Radio start: channel=%u pan=0x%04x short=0x%04x", data->channel, data->pan_id,
		data->short_addr);
	LOG_DBG("dw3000_start: Final log completed");

	return 0;
}

static int dw3000_stop(const struct device *dev)
{
	struct dw3000_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	atomic_set(&data->started, 0);
	dwt_forcetrxoff();
	k_mutex_unlock(&data->lock);

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
	const struct ieee802154_phy_channel_range phy_channel_range[2];
	const struct ieee802154_phy_supported_channels phy_supported_channels;
} dw3000_drv_attr = {
	.phy_channel_range = {
		{ .from_channel = 5, .to_channel = 5 },
		{ .from_channel = 9, .to_channel = 9 },
	},
	.phy_supported_channels = {
		.ranges = dw3000_drv_attr.phy_channel_range,
		.num_ranges = 2U,
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
	 * For the IEEE802154 net path, we poll status in a dedicated RX thread.
	 */
	dw3000_hw_interrupt_disable();
	dw3000_hw_clear_interrupt_handler();

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
	data->promiscuous = false;
	data->rx_ok_cnt = 0U;
	data->rx_err_cnt = 0U;
	data->tx_ok_cnt = 0U;
	data->tx_err_cnt = 0U;
	data->tx_attempt_cnt = 0U;
	data->tx_entry_cnt = 0U;
	data->rx_nobuf_cnt = 0U;
	data->rx_wait_logged = false;

	k_mutex_lock(&data->lock, K_FOREVER);
	/*
	 * The UWB core enables double-buffer RX for interrupt-driven operation.
	 * This minimal net driver uses polling, so force single-buffer mode.
	 */
	dwt_setdblrxbuffmode(DBL_BUF_STATE_DIS, DBL_BUF_MODE_MAN);
	ret = dw3000_apply_phy_config_locked(data);
	if (ret != 0) {
		k_mutex_unlock(&data->lock);
		LOG_ERR("Failed to configure DW3000 PHY for IEEE802154");
		return ret;
	}
	(void)dwt_setchannel(DWT_CH5);
	(void)dw3000_apply_filter_hw(data);
	dwt_setrxtimeout(0);
	dwt_setpreambledetecttimeout(0);
	dw3000_clear_status_all();
	dwt_forcetrxoff();
	k_mutex_unlock(&data->lock);

	LOG_INF("DW3000 minimal path configured for polling RX (double-buffer disabled)");

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

#define DW3000_INIT(n)                                                                  \
	K_KERNEL_STACK_DEFINE(dw3000_rx_stack_##n, CONFIG_IEEE802154_DW3000_IRQ_THREAD_STACK_SIZE); \
	static struct dw3000_data dw3000_data_##n;                                        \
	static const struct dw3000_config dw3000_config_##n = {                           \
		.rx_stack = dw3000_rx_stack_##n,                                            \
		.rx_stack_size = K_KERNEL_STACK_SIZEOF(dw3000_rx_stack_##n),                \
	};                                                                                \
	NET_DEVICE_DT_INST_DEFINE(n,                                                     \
		dw3000_init,                                                              \
		NULL,                                                                     \
		&dw3000_data_##n,                                                        \
		&dw3000_config_##n,                                                      \
		CONFIG_IEEE802154_DW3000_INIT_PRIO,                                      \
		&dw3000_radio_api,                                                       \
		IEEE802154_L2,                                                           \
		NET_L2_GET_CTX_TYPE(IEEE802154_L2),                                      \
		DWT_PSDU_LENGTH)

DT_INST_FOREACH_STATUS_OKAY(DW3000_INIT)
