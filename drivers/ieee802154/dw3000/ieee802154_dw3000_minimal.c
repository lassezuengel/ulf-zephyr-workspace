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

#include "dw3000.h"

LOG_MODULE_REGISTER(ieee802154_dw3000_minimal, CONFIG_IEEE802154_DRIVER_LOG_LEVEL);

#define DW3000_FCS_LEN 2U
#define DW3000_MAX_PHY_PACKET_SIZE 127U

struct dw3000_data {
	struct net_if *iface;
	struct k_mutex lock;
	struct k_thread rx_thread;
	atomic_t started;
	uint8_t mac_addr[8];
	uint16_t pan_id;
	uint16_t short_addr;
	uint16_t channel;
	bool promiscuous;
};

struct dw3000_config {
	k_thread_stack_t *rx_stack;
	size_t rx_stack_size;
};

static void dw3000_iface_api_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct dw3000_data *data = dev->data;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_IEEE802154);
	data->iface = iface;
	ieee802154_init(iface);
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

static inline void dw3000_restart_rx_locked(void)
{
	dwt_setrxtimeout(0);
	(void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static struct net_pkt *dw3000_rx_read_pkt_locked(const struct device *dev)
{
	struct dw3000_data *data = dev->data;
	struct net_pkt *pkt;
	uint8_t rng = 0;
	uint16_t phy_len = dwt_getframelength(&rng);
	uint16_t pkt_len = phy_len;

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
		LOG_DBG("No net_pkt buffer for RX len=%u", pkt_len);
		dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
		dw3000_restart_rx_locked();
		return NULL;
	}

	dwt_readrxdata(pkt->buffer->data, pkt_len, 0);
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
			k_usleep(5000);
			continue;
		}

		k_mutex_lock(&data->lock, K_FOREVER);
		status = dwt_readsysstatuslo();

		if (status & DWT_INT_RXFCG_BIT_MASK) {
			pkt = dw3000_rx_read_pkt_locked(dev);
		} else if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
			dwt_writesysstatuslo(status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR));
			dw3000_restart_rx_locked();
		}

		k_mutex_unlock(&data->lock);

		if (!pkt) {
			k_usleep(1000);
			continue;
		}

		if (ieee802154_handle_ack(data->iface, pkt) != NET_OK &&
		    net_recv_data(data->iface, pkt) != NET_OK) {
			net_pkt_unref(pkt);
		}
	}
}

static enum ieee802154_hw_caps dw3000_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IEEE802154_HW_FCS | IEEE802154_HW_FILTER;
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
	ret = dwt_setchannel(dw_ch);
	if (ret == DWT_SUCCESS) {
		data->channel = channel;
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
		break;

	case IEEE802154_FILTER_TYPE_SHORT_ADDR:
		data->short_addr = set ? filter->short_addr : 0xffffU;
		break;

	case IEEE802154_FILTER_TYPE_PAN_ID:
		data->pan_id = set ? filter->pan_id : 0xffffU;
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

	if (!frag || frag->len == 0U) {
		return -EINVAL;
	}

	if (!atomic_get(&data->started)) {
		return -ENETDOWN;
	}

	if ((frag->len + DW3000_FCS_LEN) > DW3000_MAX_PHY_PACKET_SIZE) {
		return -EMSGSIZE;
	}

	if (mode == IEEE802154_TX_MODE_CCA) {
		start_mode = DWT_START_TX_CCA;
	} else if (mode != IEEE802154_TX_MODE_DIRECT) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	dwt_forcetrxoff();
	dwt_writesysstatuslo(DWT_INT_ALL_LO);

	ret = dwt_writetxdata((uint16_t)frag->len, frag->data, 0);
	if (ret != DWT_SUCCESS) {
		k_mutex_unlock(&data->lock);
		return -EIO;
	}

	dwt_writetxfctrl((uint16_t)frag->len + DW3000_FCS_LEN, 0, 0);
	ret = dwt_starttx(start_mode);
	if (ret != DWT_SUCCESS) {
		dw3000_restart_rx_locked();
		k_mutex_unlock(&data->lock);
		return -EIO;
	}

	timeout_end = k_uptime_get() + 20;
	while (k_uptime_get() < timeout_end) {
		uint32_t status = dwt_readsysstatuslo();

		if (status & DWT_INT_TXFRS_BIT_MASK) {
			dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
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
	k_mutex_unlock(&data->lock);

	return -EIO;
}

static int dw3000_start(const struct device *dev)
{
	struct dw3000_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	atomic_set(&data->started, 1);
	dwt_forcetrxoff();
	dwt_writesysstatuslo(DWT_INT_ALL_LO);
	dw3000_restart_rx_locked();
	k_mutex_unlock(&data->lock);

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
	atomic_set(&data->started, 0);
	dw3000_generate_mac(data->mac_addr);
	data->pan_id = 0xffffU;
	data->short_addr = 0xffffU;
	data->channel = 5U;
	data->promiscuous = false;

	k_mutex_lock(&data->lock, K_FOREVER);
	(void)dwt_setchannel(DWT_CH5);
	(void)dw3000_apply_filter_hw(data);
	dwt_setrxtimeout(0);
	dwt_forcetrxoff();
	k_mutex_unlock(&data->lock);

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
