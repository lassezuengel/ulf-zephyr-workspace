/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include "dw3000.h"

LOG_MODULE_REGISTER(ieee802154_dw3000_minimal, CONFIG_IEEE802154_DRIVER_LOG_LEVEL);

/* Minimal IEEE 802.15.4 Radio Driver API Implementation */
static enum ieee802154_hw_caps dw3000_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IEEE802154_HW_FCS |
	       IEEE802154_HW_FILTER |
	       IEEE802154_HW_TX_RX_ACK |
	       IEEE802154_HW_RX_TX_ACK;
}

static int dw3000_cca(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0; /* Channel always clear for ranging */
}

static int dw3000_set_channel(const struct device *dev, uint16_t channel)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
	return 0; /* Handled by UWB abstraction layer */
}

static int dw3000_filter(const struct device *dev,
			 bool set,
			 enum ieee802154_filter_type type,
			 const struct ieee802154_filter *filter)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(set);
	ARG_UNUSED(type);
	ARG_UNUSED(filter);
	return 0; /* Handled by UWB abstraction layer */
}

static int dw3000_set_txpower(const struct device *dev, int16_t dbm)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dbm);
	return 0; /* Handled by UWB abstraction layer */
}

static int dw3000_tx(const struct device *dev,
		     enum ieee802154_tx_mode mode,
		     struct net_pkt *pkt,
		     struct net_buf *frag)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(mode);
	ARG_UNUSED(pkt);
	ARG_UNUSED(frag);
	return -ENOTSUP; /* Handled by UWB abstraction layer */
}

static int dw3000_start(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0; /* Handled by UWB abstraction layer */
}

static int dw3000_stop(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0; /* Handled by UWB abstraction layer */
}

static int dw3000_configure(const struct device *dev,
			    enum ieee802154_config_type type,
			    const struct ieee802154_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(type);
	ARG_UNUSED(config);
	return 0; /* Handled by UWB abstraction layer */
}

static const struct ieee802154_radio_api dw3000_radio_api = {
	.iface_api.init = NULL,
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
	int ret;

	LOG_INF("DW3000 minimal IEEE 802.15.4 driver initialized");

	/* Initialize UWB driver abstraction layer */
	ret = uwb_driver_dw3000_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize UWB driver: %d", ret);
		return ret;
	}

	return 0;
}

/* Device tree configuration */
#define DT_DRV_COMPAT decawave_dw3000

#define DW3000_INIT(n)						\
	DEVICE_DT_INST_DEFINE(n,				\
			      dw3000_init,			\
			      NULL,				\
			      NULL,				\
			      NULL,				\
			      POST_KERNEL,			\
			      CONFIG_IEEE802154_DW3000_INIT_PRIO, \
			      &dw3000_radio_api);

DT_INST_FOREACH_STATUS_OKAY(DW3000_INIT)