/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * @author Patrick Rathje git@patrickrathje.de
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_USER_CAU_H_
#define ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_USER_CAU_H_

#include <zephyr/device.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>

/* All protocol structures and types moved to uwb_driver_api.h for hardware abstraction */

/* DW1000-specific legacy functions that should eventually be deprecated */
int dwt_set_channel(const struct device *dev, uint16_t channel);
uint64_t dwt_system_ts(const struct device *dev);

/* DW1000 range bias correction function */
#ifdef CONFIG_DW1000_RANGE_BIAS_CORRECTION
double dwt_getrangebias(uint8_t chan, float range, uint8_t prf);
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_USER_CAU_H_ */
