// my_device.c

#ifndef SYNCHROFLY_HW
#define SYNCHROFLY_HW

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

extern const struct device *ieee802154_dev;

bool ieee802154_radio_is_ready(void);

#endif
