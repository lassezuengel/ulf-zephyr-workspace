#include <app/lib/system/hw.h>

const struct device *ieee802154_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));

bool ieee802154_radio_is_ready(void) {
    return device_is_ready(ieee802154_dev);
}
