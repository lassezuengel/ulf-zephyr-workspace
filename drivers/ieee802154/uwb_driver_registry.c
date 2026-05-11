#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uwb_driver_registry, LOG_LEVEL_INF);

// Registry to store driver implementations for each device
static const uwb_driver_t *device_drivers[32]; // Support up to 32 devices
static const struct device *registered_devices[32];
static int registry_count = 0;

int uwb_driver_register(const struct device *dev, const uwb_driver_t *driver)
{
    if (!dev || !driver) {
        LOG_ERR("Invalid device or driver pointer");
        return -EINVAL;
    }

    if (registry_count >= ARRAY_SIZE(device_drivers)) {
        LOG_ERR("Driver registry full");
        return -ENOMEM;
    }

    // Check if device is already registered
    for (int i = 0; i < registry_count; i++) {
        if (registered_devices[i] == dev) {
            LOG_WRN("Device %s already has a registered driver, replacing", dev->name);
            device_drivers[i] = driver;
            return 0;
        }
    }

    // Register new device
    registered_devices[registry_count] = dev;
    device_drivers[registry_count] = driver;
    registry_count++;

    LOG_INF("Registered UWB driver '%s' for device '%s'", driver->driver_name, dev->name);
    return 0;
}

const uwb_driver_t *uwb_driver_get(const struct device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return NULL;
    }

    // Find driver for this device
    for (int i = 0; i < registry_count; i++) {
        if (registered_devices[i] == dev) {
            return device_drivers[i];
        }
    }

    LOG_ERR("No UWB driver registered for device '%s'", dev->name);
    return NULL;
}

