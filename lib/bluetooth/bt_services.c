/**
 * @file bt_services.c
 * @brief Bluetooth GATT service registry for extensible service management
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "bt_services.h"

LOG_MODULE_REGISTER(synchrofly_bt_services, CONFIG_LOG_DEFAULT_LEVEL);

/* Maximum number of custom GATT services that can be registered */
#define MAX_SERVICES 8

/* Service registry storage */
static const struct bt_gatt_service *registered_services[MAX_SERVICES];
static size_t num_services = 0;

/* Mutex to protect service registration */
K_MUTEX_DEFINE(service_mutex);

int synchrofly_bt_register_service(const struct bt_gatt_service *svc)
{
    if (!svc) {
        LOG_ERR("Attempted to register NULL service");
        return -EINVAL;
    }

    k_mutex_lock(&service_mutex, K_FOREVER);

    if (num_services >= MAX_SERVICES) {
        LOG_ERR("Service registry full (max %d services)", MAX_SERVICES);
        k_mutex_unlock(&service_mutex);
        return -ENOMEM;
    }

    registered_services[num_services++] = svc;
    LOG_INF("Registered service %zu/%d", num_services, MAX_SERVICES);

    k_mutex_unlock(&service_mutex);
    return 0;
}

int synchrofly_bt_services_init(void)
{
    int err;

    k_mutex_lock(&service_mutex, K_FOREVER);

    for (size_t i = 0; i < num_services; i++) {
        err = bt_gatt_service_register((struct bt_gatt_service *)registered_services[i]);
        if (err) {
            LOG_ERR("Failed to register service %zu (err %d)", i, err);
            k_mutex_unlock(&service_mutex);
            return err;
        }
        LOG_INF("Service %zu registered with GATT stack", i);
    }

    LOG_INF("All %zu services registered successfully", num_services);
    k_mutex_unlock(&service_mutex);
    return 0;
}
