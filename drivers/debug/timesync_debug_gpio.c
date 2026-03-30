#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <app/drivers/debug/timesync_debug_gpio.h>

LOG_MODULE_REGISTER(timesync_debug_gpio, CONFIG_LOG_DEFAULT_LEVEL);

#define TIMESYNC_DEBUG_GPIO_NODE DT_NODELABEL(timesync_debug)

#if DT_NODE_HAS_STATUS(TIMESYNC_DEBUG_GPIO_NODE, okay)
#define TIMESYNC_DEBUG_GPIO_ENABLED 1

static const struct gpio_dt_spec debug_gpio = GPIO_DT_SPEC_GET(TIMESYNC_DEBUG_GPIO_NODE, gpios);
static bool initialized = false;

static int timesync_debug_gpio_init(void)
{
	int ret;

	if (!gpio_is_ready_dt(&debug_gpio)) {
		LOG_ERR("Debug GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&debug_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure debug GPIO: %d", ret);
		return ret;
	}

	initialized = true;
	LOG_INF("Time synchronization debug GPIO initialized on pin %d", debug_gpio.pin);
	return 0;
}

void timesync_debug_assert(void)
{
	if (initialized) {
		gpio_pin_set_dt(&debug_gpio, 1);
	}
}

void timesync_debug_deassert(void)
{
	if (initialized) {
		gpio_pin_set_dt(&debug_gpio, 0);
	}
}

void timesync_debug_pulse(void)
{
	if (initialized) {
		gpio_pin_set_dt(&debug_gpio, 1);
		gpio_pin_set_dt(&debug_gpio, 0);
	}
}

SYS_INIT(timesync_debug_gpio_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

#else
/* No GPIO configured - provide no-op implementations */

void timesync_debug_assert(void)
{
	/* No-op */
}

void timesync_debug_deassert(void)
{
	/* No-op */
}

void timesync_debug_pulse(void)
{
	/* No-op */
}

#endif /* DT_NODE_HAS_STATUS */
