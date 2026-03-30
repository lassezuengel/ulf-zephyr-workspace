#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(foo_driver, CONFIG_LOG_DEFAULT_LEVEL);

#ifdef CONFIG_FOO_DRIVER_ENABLE
static bool initialized = false;

static int foo_driver_init(void)
{
	LOG_INF("Foo driver initialized with value %d", CONFIG_FOO_DRIVER_VALUE);
  initialized = true;

	return 0;
}

SYS_INIT(foo_driver_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
#endif /* CONFIG_FOO_DRIVER_ENABLE */