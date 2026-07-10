#include <zephyr/init.h>
#include <zephyr/device.h>

#include <app/lib/tracing/systemview.h>

#ifdef CONFIG_SEGGER_SYSTEMVIEW
#include <SEGGER_SYSVIEW.h>
#endif

void synchrofly_sysview_start(void)
{
#ifdef CONFIG_SEGGER_SYSTEMVIEW
	SEGGER_SYSVIEW_Start();
        printk("Starting Segger SystemView!\n");
#endif
}

void synchrofly_sysview_stop(void)
{
#ifdef CONFIG_SEGGER_SYSTEMVIEW
	SEGGER_SYSVIEW_Stop();
#endif
}

#ifdef CONFIG_SYNCHROFLY_TRACING_SYSTEMVIEW_AUTOSTART
static int synchrofly_sysview_autostart(const struct device *unused)
{
	ARG_UNUSED(unused);

	synchrofly_sysview_start();
	return 0;
}

SYS_INIT(synchrofly_sysview_autostart, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);
#endif
