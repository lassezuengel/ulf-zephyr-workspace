#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "deca_device_api.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"

LOG_MODULE_REGISTER(dw3000, CONFIG_IEEE802154_DW3000_LOG_LEVEL);

#define DW_INST DT_INST(0, decawave_dw3000)

static struct gpio_callback gpio_cb;
static dw3000_irq_handler_t custom_irq_handler = NULL;

struct dw3000_config {
	struct gpio_dt_spec gpio_irq;
	struct gpio_dt_spec gpio_reset;
	struct gpio_dt_spec gpio_wakeup;
	struct gpio_dt_spec gpio_spi_pol;
	struct gpio_dt_spec gpio_spi_pha;
};

static const struct dw3000_config conf = {
	.gpio_irq = GPIO_DT_SPEC_GET_OR(DW_INST, int_gpios, {0}),
	.gpio_reset = GPIO_DT_SPEC_GET_OR(DW_INST, reset_gpios, {0}),
	.gpio_wakeup = GPIO_DT_SPEC_GET_OR(DW_INST, wakeup_gpios, {0}),
	.gpio_spi_pol = GPIO_DT_SPEC_GET_OR(DW_INST, spi_pol_gpios, {0}),
	.gpio_spi_pha = GPIO_DT_SPEC_GET_OR(DW_INST, spi_pha_gpios, {0}),
};

int dw3000_hw_init(void)
{
	printk("[DEBUG] dw3000_hw_init: Entry point\n");
	/* Reset */
	if (conf.gpio_reset.port) {
		printk("[DEBUG] dw3000_hw_init: Configuring RESET pin\n");
		gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
		LOG_INF("RESET on %s pin %d", conf.gpio_reset.port->name,
				conf.gpio_reset.pin);
		printk("[DEBUG] dw3000_hw_init: RESET pin configured\n");
	} else {
		printk("[DEBUG] dw3000_hw_init: No RESET pin configured\n");
	}

	/* Wakeup (optional) */
	if (conf.gpio_wakeup.port) {
		gpio_pin_configure_dt(&conf.gpio_wakeup, GPIO_OUTPUT_ACTIVE);
		LOG_INF("WAKEUP on %s pin %d", conf.gpio_wakeup.port->name,
				conf.gpio_wakeup.pin);
	}

	/* SPI Polarity (optional) */
	if (conf.gpio_spi_pol.port) {
		gpio_pin_configure_dt(&conf.gpio_spi_pol, GPIO_OUTPUT_INACTIVE);
		LOG_INF("SPI_POL on %s pin %d", conf.gpio_spi_pol.port->name,
				conf.gpio_spi_pol.pin);
	}

	/* SPI Phase (optional) */
	if (conf.gpio_spi_pha.port) {
		printk("[DEBUG] dw3000_hw_init: Configuring SPI_PHA pin\n");
		gpio_pin_configure_dt(&conf.gpio_spi_pha, GPIO_OUTPUT_INACTIVE);
		LOG_INF("SPI_PHA on %s pin %d", conf.gpio_spi_pha.port->name,
				conf.gpio_spi_pha.pin);
	}

	printk("[DEBUG] dw3000_hw_init: Completed successfully\n");
	return 0;
}

static void dw3000_hw_isr(const struct device* dev, struct gpio_callback* cb,
						  uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	// Call custom handler if set
	if (custom_irq_handler != NULL) {
		// Call the custom interrupt handler directly
		custom_irq_handler();
	} else {
		// Fallback: just log
		LOG_DBG("DW3000 interrupt triggered - no custom handler set");
	}
}

int dw3000_hw_init_interrupt(void)
{
	printk("[DEBUG] dw3000_hw_init_interrupt: Entry point\n");
	if (conf.gpio_irq.port) {
		printk("[DEBUG] dw3000_hw_init_interrupt: Setting up IRQ pin\n");

		gpio_pin_configure_dt(&conf.gpio_irq, GPIO_INPUT);
		gpio_init_callback(&gpio_cb, dw3000_hw_isr, BIT(conf.gpio_irq.pin));
		gpio_add_callback(conf.gpio_irq.port, &gpio_cb);
		// Don't enable interrupt yet - wait until custom handler is set
		gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_DISABLE);

		LOG_INF("IRQ on %s pin %d (disabled until handler set)", conf.gpio_irq.port->name,
				conf.gpio_irq.pin);
		printk("[DEBUG] dw3000_hw_init_interrupt: IRQ setup complete (disabled)\n");
		return 0;
	} else {
		printk("[DEBUG] dw3000_hw_init_interrupt: No IRQ pin configured\n");
		LOG_ERR("IRQ pin not configured");
		return -ENOENT;
	}
}

void dw3000_hw_interrupt_enable(void)
{
	if (conf.gpio_irq.port) {
		gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_EDGE_TO_ACTIVE);
	}
}

void dw3000_hw_interrupt_disable(void)
{
	if (conf.gpio_irq.port) {
		gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_DISABLE);
	}
}

bool dw3000_hw_interrupt_is_enabled(void)
{
	return true; // TODO
}

void dw3000_hw_fini(void)
{
	// TODO
	if (conf.gpio_irq.port) {
		gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_DISABLE);
		gpio_pin_configure_dt(&conf.gpio_irq, GPIO_DISCONNECTED);
	}
	if (conf.gpio_reset.port) {
		gpio_pin_configure_dt(&conf.gpio_reset, GPIO_DISCONNECTED);
	}
	if (conf.gpio_wakeup.port) {
		gpio_pin_configure_dt(&conf.gpio_wakeup, GPIO_DISCONNECTED);
	}

	// SPI cleanup now handled by dw3000_spi.c
}

void dw3000_hw_reset()
{
	printk("[DEBUG] dw3000_hw_reset: Entry point\n");
	if (!conf.gpio_reset.port) {
		printk("[DEBUG] dw3000_hw_reset: No reset pin configured\n");
		LOG_ERR("No HW reset configured");
		return;
	}

	printk("[DEBUG] dw3000_hw_reset: Performing reset sequence\n");
	gpio_pin_configure_dt(&conf.gpio_reset, GPIO_OUTPUT_ACTIVE);
	k_msleep(1); // 10 us?
	gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
	k_msleep(2);
	printk("[DEBUG] dw3000_hw_reset: Reset sequence completed\n");
}

/** wakeup either using the WAKEUP pin or SPI CS */
void dw3000_hw_wakeup(void)
{
	printk("[DEBUG] dw3000_hw_wakeup: Entry point\n");
	if (conf.gpio_wakeup.port) {
		/* Use WAKEUP pin if available */
		printk("[DEBUG] dw3000_hw_wakeup: Using WAKEUP pin\n");
		LOG_INF("WAKEUP PIN");
		gpio_pin_set_dt(&conf.gpio_wakeup, 1);
		k_msleep(1);
		gpio_pin_set_dt(&conf.gpio_wakeup, 0);
		printk("[DEBUG] dw3000_hw_wakeup: WAKEUP pin sequence completed\n");

	} else {
		/* Use SPI CS pin */
		printk("[DEBUG] dw3000_hw_wakeup: Using SPI CS wakeup\n");
		LOG_INF("WAKEUP CS");
		dw3000_spi_wakeup();
		printk("[DEBUG] dw3000_hw_wakeup: SPI CS wakeup completed\n");
	}
}

/** set WAKEUP pin low if available */
void dw3000_hw_wakeup_pin_low(void)
{
	if (conf.gpio_wakeup.port) {
		gpio_pin_set_dt(&conf.gpio_wakeup, 0);
	}
}

void dw3000_hw_set_interrupt_handler(dw3000_irq_handler_t handler)
{
	custom_irq_handler = handler;
	LOG_DBG("Custom interrupt handler set");
}

void dw3000_hw_clear_interrupt_handler(void)
{
	custom_irq_handler = NULL;
	LOG_DBG("Custom interrupt handler cleared");
}
