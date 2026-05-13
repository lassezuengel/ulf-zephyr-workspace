#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "deca_device_api.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"

LOG_MODULE_REGISTER(dw3000, CONFIG_IEEE802154_DW3000_LOG_LEVEL);

#define DW_INST DT_INST(0, decawave_dw3000)
#define DW3000_IRQ_HANDLER_SLOTS 2

static struct gpio_callback gpio_cb;
static dw3000_irq_handler_t irq_handlers[DW3000_IRQ_HANDLER_SLOTS];
static bool irq_enabled;
static int irq_disable_nesting = 0;  /* Track nested IRQ disable/enable calls from decamutexon/off */

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
	LOG_DBG("dw3000_hw_init: entry");
	/* Reset */
	if (conf.gpio_reset.port) {
		LOG_DBG("dw3000_hw_init: configuring RESET pin");
		gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
		LOG_INF("RESET on %s pin %d", conf.gpio_reset.port->name,
				conf.gpio_reset.pin);
		LOG_DBG("dw3000_hw_init: RESET pin configured");
	} else {
		LOG_DBG("dw3000_hw_init: no RESET pin configured");
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
		LOG_DBG("dw3000_hw_init: configuring SPI_PHA pin");
		gpio_pin_configure_dt(&conf.gpio_spi_pha, GPIO_OUTPUT_INACTIVE);
		LOG_INF("SPI_PHA on %s pin %d", conf.gpio_spi_pha.port->name,
				conf.gpio_spi_pha.pin);
	}

	LOG_DBG("dw3000_hw_init: done");
	return 0;
}

static void dw3000_hw_isr(const struct device* dev, struct gpio_callback* cb,
						  uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (!irq_enabled) {
		LOG_WRN("DW3000 interrupt arrived while IRQ line is disabled");
	}

	if (irq_handlers[0] != NULL || irq_handlers[1] != NULL) {
		LOG_DBG("DW3000 interrupt dispatched to shared handlers: slot0=%p slot1=%p",
			(void *)irq_handlers[0], (void *)irq_handlers[1]);
		if (irq_handlers[0] != NULL) {
			irq_handlers[0]();
		}
		if (irq_handlers[1] != NULL) {
			irq_handlers[1]();
		}
	} else {
		LOG_DBG("DW3000 interrupt triggered with no registered handlers");
	}
}

int dw3000_hw_init_interrupt(void)
{
	LOG_DBG("dw3000_hw_init_interrupt: entry");
	if (conf.gpio_irq.port) {
		LOG_DBG("dw3000_hw_init_interrupt: setting up IRQ pin");

		gpio_pin_configure_dt(&conf.gpio_irq, GPIO_INPUT);
		gpio_init_callback(&gpio_cb, dw3000_hw_isr, BIT(conf.gpio_irq.pin));
		gpio_add_callback(conf.gpio_irq.port, &gpio_cb);
		// Don't enable interrupt yet - wait until custom handler is set
		gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_DISABLE);

		LOG_INF("IRQ on %s pin %d (disabled until handler set)", conf.gpio_irq.port->name,
				conf.gpio_irq.pin);
		LOG_DBG("dw3000_hw_init_interrupt: IRQ setup complete (disabled)");
		return 0;
	} else {
		LOG_DBG("dw3000_hw_init_interrupt: no IRQ pin configured");
		LOG_ERR("IRQ pin not configured");
		return -ENOENT;
	}
}

void dw3000_hw_interrupt_enable(void)
{
	if (conf.gpio_irq.port) {
		irq_disable_nesting--;
		if (irq_disable_nesting < 0) {
			LOG_WRN("IRQ enable called without matching disable (nesting=%d); clamping to 0",
				irq_disable_nesting);
			irq_disable_nesting = 0;
		}
		LOG_DBG("DW3000 IRQ enable: nesting now=%d", irq_disable_nesting);
	}
}

void dw3000_hw_interrupt_disable(void)
{
	if (conf.gpio_irq.port) {
		irq_disable_nesting++;
		LOG_DBG("DW3000 IRQ disable: nesting now=%d", irq_disable_nesting);
	}
}

bool dw3000_hw_interrupt_is_enabled(void)
{
	return irq_enabled;
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
	LOG_DBG("dw3000_hw_reset: entry");
	if (!conf.gpio_reset.port) {
		LOG_DBG("dw3000_hw_reset: no reset pin configured");
		LOG_ERR("No HW reset configured");
		return;
	}

	LOG_DBG("dw3000_hw_reset: performing reset sequence");
	gpio_pin_configure_dt(&conf.gpio_reset, GPIO_OUTPUT_ACTIVE);
	k_msleep(1); // 10 us?
	gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
	k_msleep(2);
	LOG_DBG("dw3000_hw_reset: reset sequence completed");
}

/** wakeup either using the WAKEUP pin or SPI CS */
void dw3000_hw_wakeup(void)
{
	LOG_DBG("dw3000_hw_wakeup: entry");
	if (conf.gpio_wakeup.port) {
		/* Use WAKEUP pin if available */
		LOG_DBG("dw3000_hw_wakeup: using WAKEUP pin");
		LOG_INF("WAKEUP PIN");
		gpio_pin_set_dt(&conf.gpio_wakeup, 1);
		k_msleep(1);
		gpio_pin_set_dt(&conf.gpio_wakeup, 0);
		LOG_DBG("dw3000_hw_wakeup: WAKEUP pin sequence completed");

	} else {
		/* Use SPI CS pin */
		LOG_DBG("dw3000_hw_wakeup: using SPI CS wakeup");
		LOG_INF("WAKEUP CS");
		dw3000_spi_wakeup();
		LOG_DBG("dw3000_hw_wakeup: SPI CS wakeup completed");
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
	int slot;

	if (handler == NULL) {
		LOG_WRN("Ignoring NULL interrupt handler registration request");
		return;
	}

	for (slot = 0; slot < DW3000_IRQ_HANDLER_SLOTS; slot++) {
		if (irq_handlers[slot] == handler) {
			LOG_DBG("Interrupt handler already registered in slot %d (%p)", slot,
				(void *)handler);
			return;
		}
	}

	for (slot = 0; slot < DW3000_IRQ_HANDLER_SLOTS; slot++) {
		if (irq_handlers[slot] == NULL) {
			irq_handlers[slot] = handler;
			LOG_INF("Registered DW3000 IRQ handler in slot %d (%p)", slot, (void *)handler);

			/* Enable GPIO IRQ when first handler is registered */
			if (slot == 0 && conf.gpio_irq.port) {
				irq_enabled = true;
				gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_EDGE_TO_ACTIVE);
				LOG_INF("DW3000 GPIO IRQ enabled (first handler registered)");
			}
			return;
		}
	}

	LOG_WRN("IRQ handler slots full; replacing slot 1 (%p -> %p)", (void *)irq_handlers[1],
		(void *)handler);
	irq_handlers[1] = handler;
}

void dw3000_hw_clear_interrupt_handler(void)
{
	int slot;

	for (slot = 0; slot < DW3000_IRQ_HANDLER_SLOTS; slot++) {
		irq_handlers[slot] = NULL;
	}

	LOG_INF("Cleared all DW3000 IRQ handlers");
}
