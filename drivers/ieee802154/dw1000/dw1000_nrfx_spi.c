/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DW1000 nRFx SPI Implementation
 *
 * This implementation provides direct nRFx SPI access using the standard
 * nRFx SPI interface (not SPIM) for better board compatibility.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_spi.h>
#include <hal/nrf_spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <nrfx_glue.h>
#include <string.h>

#include "dw1000_spi.h"
#include "dw1000_driver.h"

LOG_MODULE_DECLARE(dw1000, LOG_LEVEL_INF);

#define SPI_NODE DT_NODELABEL(spi2)
/* Suppress unused variable warning for Zephyr-generated pinctrl config */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-const-variable"
PINCTRL_DT_DEFINE(SPI_NODE);
#pragma GCC diagnostic pop
static nrfx_spi_t spi = NRFX_SPI_INSTANCE(2);

static bool spi_initialized = false;
static nrfx_spi_config_t spi_config;

bool spi_transfer(const uint8_t *tx_data, size_t tx_data_len,
                  uint8_t *rx_buf, size_t rx_buf_size)
{
	nrfx_err_t err;
	nrfx_spi_xfer_desc_t xfer_desc = {
		.p_tx_buffer = tx_data,
		.tx_length = tx_data_len,
		.p_rx_buffer = rx_buf,
		.rx_length = rx_buf_size,
	};

	err = nrfx_spi_xfer(&spi, &xfer_desc, 0);

	if (err != NRFX_SUCCESS) {
		printk("nrfx_spi_xfer() failed: 0x%08x\n", err);
		return false;
	}

	return true;
}

int dw1000_spi_init(const struct device *dev)
{
	nrfx_err_t err;
	struct dwt_context *ctx = dev->data;


	if (spi_initialized) {
		return 0;
	}

	// Use device tree pin configuration for standard nRFx SPI
#define SPI_PINCTRL_NODE DT_CHILD(DT_PINCTRL_0(SPI_NODE, 0), group1)
#define SCK_PIN (DT_PROP_BY_IDX(SPI_PINCTRL_NODE, psels, 0) & 0x3F)
#define MOSI_PIN (DT_PROP_BY_IDX(SPI_PINCTRL_NODE, psels, 1) & 0x3F)
#define MISO_PIN (DT_PROP_BY_IDX(SPI_PINCTRL_NODE, psels, 2) & 0x3F)

	spi_config = (nrfx_spi_config_t)NRFX_SPI_DEFAULT_CONFIG(
		SCK_PIN, MOSI_PIN, MISO_PIN,
		NRF_DT_GPIOS_TO_PSEL(SPI_NODE, cs_gpios));

	spi_config.frequency = DWT_SPI_SLOW_FREQ;
	spi_config.skip_gpio_cfg = false;
	spi_config.skip_psel_cfg = false; // only when using pinctrl
	spi_config.mode = NRF_SPI_MODE_0;

	ctx->spi_cfg_hw = &spi_config;

	err = nrfx_spi_init(&spi, &spi_config, NULL, NULL);
	if (err != NRFX_SUCCESS) {
		printk("nrfx_spi_init() failed: 0x%08x\n", err);
		return -EIO;
	}

	spi_initialized = true;
	printk("Switched to nRFx SPI\n");
	return 0;
}

int dw1000_spi_read(const struct device *dev,
                    uint16_t hdr_len, const uint8_t *hdr_buf,
                    uint32_t data_len, uint8_t *data)
{
	// merge together frames, before calling spi_transfer
	uint8_t transfer_buf[hdr_len + data_len];
	uint8_t rx_buf[hdr_len + data_len];

	// Copy header to transfer buffer
	memcpy(transfer_buf, hdr_buf, hdr_len);

	// Copy data to transfer buffer, starting right after the header
	// (This matches the original implementation exactly)
	memcpy(transfer_buf + hdr_len, data, data_len);

	// Now call spi_transfer with the combined buffer
	bool success = spi_transfer(transfer_buf, hdr_len + data_len, rx_buf, hdr_len + data_len);

	if (!success) {
		printk("SPI transfer failed\n");
		return -1;
	}

	// Copy received data back (skip the header part of rx_buf)
	memcpy(data, rx_buf + hdr_len, data_len);

	return 0;
}

int dw1000_spi_write(const struct device *dev,
                     uint16_t hdr_len, const uint8_t *hdr_buf,
                     uint32_t data_len, const uint8_t *data)
{
	// Match original implementation exactly - just call read function
	return dw1000_spi_read(dev, hdr_len, hdr_buf, data_len, (uint8_t*)data);
}

void dw1000_spi_set_slow(const struct device *dev, uint32_t freq)
{
	struct dwt_context *ctx = dev->data;
	nrfx_spi_config_t *spi_config = (nrfx_spi_config_t*)ctx->spi_cfg_hw;

	if (!spi_config) {
		printk("SPI config not available\n");
		return;
	}

	// Update frequency and reconfigure SPI
	spi_config->frequency = freq;

	nrfx_err_t err = nrfx_spi_reconfigure(&spi, spi_config);
	if (err != NRFX_SUCCESS) {
		printk("SPI reconfigure slow failed: 0x%08x\n", err);
	}
}

void dw1000_spi_set_fast(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;
	nrfx_spi_config_t *spi_config = (nrfx_spi_config_t*)ctx->spi_cfg_hw;

	if (!spi_config) {
		printk("SPI config not available\n");
		return;
	}

	// Set to fast frequency (8MHz) and reconfigure SPI
	spi_config->frequency = DWT_SPI_FAST_FREQ;

	nrfx_err_t err = nrfx_spi_reconfigure(&spi, spi_config);
	if (err != NRFX_SUCCESS) {
		printk("SPI reconfigure fast failed: 0x%08x\n", err);
	}
}
