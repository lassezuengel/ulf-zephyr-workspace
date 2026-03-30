/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DW3000 nRFx SPI Implementation
 *
 * This implementation provides direct nRFx SPIM access for DW3000,
 * following the same pattern as DW1000 but using SPIM instead of SPI.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_spim.h>
#include <hal/nrf_spim.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <nrfx_glue.h>
#include <string.h>

#include "dw3000_spi.h"
#include "deca_device_api.h"

LOG_MODULE_DECLARE(dw3000, CONFIG_IEEE802154_DW3000_LOG_LEVEL);

#define DWT_SPI_SLOW_FREQ   2000000
#define DWT_SPI_FAST_FREQ   20000000

#define SPI_NODE DT_NODELABEL(spi3)
PINCTRL_DT_DEFINE(SPI_NODE);
static nrfx_spim_t spi = NRFX_SPIM_INSTANCE(3);

static bool spi_initialized = false;
static nrfx_spim_config_t spi_config;

bool spi_transfer(const uint8_t *tx_data, size_t tx_data_len,
                  uint8_t *rx_buf, size_t rx_buf_size)
{
	nrfx_err_t err;

	// TODO: Remove debug printk statements after SPI issue is resolved
	// if (tx_data_len > 0 && tx_data != NULL) {
	// 	printk("SPI TX (%zu): ", tx_data_len);
	// 	for (size_t i = 0; i < tx_data_len && i < 8; i++) {
	// 		printk("%02x ", tx_data[i]);
	// 	}
	// 	printk("\n");
	// }

	nrfx_spim_xfer_desc_t xfer_desc = {
		.p_tx_buffer = tx_data,
		.tx_length = tx_data_len,
		.p_rx_buffer = rx_buf,
		.rx_length = rx_buf_size,
	};

	err = nrfx_spim_xfer(&spi, &xfer_desc, 0);

	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_spim_xfer() failed: 0x%08x", err);
		return false;
	}

	// TODO: Remove debug printk statements after SPI issue is resolved
	// if (rx_buf_size > 0 && rx_buf != NULL) {
	// 	printk("SPI RX (%zu): ", rx_buf_size);
	// 	for (size_t i = 0; i < rx_buf_size && i < 8; i++) {
	// 		printk("%02x ", rx_buf[i]);
	// 	}
	// 	printk("\n");
	// }

	return true;
}

int dw3000_spi_init(void)
{
	nrfx_err_t err;

	printk("[DEBUG] dw3000_spi_init: Entry point\n");
	if (spi_initialized) {
		printk("[DEBUG] dw3000_spi_init: Already initialized, returning\n");
		return 0;
	}

	// Use device tree pin configuration for nRFx SPIM
#define SPI_PINCTRL_NODE DT_CHILD(DT_PINCTRL_0(SPI_NODE, 0), group1)
#define SCK_PIN (DT_PROP_BY_IDX(SPI_PINCTRL_NODE, psels, 0) & 0x3F)
#define MOSI_PIN (DT_PROP_BY_IDX(SPI_PINCTRL_NODE, psels, 1) & 0x3F)
#define MISO_PIN (DT_PROP_BY_IDX(SPI_PINCTRL_NODE, psels, 2) & 0x3F)

	spi_config = (nrfx_spim_config_t)NRFX_SPIM_DEFAULT_CONFIG(
		SCK_PIN, MOSI_PIN, MISO_PIN,
		NRF_DT_GPIOS_TO_PSEL(SPI_NODE, cs_gpios));

	spi_config.frequency = DWT_SPI_SLOW_FREQ;
	spi_config.skip_gpio_cfg = false;
	spi_config.skip_psel_cfg = false;
	spi_config.mode = NRF_SPIM_MODE_0;

	printk("[DEBUG] dw3000_spi_init: Calling nrfx_spim_init\n");
	err = nrfx_spim_init(&spi, &spi_config, NULL, NULL);
	if (err != NRFX_SUCCESS) {
		printk("[DEBUG] dw3000_spi_init: nrfx_spim_init FAILED: 0x%08x\n", err);
		LOG_ERR("nrfx_spim_init() failed: 0x%08x", err);
		return -EIO;
	}

	printk("[DEBUG] dw3000_spi_init: nrfx_spim_init SUCCESS\n");
	spi_initialized = true;
	LOG_INF("DW3000 nRFx SPIM initialized");
	return 0;
}

void dw3000_spi_fini(void)
{
	if (spi_initialized) {
		nrfx_spim_uninit(&spi);
		spi_initialized = false;
	}
}

int32_t dw3000_spi_read(uint16_t headerLength, uint8_t* headerBuffer,
                       uint16_t readLength, uint8_t* readBuffer)
{
	// Merge together frames, before calling spi_transfer
	uint8_t transfer_buf[headerLength + readLength];
	uint8_t rx_buf[headerLength + readLength];

	// Copy header to transfer buffer
	memcpy(transfer_buf, headerBuffer, headerLength);

	// Zero out the data portion for reading
	memset(transfer_buf + headerLength, 0, readLength);

	// Perform the SPI transfer
	bool success = spi_transfer(transfer_buf, headerLength + readLength,
	                           rx_buf, headerLength + readLength);

	if (!success) {
		LOG_ERR("SPI transfer failed");
		return DWT_ERROR;
	}

	// Copy received data back (skip the header part of rx_buf)
	memcpy(readBuffer, rx_buf + headerLength, readLength);

	return DWT_SUCCESS;
}

int32_t dw3000_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
                        uint16_t bodyLength, const uint8_t* bodyBuffer)
{
	// Merge header and body into single buffer
	uint8_t transfer_buf[headerLength + bodyLength];

	// Copy header
	memcpy(transfer_buf, headerBuffer, headerLength);

	// Copy body if present
	if (bodyLength > 0 && bodyBuffer) {
		memcpy(transfer_buf + headerLength, bodyBuffer, bodyLength);
	}

	// Perform the SPI transfer
	bool success = spi_transfer(transfer_buf, headerLength + bodyLength, NULL, 0);

	if (!success) {
		LOG_ERR("SPI write failed");
		return DWT_ERROR;
	}

	return DWT_SUCCESS;
}

int32_t dw3000_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
                             uint16_t bodyLength, const uint8_t* bodyBuffer,
                             uint8_t crc8)
{
	// Merge header, body and CRC into single buffer
	uint8_t transfer_buf[headerLength + bodyLength + 1];

	// Copy header
	memcpy(transfer_buf, headerBuffer, headerLength);

	// Copy body if present
	if (bodyLength > 0 && bodyBuffer) {
		memcpy(transfer_buf + headerLength, bodyBuffer, bodyLength);
	}

	// Add CRC
	transfer_buf[headerLength + bodyLength] = crc8;

	// Perform the SPI transfer
	bool success = spi_transfer(transfer_buf, headerLength + bodyLength + 1, NULL, 0);

	if (!success) {
		LOG_ERR("SPI write with CRC failed");
		return DWT_ERROR;
	}

	return DWT_SUCCESS;
}

void dw3000_spi_speed_slow(void)
{
	if (!spi_initialized) {
		LOG_ERR("SPI not initialized");
		return;
	}

	// Update frequency and reconfigure SPI
	spi_config.frequency = DWT_SPI_SLOW_FREQ;

	nrfx_err_t err = nrfx_spim_reconfigure(&spi, &spi_config);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("SPI reconfigure slow failed: 0x%08x", err);
	}
}

void dw3000_spi_speed_fast(void)
{
	if (!spi_initialized) {
		LOG_ERR("SPI not initialized");
		return;
	}

	// Set to fast frequency and reconfigure SPI
	spi_config.frequency = DWT_SPI_FAST_FREQ;

	nrfx_err_t err = nrfx_spim_reconfigure(&spi, &spi_config);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("SPI reconfigure fast failed: 0x%08x", err);
	}
}

void dw3000_spi_wakeup(void)
{
	// TODO: Implement proper wakeup sequence
	k_sleep(K_USEC(500));
}

void dw3000_spi_trace_output(void)
{
	// TODO: Implement SPI trace output
}
