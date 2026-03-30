/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "dw1000_spi.h"
#include "dw1000_driver.h"

LOG_MODULE_DECLARE(dw1000, LOG_LEVEL_INF);

#define DWT_SPI_SLOW_FREQ   2000000
#define DWT_SPI_FAST_FREQ   20000000

int dw1000_spi_init(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;

	// Set up SPI configuration
	ctx->spi_cfg_slow.frequency = DWT_SPI_SLOW_FREQ;
	ctx->spi_cfg_slow.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	ctx->spi_cfg_slow.slave = 0;
	ctx->spi_cfg_slow.cs = (struct spi_cs_control){0};  // No CS control for now

	// Set current config to slow for initialization
	ctx->spi_cfg = &ctx->spi_cfg_slow;

	return 0;
}

int dw1000_spi_read(const struct device *dev,
                    uint16_t hdr_len, const uint8_t *hdr_buf,
                    uint32_t data_len, uint8_t *data)
{
	// TODO: Implement actual SPI communication
	// For now, just return success to allow build testing
	LOG_DBG("SPI read: hdr_len=%d, data_len=%d", hdr_len, data_len);
	return 0;
}

int dw1000_spi_write(const struct device *dev,
                     uint16_t hdr_len, const uint8_t *hdr_buf,
                     uint32_t data_len, const uint8_t *data)
{
	// TODO: Implement actual SPI communication
	// For now, just return success to allow build testing
	LOG_DBG("SPI write: hdr_len=%d, data_len=%d", hdr_len, data_len);
	return 0;
}