/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DW1000_SPI_H
#define DW1000_SPI_H

#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SPI interface for DW1000
 *
 * @param dev DW1000 device instance
 * @return 0 on success, negative error code on failure
 */
int dw1000_spi_init(const struct device *dev);

/**
 * @brief Read data from DW1000 via SPI
 *
 * @param dev DW1000 device instance
 * @param hdr_len Length of header buffer
 * @param hdr_buf Header buffer
 * @param data_len Length of data to read
 * @param data Buffer to store read data
 * @return 0 on success, negative error code on failure
 */
int dw1000_spi_read(const struct device *dev,
                    uint16_t hdr_len, const uint8_t *hdr_buf,
                    uint32_t data_len, uint8_t *data);

/**
 * @brief Write data to DW1000 via SPI
 *
 * @param dev DW1000 device instance
 * @param hdr_len Length of header buffer
 * @param hdr_buf Header buffer
 * @param data_len Length of data to write
 * @param data Data buffer to write
 * @return 0 on success, negative error code on failure
 */
int dw1000_spi_write(const struct device *dev,
                     uint16_t hdr_len, const uint8_t *hdr_buf,
                     uint32_t data_len, const uint8_t *data);

/**
 * @brief Set SPI frequency to slow mode
 *
 * @param dev DW1000 device instance
 * @param freq Frequency to set (e.g., DWT_SPI_CSWAKEUP_FREQ, DWT_SPI_SLOW_FREQ)
 */
void dw1000_spi_set_slow(const struct device *dev, uint32_t freq);

/**
 * @brief Set SPI frequency to fast mode
 *
 * @param dev DW1000 device instance
 */
void dw1000_spi_set_fast(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* DW1000_SPI_H */
