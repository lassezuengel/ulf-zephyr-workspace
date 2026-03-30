/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include "deca_interface.h"
#include "deca_probe_interface.h"
#include "dw3000_spi.h"

/* Zephyr SPI context */
static const struct device *spi_dev;
static struct spi_config spi_cfg;
static struct spi_cs_control cs_ctrl;

/* GPIO devices */
static const struct device *rst_gpio_dev;
static gpio_pin_t rst_gpio_pin;
static const struct device *irq_gpio_dev;
static gpio_pin_t irq_gpio_pin;

/* SPI transfer functions for DW3000 */
int readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                uint16_t readLength, uint8_t *readBuffer)
{
    struct spi_buf tx_bufs[2];
    struct spi_buf rx_bufs[2];
    struct spi_buf_set tx;
    struct spi_buf_set rx;
    int ret;

    /* Setup TX buffers */
    tx_bufs[0].buf = headerBuffer;
    tx_bufs[0].len = headerLength;
    tx_bufs[1].buf = NULL;
    tx_bufs[1].len = readLength;

    tx.buffers = tx_bufs;
    tx.count = 2;

    /* Setup RX buffers */
    rx_bufs[0].buf = NULL;
    rx_bufs[0].len = headerLength;
    rx_bufs[1].buf = readBuffer;
    rx_bufs[1].len = readLength;

    rx.buffers = rx_bufs;
    rx.count = 2;

    ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);

    return (ret == 0) ? 0 : -1;
}

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint16_t bodyLength, const uint8_t *bodyBuffer)
{
    struct spi_buf tx_bufs[2];
    struct spi_buf_set tx;
    int ret;

    /* Setup TX buffers */
    tx_bufs[0].buf = (uint8_t *)headerBuffer;
    tx_bufs[0].len = headerLength;
    tx_bufs[1].buf = (uint8_t *)bodyBuffer;
    tx_bufs[1].len = bodyLength;

    tx.buffers = tx_bufs;
    tx.count = (bodyLength > 0) ? 2 : 1;

    ret = spi_write(spi_dev, &spi_cfg, &tx);

    return (ret == 0) ? 0 : -1;
}

/* Sleep functions */
void deca_sleep(unsigned int time_ms)
{
    k_sleep(K_MSEC(time_ms));
}

void deca_usleep(unsigned long time_us)
{
    k_sleep(K_USEC(time_us));
}

/* Mutex functions */
static struct k_mutex spi_mutex;

decaIrqStatus_t decamutexon(void)
{
    k_mutex_lock(&spi_mutex, K_FOREVER);
    return 0;
}

void decamutexoff(decaIrqStatus_t stat)
{
    k_mutex_unlock(&spi_mutex);
}

/* Reset control */
void reset_DWIC(void)
{
    gpio_pin_set(rst_gpio_dev, rst_gpio_pin, 0);
    k_sleep(K_MSEC(1));
    gpio_pin_set(rst_gpio_dev, rst_gpio_pin, 1);
    k_sleep(K_MSEC(2));
}

/* SPI rate control functions */
int setslowrate(void)
{
    /* Set SPI to slow rate for initial communication (2 MHz) */
    spi_cfg.frequency = 2000000U;
    return 0;
}

int setfastrate(void)
{
    /* Set SPI to fast rate for normal operation (20 MHz) */
    spi_cfg.frequency = 20000000U;
    return 0;
}

/* Forward declaration of writetospiwithcrc function */
int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer,
                     uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8)
{
    /* For now, just call writetospi and ignore CRC */
    return writetospi(headerLength, headerBuffer, bodyLength, bodyBuffer);
}

/* DW3000 SPI function structure */
static struct dwt_spi_s dw3000_spi_fct_s = {
    .readfromspi = readfromspi,
    .writetospi = writetospi,
    .writetospiwithcrc = writetospiwithcrc,
    .setslowrate = setslowrate,
    .setfastrate = setfastrate
};

/* Wakeup function */
void wakeup_device_with_io(void)
{
    /* Toggle CS pin to wake up DW3000 */
    if (spi_cfg.cs && spi_cfg.cs->gpio.port) {
        gpio_pin_set(spi_cfg.cs->gpio.port, spi_cfg.cs->gpio.pin, 0);
        k_sleep(K_USEC(1));
        gpio_pin_set(spi_cfg.cs->gpio.port, spi_cfg.cs->gpio.pin, 1);
        k_sleep(K_MSEC(1));
    }
}

/* Note: dw3000_probe_interf is defined in deca_port.c */

/* Initialize Zephyr SPI for DW3000 */
int dw3000_zephyr_spi_init(const struct device *spi_device,
                          const struct spi_config *config,
                          const struct device *reset_gpio,
                          gpio_pin_t reset_pin,
                          const struct device *int_gpio,
                          gpio_pin_t int_pin)
{
    spi_dev = spi_device;
    memcpy(&spi_cfg, config, sizeof(spi_cfg));

    rst_gpio_dev = reset_gpio;
    rst_gpio_pin = reset_pin;
    irq_gpio_dev = int_gpio;
    irq_gpio_pin = int_pin;

    k_mutex_init(&spi_mutex);

    return 0;
}
