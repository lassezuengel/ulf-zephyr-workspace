/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * @author Patrick Rathje git@patrickrathje.de
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_CAU_H_
#define ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_CAU_H_

#include <zephyr/device.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/net/net_if.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <app/drivers/ieee802154/dw1000.h>
#include "dw1000_regs.h"

/* FRAME_LENGTH_ADDITIONAL defined in uwb_driver_api.h for consistency across all UWB drivers */

#define ZEPHYR_SPI 0
#define SPIM       0

#if ZEPHYR_SPI
#include <zephyr/drivers/spi.h>
#else
/* #include <nrfx_spi.h> */
#if SPIM
#include <nrfx_spim.h>
#else
#include <nrfx_spi.h>
#endif
#include <drivers/src/prs/nrfx_prs.h>
#include <zephyr/drivers/pinctrl.h>
#endif

#define DWT_MTM_START_FRAME_ID 0x01
#define DWT_MTM_RANGING_REFERENCE_FRAME_ID 0x04

// Reference ranging message types
#define DWT_MTM_REF_POLL 0x10
#define DWT_MTM_REF_RESPONSE 0x11
#define DWT_MTM_REF_FINAL 0x12
#define DWT_MTM_REF_POST_FINAL 0x13
#define DWT_MTM_REF_MEASUREMENT_EXCHANGE 0x14
#define DWT_MTM_REF_MEASUREMENT_EXCHANGE_ACK 0x15
#define DWT_MTM_MAX_FRAMES CONFIG_IEEE802154_DSKIEL_DW1000_MTM_MAX_RX_FRAMES

#define DWT_STATE_TX		0
#define DWT_STATE_CCA		1
#define DWT_STATE_RX_DEF_ON	2
#define DWT_STATE_IRQ_POLLING_EMU	3

// these are used when using the irq handler in a pseudo polling manner in the dwt ranging methods
#define DWT_IRQ_NONE 0
#define DWT_IRQ_TX 1
#define DWT_IRQ_RX 2
#define DWT_IRQ_FRAME_WAIT_TIMEOUT 4
#define DWT_IRQ_PREAMBLE_DETECT_TIMEOUT 5
#define DWT_IRQ_ERR 6
#define DWT_IRQ_HALF_DELAY_WARNING 7


/* SPI */

#if ZEPHYR_SPI
#define DWT_SPI_CSWAKEUP_FREQ 500000U
#define DWT_SPI_SLOW_FREQ 2000000U
#else
#define DWT_SPIM_CSWAKEUP_FREQ NRFX_KHZ_TO_HZ(500)
#define DWT_SPIM_SLOW_FREQ     NRFX_MHZ_TO_HZ(2)
#define DWT_SPIM_FAST_FREQ     NRFX_MHZ_TO_HZ(8)

#define DWT_SPI_CSWAKEUP_FREQ NRF_SPI_FREQ_500K
#define DWT_SPI_SLOW_FREQ     NRF_SPI_FREQ_2M
#define DWT_SPI_FAST_FREQ     NRF_SPI_FREQ_8M
#endif

#define DWT_SPI_TRANS_MAX_HDR_LEN	3
#define DWT_SPI_TRANS_REG_MAX_RANGE	0x3F
#define DWT_SPI_TRANS_SHORT_MAX_OFFSET	0x7F
#define DWT_SPI_TRANS_WRITE_OP		BIT(7)
#define DWT_SPI_TRANS_SUB_ADDR		BIT(6)
#define DWT_SPI_TRANS_EXTEND_ADDR	BIT(7)

struct dwt_phy_config {
	uint8_t channel;	/* Channel 1, 2, 3, 4, 5, 7 */
	uint8_t dr;	/* Data rate DWT_BR_110K, DWT_BR_850K, DWT_BR_6M8 */
	uint8_t prf;	/* PRF DWT_PRF_16M or DWT_PRF_64M */

	uint8_t rx_pac_l;		/* DWT_PAC8..DWT_PAC64 */
	uint8_t rx_shr_code;	/* RX SHR preamble code */
	uint8_t rx_ns_sfd;		/* non-standard SFD */
	uint16_t rx_sfd_to;	/* SFD timeout value (in symbols)
				 * (tx_shr_nsync + 1 + SFD_length - rx_pac_l)
				 */

	uint8_t tx_shr_code;	/* TX SHR preamble code */
	uint32_t tx_shr_nsync;	/* PLEN index, e.g. DWT_PLEN_64 */

	float t_shr;
	float t_phr;
	float t_dsym;
};

struct dwt_context {
	const struct device *dev;
	struct net_if *iface;

	// SPI configuration (platform-specific implementation is now abstracted)
	const struct spi_config *spi_cfg;
	struct spi_config spi_cfg_slow;

	// nRFx-specific configuration (used by nrfx SPI implementation)
	const struct pinctrl_dev_config *pcfg;
	void *spi_cfg_hw;  // Points to nrfx_spim_config_t or nrfx_spi_config_t

	struct gpio_callback gpio_cb;
	struct k_sem dev_lock;
	struct k_sem phy_sem;
	struct k_work irq_cb_work;
	struct k_thread thread;
	struct dwt_phy_config rf_cfg;
	atomic_t state;
	bool cca_busy;
	uint8_t phy_irq_event;
	uint32_t phy_irq_sys_stat;
	uint16_t sleep_mode;
	uint8_t mac_addr[8];

	uint16_t rx_ant_dly, tx_ant_dly;
	bool delayed_tx_short_ts_set;
	uint32_t delayed_tx_short_ts;

	uint64_t rx_ts;
	uint8_t rx_ttcko_rc_phase;
};

/* This struct is used to read all additional RX frame info at one push */
struct __attribute__((__packed__)) dwt_rx_info_regs {
	uint8_t rx_fqual[DWT_RX_FQUAL_LEN];
	uint8_t rx_ttcki[DWT_RX_TTCKI_LEN];
	uint8_t rx_ttcko[DWT_RX_TTCKO_LEN];
	/* RX_TIME without RX_RAWST */
	uint8_t rx_time[DWT_RX_TIME_FP_RAWST_OFFSET];
};

struct mtm_ranging_timing {
	uint64_t min_slot_length_us,
		phy_activate_rx_delay,
		phase_setup_delay, round_setup_delay,
		preamble_chunk_duration;
	uint16_t frame_timeout_period, preamble_timeout;
};

#if !ZEPHYR_SPI
bool spi_transfer(const uint8_t *tx_data, size_t tx_data_len,
    uint8_t *rx_buf, size_t rx_buf_size);
#endif

uint32_t dwt_reg_read_u32(const struct device *dev, uint8_t reg, uint16_t offset);
uint16_t dwt_reg_read_u16(const struct device *dev, uint8_t reg, uint16_t offset);
int dwt_register_read(const struct device *dev, uint8_t reg, uint16_t offset, size_t buf_len, uint8_t *buf);
void dwt_switch_buffers(const struct device *dev);

uint16_t dwt_cir_pwr_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg);
uint16_t dwt_fp_index_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg);
uint16_t dwt_fp_ampl1_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg);
uint16_t dwt_std_noise_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg);
uint16_t dwt_fp_ampl2_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg);
uint16_t dwt_fp_ampl3_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg);
uint8_t dwt_rcphase_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg);
uint64_t dwt_rx_timestamp_from_rx_info(const struct dwt_rx_info_regs *rx_inf_reg);

void dwt_setup_rx_timeout(const struct device *dev, uint16_t timeout);
void dwt_setup_preamble_detection_timeout(const struct device *dev, uint16_t pac_symbols);
void dwt_disable_txrx(const struct device *dev);
int dwt_enable_rx(const struct device *dev, uint16_t timeout, uint64_t dwt_rx_ts);
void dwt_fast_enable_rx(const struct device *dev, uint64_t dwt_rx_ts);
int dwt_fast_enable_tx(const struct device *dev, uint64_t dwt_tx_ts);
int setup_tx_frame(const struct device *dev, const uint8_t *data, uint8_t len);
int wait_for_phy(const struct device *dev);
void dwt_read_rx_info(const struct device *dev, struct dwt_rx_info_regs *rx_inf_reg);
void dwt_double_buffering_align(const struct device *dev);
void dwt_set_delayed_tx_short_ts(const struct device *dev, uint32_t short_ts);
void dwt_enable_accumulator_memory_access(const struct device *dev);
void dwt_disable_accumulator_memory_access(const struct device *dev);
uint64_t dwt_plan_delayed_tx(const struct device *dev, uint64_t uus_delay);
uint64_t dwt_rx_ts(const struct device *dev);
uint32_t dwt_system_short_ts(const struct device *dev);
uint64_t dwt_ts_to_fs(uint64_t ts);
uint64_t dwt_fs_to_ts(uint64_t fs);
uint64_t dwt_short_ts_to_fs(uint32_t ts);
uint32_t dwt_fs_to_short_ts(uint64_t fs);
uint64_t dwt_calculate_actual_tx_ts(uint32_t planned_short_ts, uint16_t tx_antenna_delay);
void     dwt_set_frame_filter(const struct device *dev, bool ff_enable, uint8_t ff_type);
uint8_t *dwt_get_mac(const struct device *dev);

void     dwt_set_antenna_delay_rx(const struct device *dev, uint16_t rx_delay_ts);
void     dwt_set_antenna_delay_tx(const struct device *dev, uint16_t tx_delay_ts);
uint16_t dwt_antenna_delay_rx(const struct device *dev);
uint16_t dwt_antenna_delay_tx(const struct device *dev);
uint32_t dwt_otp_antenna_delay(const struct device *dev);
uint8_t  dwt_rx_ttcko_rc_phase(const struct device *dev);
int      dwt_readcarrierintegrator(const struct device *dev);
float    dwt_rx_clock_ratio_offset(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_CAU_H_ */
