#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include "dw1000_driver.h"
#include "dw1000_regs.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <math.h>
LOG_MODULE_REGISTER(uwb_driver_dw1000, LOG_LEVEL_INF);

// ==================== TIME CONVERSION ====================

static uint64_t dw1000_us_to_timestamp(const struct device *dev, uint32_t us)
{
    return US_TO_DWT_TS(us);
}

static uint32_t dw1000_timestamp_to_us(const struct device *dev, uint64_t ts)
{
    return DWT_TS_TO_US(ts);
}

static uint64_t dw1000_system_timestamp(const struct device *dev)
{
    return dwt_system_ts(dev);
}

// ==================== TRANSCEIVER CONTROL ====================

static int dw1000_enable_rx(const struct device *dev, uint32_t timeout_us, uint64_t delayed_timestamp)
{
    if (delayed_timestamp != 0) {
        dwt_fast_enable_rx(dev, delayed_timestamp & DWT_TS_MASK);
        return 0;
    } else {
        dwt_enable_rx(dev, timeout_us, 0);
        return 0;
    }
}

// Simple immediate TX implementation for DW1000
static int dwt_immediate_tx(const struct device *dev)
{
    // For immediate TX, call dwt_fast_enable_tx with timestamp 0
    // The underlying implementation should handle this case
    return dwt_fast_enable_tx(dev, 0);
}

static int dw1000_start_tx(const struct device *dev, uint64_t delayed_timestamp)
{
    if (delayed_timestamp != 0) {
        dwt_fast_enable_tx(dev, delayed_timestamp & DWT_TS_MASK);
        return 0;
    } else {
        // Immediate TX - use immediate TX implementation
        LOG_DBG("Starting immediate TX");
        return dwt_immediate_tx(dev);
    }
}

static void dw1000_force_trx_off(const struct device *dev)
{
    dwt_disable_txrx(dev);
}

static uwb_irq_state_e dw1000_wait_for_irq(const struct device *dev)
{
    int irq_state = wait_for_phy(dev);

    // Map DW1000 IRQ states to universal states
    switch (irq_state) {
        case DWT_IRQ_RX:
            return UWB_IRQ_RX;
        case DWT_IRQ_TX:
            return UWB_IRQ_TX;
        case DWT_IRQ_FRAME_WAIT_TIMEOUT:
            return UWB_IRQ_FRAME_WAIT_TIMEOUT;
        case DWT_IRQ_PREAMBLE_DETECT_TIMEOUT:
            return UWB_IRQ_PREAMBLE_DETECT_TIMEOUT;
        case DWT_IRQ_ERR:
            return UWB_IRQ_ERR;
        case DWT_IRQ_HALF_DELAY_WARNING:
            return UWB_IRQ_HALF_DELAY_WARNING;
        default:
            return UWB_IRQ_NONE;
    }
}

// ==================== TIMEOUT MANAGEMENT ====================

static void dw1000_setup_frame_timeout(const struct device *dev, uint32_t timeout_us)
{
    dwt_setup_rx_timeout(dev, timeout_us);
}

static void dw1000_setup_preamble_timeout(const struct device *dev, uint16_t timeout_symbols)
{
    dwt_setup_preamble_detection_timeout(dev, timeout_symbols);
}

static void dw1000_clear_timeouts(const struct device *dev)
{
    dwt_setup_rx_timeout(dev, 0);
    dwt_setup_preamble_detection_timeout(dev, 0);
}

// ==================== DOUBLE BUFFERING ====================

static void dw1000_enable_double_buffering(const struct device *dev, bool auto_reenable)
{
    // DW1000 double buffering is controlled by compile-time defines
    // The enable/disable would need to be implemented in the DW1000 driver
    LOG_DBG("Double buffering control for DW1000: auto_reenable=%d", auto_reenable);
}

static void dw1000_switch_buffers(const struct device *dev)
{
    dwt_switch_buffers(dev);
}

static void dw1000_signal_buffer_free(const struct device *dev)
{
    // DW1000 doesn't have explicit buffer free signaling
    // This is handled automatically by buffer switching
}

static void dw1000_align_double_buffering(const struct device *dev)
{
    dwt_double_buffering_align(dev);
}

// ==================== FRAME I/O ====================

static void dw1000_setup_tx_frame(const struct device *dev, uint8_t *buffer, uint16_t length)
{
    setup_tx_frame(dev, buffer, length);
}

static void dw1000_read_rx_frame(const struct device *dev, uint8_t *buffer, uint16_t length, uint16_t offset)
{
    dwt_register_read(dev, DWT_RX_BUFFER_ID, offset, length, buffer);
}

static uint64_t dw1000_read_rx_timestamp(const struct device *dev, uwb_rx_diagnostics_t *diag)
{
    struct dwt_rx_info_regs rx_info;
    dwt_read_rx_info(dev, &rx_info);

    if (diag) {
        // Fill diagnostics structure
        uint32_t rx_finfo = dwt_reg_read_u32(dev, DWT_RX_FINFO_ID, DWT_RX_FINFO_OFFSET);
        struct dwt_context *ctx = dev->data;

        diag->cir_pwr = dwt_cir_pwr_from_info_reg(&rx_info);
        diag->rx_pacc = (rx_finfo & DWT_RX_FINFO_RXPACC_MASK) >> DWT_RX_FINFO_RXPACC_SHIFT;
        diag->fp_index = dwt_fp_index_from_info_reg(&rx_info);
        diag->fp_ampl1 = dwt_fp_ampl1_from_info_reg(&rx_info);
        diag->fp_ampl2 = dwt_fp_ampl2_from_info_reg(&rx_info);
        diag->fp_ampl3 = dwt_fp_ampl3_from_info_reg(&rx_info);
        diag->std_noise = dwt_std_noise_from_info_reg(&rx_info);
        diag->rx_phase = dwt_rcphase_from_info_reg(&rx_info);

        // Calculate RX level
        float a_const = (ctx->rf_cfg.prf == DWT_PRF_16M) ?
            DWT_RX_SIG_PWR_A_CONST_PRF16 : DWT_RX_SIG_PWR_A_CONST_PRF64;
        diag->rx_level = 10.0f * log10f(diag->cir_pwr * BIT(17) /
            (diag->rx_pacc * diag->rx_pacc)) - a_const;

        diag->cfo_ppm = 0.0f; // Will be filled by caller if CFO was read
    }

    return dwt_rx_timestamp_from_rx_info(&rx_info);
}

static uint64_t dw1000_read_tx_timestamp(const struct device *dev)
{
    // Re-implement using register read since dwt_read_tx_timestamp is static
    uint8_t ts_buf[sizeof(uint64_t)] = {0};
    dwt_register_read(dev, DWT_TX_TIME_ID, DWT_TX_TIME_TX_STAMP_OFFSET, DWT_TX_TIME_TX_STAMP_LEN, ts_buf);
    return sys_get_le64(ts_buf);
}

static uint16_t dw1000_get_rx_frame_length(const struct device *dev)
{
    uint32_t rx_finfo = dwt_reg_read_u32(dev, DWT_RX_FINFO_ID, DWT_RX_FINFO_OFFSET);
    return rx_finfo & DWT_RX_FINFO_RXFLEN_MASK;
}

// ==================== CIR/DIAGNOSTICS ACCESS ====================

static void dw1000_enable_cir_access(const struct device *dev)
{
    dwt_enable_accumulator_memory_access(dev);
}

static void dw1000_disable_cir_access(const struct device *dev)
{
    dwt_disable_accumulator_memory_access(dev);
}

static void dw1000_read_cir_data(const struct device *dev, uint8_t *buffer, uint16_t offset, uint16_t length)
{
    dwt_register_read(dev, DWT_ACC_MEM_ID, offset, length, buffer);
}

static void dw1000_read_diagnostics(const struct device *dev, uwb_rx_diagnostics_t *diag)
{
    // This is handled in dw1000_read_rx_timestamp to ensure consistency
    // Individual diagnostic reads could be implemented here if needed
}

static int32_t dw1000_read_carrier_integrator(const struct device *dev)
{
    return dwt_readcarrierintegrator(dev);
}

static int16_t dw1000_read_clock_offset(const struct device *dev)
{
    // DW1000 doesn't have direct clock offset read - would need implementation
    return 0;
}

// ==================== CONFIGURATION ====================

static int dw1000_configure(const struct device *dev, const uwb_config_t *config)
{
    // DW1000 configuration is handled at initialization
    // This function is not needed for current protocols
    return 0;
}

static int dw1000_get_config(const struct device *dev, uwb_config_t *config)
{
    struct dwt_context *ctx = dev->data;
    if (!ctx || !config) {
        return -EINVAL;
    }

    config->channel = ctx->rf_cfg.channel;

    // Convert DW1000 PRF index to UWB API enum value
    switch (ctx->rf_cfg.prf) {
        case DWT_PRF_16M:
            config->prf = UWB_PRF_16MHZ;
            break;
        case DWT_PRF_64M:
            config->prf = UWB_PRF_64MHZ;
            break;
        default:
            return -EINVAL;
    }

    // Convert DW1000 data rate index to UWB API enum value
    switch (ctx->rf_cfg.dr) {
        case DWT_BR_110K:
            config->datarate = UWB_DATARATE_110K;
            break;
        case DWT_BR_850K:
            config->datarate = UWB_DATARATE_850K;
            break;
        case DWT_BR_6M8:
            config->datarate = UWB_DATARATE_6M8;
            break;
        default:
            return -EINVAL;
    }

    // Convert DW1000 preamble length index to UWB API enum value
    switch (ctx->rf_cfg.tx_shr_nsync) {
        case DWT_PLEN_64:
            config->preamble_length = UWB_PREAMBLE_64;
            break;
        case DWT_PLEN_128:
            config->preamble_length = UWB_PREAMBLE_128;
            break;
        case DWT_PLEN_256:
            config->preamble_length = UWB_PREAMBLE_256;
            break;
        case DWT_PLEN_512:
            config->preamble_length = UWB_PREAMBLE_512;
            break;
        case DWT_PLEN_1024:
            config->preamble_length = UWB_PREAMBLE_1024;
            break;
        case DWT_PLEN_2048:
            config->preamble_length = UWB_PREAMBLE_2048;
            break;
        case DWT_PLEN_4096:
            config->preamble_length = UWB_PREAMBLE_4096;
            break;
        default:
            return -EINVAL;
    }

    // Convert DW1000 PAC size index to UWB API enum value
    switch (ctx->rf_cfg.rx_pac_l) {
        case DWT_PAC8:
            config->pac_size = UWB_PAC_8;
            break;
        case DWT_PAC16:
            config->pac_size = UWB_PAC_16;
            break;
        case DWT_PAC32:
            config->pac_size = UWB_PAC_32;
            break;
        case DWT_PAC64:
            config->pac_size = UWB_PAC_64;
            break;
        default:
            return -EINVAL;
    }

    config->sfd_type = ctx->rf_cfg.rx_ns_sfd;
    config->sfd_timeout = ctx->rf_cfg.rx_sfd_to;
    config->smart_power = false; // DW1000 doesn't have smart power

    return 0;
}

static void dw1000_set_tx_power(const struct device *dev, uint32_t power)
{
    // DW1000 doesn't have a separate dwt_set_tx_power function,
    // power is set during RF configuration
    LOG_DBG("TX power setting: 0x%08x", power);
}

static int dw1000_set_channel(const struct device *dev, uint8_t channel)
{
    // Use low-level DW1000 API
    return dwt_set_channel(dev, channel);
}

// ==================== DEVICE ACCESS MANAGEMENT ====================

static int dw1000_acquire_device(const struct device *dev)
{
    struct dwt_context *ctx = dev->data;
    return k_sem_take(&ctx->dev_lock, K_FOREVER);
}

static void dw1000_release_device(const struct device *dev)
{
    struct dwt_context *ctx = dev->data;
    k_sem_give(&ctx->dev_lock);
}

static void dw1000_get_antenna_delay(const struct device *dev, uwb_antenna_delay_t *delay)
{
    struct dwt_context *ctx = dev->data;
    delay->tx_ant_dly = ctx->tx_ant_dly;
    delay->rx_ant_dly = ctx->rx_ant_dly;
}

static void dw1000_set_antenna_delay_new(const struct device *dev, const uwb_antenna_delay_t *delay)
{
    struct dwt_context *ctx = dev->data;
    ctx->tx_ant_dly = delay->tx_ant_dly;
    ctx->rx_ant_dly = delay->rx_ant_dly;
}

static void dw1000_disable_txrx(const struct device *dev)
{
    dwt_disable_txrx(dev);
}

static void dw1000_set_frame_filter(const struct device *dev, uint16_t enable, uint16_t allow_beacon)
{
    dwt_set_frame_filter(dev, enable, allow_beacon);
}

static double dw1000_get_range_bias(const struct device *dev, uint8_t channel, float range, uwb_prf_e prf)
{
    // Convert UWB API PRF enum to DW1000 constant
    uint8_t prf_dwt;
    switch (prf) {
        case UWB_PRF_16MHZ:
            prf_dwt = DWT_PRF_16M;
            break;
        case UWB_PRF_64MHZ:
            prf_dwt = DWT_PRF_64M;
            break;
        default:
            LOG_ERR("Unknown PRF value: %d", prf);
            return 0.0;  // No correction for unknown PRF
    }

    // Call DW1000-specific range bias function with converted values
    return dwt_getrangebias(channel, range, prf_dwt);
}

// ==================== REGISTER ACCESS ====================

static uint32_t dw1000_read_reg_u32(const struct device *dev, uint32_t reg_id, uint16_t offset)
{
    return dwt_reg_read_u32(dev, reg_id, offset);
}

static uint16_t dw1000_read_reg_u16(const struct device *dev, uint32_t reg_id, uint16_t offset)
{
    return dwt_reg_read_u16(dev, reg_id, offset);
}

static void dw1000_write_reg_u32(const struct device *dev, uint32_t reg_id, uint16_t offset, uint32_t value)
{
    // DW1000 register write is not exposed directly - would need to be implemented
    // in the DW1000 driver if register writes are needed from the abstraction layer
    LOG_WRN("Register write not implemented for DW1000 abstraction layer");
}

static void dw1000_register_read(const struct device *dev, uint32_t reg_id, uint16_t offset, uint16_t length, uint8_t *buffer)
{
    dwt_register_read(dev, reg_id, offset, length, buffer);
}

// ==================== RADIO ABSTRACTION ====================

// DW1000 specific constants
#define DWT_TIME_UNITS_PS 15650  // Picoseconds per timestamp unit
#define DWT_RX_SIG_PWR_A_CONST_PRF16 113.77f
#define DWT_RX_SIG_PWR_A_CONST_PRF64 121.74f

static const uwb_radio_constants_t dw1000_radio_constants = {
    .timestamp_mask = DWT_TS_MASK,
    .cfo_conversion_factor = -0.000573121584378756f,
    .timestamp_resolution_ps = DWT_TIME_UNITS_PS,
    .rssi_constants = {
        .prf16 = DWT_RX_SIG_PWR_A_CONST_PRF16,
        .prf64 = DWT_RX_SIG_PWR_A_CONST_PRF64,
    }
};

static const uwb_radio_constants_t* dw1000_get_radio_constants(const struct device *dev)
{
    return &dw1000_radio_constants;
}

static uint8_t dw1000_read_system_state(const struct device *dev)
{
    // DW1000 doesn't have the same system state register as DW3000
    // Return a fixed state indicating we're operational
    return 0; // Could be IDLE or similar
}

static const char* dw1000_get_system_state_name(const struct device *dev)
{
    // DW1000 system state stub - always return operational
    return "OPERATIONAL";
}

// ==================== DRIVER DEFINITION ====================

static const uwb_driver_t dw1000_driver = {
    .driver_name = "DW1000",
    .driver_data = NULL,

    // Time conversion
    .us_to_timestamp = dw1000_us_to_timestamp,
    .timestamp_to_us = dw1000_timestamp_to_us,
    .system_timestamp = dw1000_system_timestamp,

    // Transceiver control
    .enable_rx = dw1000_enable_rx,
    .start_tx = dw1000_start_tx,
    .force_trx_off = dw1000_force_trx_off,
    .wait_for_irq = dw1000_wait_for_irq,
    .set_channel = dw1000_set_channel,

    // Timeout management
    .setup_frame_timeout = dw1000_setup_frame_timeout,
    .setup_preamble_timeout = dw1000_setup_preamble_timeout,
    .clear_timeouts = dw1000_clear_timeouts,

    // Double buffering
    .enable_double_buffering = dw1000_enable_double_buffering,
    .switch_buffers = dw1000_switch_buffers,
    .signal_buffer_free = dw1000_signal_buffer_free,
    .align_double_buffering = dw1000_align_double_buffering,

    // Frame I/O
    .setup_tx_frame = dw1000_setup_tx_frame,
    .read_rx_frame = dw1000_read_rx_frame,
    .read_rx_timestamp = dw1000_read_rx_timestamp,
    .read_tx_timestamp = dw1000_read_tx_timestamp,
    .get_rx_frame_length = dw1000_get_rx_frame_length,

    // CIR/Diagnostics
    .enable_cir_access = dw1000_enable_cir_access,
    .disable_cir_access = dw1000_disable_cir_access,
    .read_cir_data = dw1000_read_cir_data,
    .read_diagnostics = dw1000_read_diagnostics,
    .read_carrier_integrator = dw1000_read_carrier_integrator,
    .read_clock_offset = dw1000_read_clock_offset,

    // Configuration
    .configure = dw1000_configure,
    .get_config = dw1000_get_config,
    .set_tx_power = dw1000_set_tx_power,
    .disable_txrx = dw1000_disable_txrx,
    .set_frame_filter = dw1000_set_frame_filter,
    .get_range_bias = dw1000_get_range_bias,

    // Register access
    .read_reg_u32 = dw1000_read_reg_u32,
    .read_reg_u16 = dw1000_read_reg_u16,
    .write_reg_u32 = dw1000_write_reg_u32,
    .register_read = dw1000_register_read,
    .read_system_state = dw1000_read_system_state,
    .get_system_state_name = dw1000_get_system_state_name,

    // Device access management
    .acquire_device = dw1000_acquire_device,
    .release_device = dw1000_release_device,
    .get_antenna_delay = dw1000_get_antenna_delay,
    .set_antenna_delay = dw1000_set_antenna_delay_new,

    // Radio abstraction
    .get_radio_constants = dw1000_get_radio_constants,
};

// Driver registration function
int uwb_driver_dw1000_init(const struct device *dev)
{
    // Configure default state for IEEE 802.15.4 compatibility
    struct dwt_context *ctx = dev->data;
    if (ctx) {
        // Enable IRQ polling emulation by default
        atomic_set_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU);
        // Disable auto RX default on
        atomic_clear_bit(&ctx->state, DWT_STATE_RX_DEF_ON);
    }

    return uwb_driver_register(dev, &dw1000_driver);
}
