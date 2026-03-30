/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/drivers/ieee802154/uwb_driver_api.h>
#include "deca_device_api.h"
#include "deca_interface.h"
#include "deca_ull.h"
#include "deca_private.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "dw3000_radio_constants.h"
#include "dw3000/dw3000_deca_regs.h"
#include "dw3000/dw3000_deca_vals.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <math.h>

LOG_MODULE_REGISTER(uwb_driver_dw3000, LOG_LEVEL_INF);

// DW3000 IRQ states matching DW1000
#define DW3000_IRQ_NONE 0
#define DW3000_IRQ_TX 1
#define DW3000_IRQ_RX 2
#define DW3000_IRQ_FRAME_WAIT_TIMEOUT 4
#define DW3000_IRQ_PREAMBLE_DETECT_TIMEOUT 5
#define DW3000_IRQ_ERR 6
#define DW3000_IRQ_HALF_DELAY_WARNING 7

// DW3000 state flags
#define DW3000_STATE_IRQ_POLLING_EMU 3

// DW3000 buffer access tracking (similar to vendor driver)
typedef enum {
    DW3000_BUFFER_ACCESS_0 = 0,    // Access buffer 0
    DW3000_BUFFER_ACCESS_1 = 1,    // Access buffer 1
    DW3000_BUFFER_ACCESS_DEFAULT = 2  // Default/single buffer mode
} dw3000_buffer_access_t;

struct dw3000_context {
	struct k_sem dev_lock;
	struct k_sem phy_sem;
	struct k_work irq_cb_work;
	atomic_t state;
	uint8_t phy_irq_event;
	uint32_t phy_irq_sys_stat;
	uint32_t spi_crc_error_count;  // Track SPI CRC errors for debugging
	dw3000_buffer_access_t current_rx_buffer;  // Which buffer contains valid RX data
};

static struct dw3000_context dw3000_ctx = {
	.dev_lock = Z_SEM_INITIALIZER(dw3000_ctx.dev_lock, 1, 1),
	.phy_sem = Z_SEM_INITIALIZER(dw3000_ctx.phy_sem, 0, 1),
};


// Static dwchip_t structure for direct hardware access (avoiding probe indirection)
static struct dwchip_s dw3000_chip = { 0 };

// External references
extern const struct dwt_spi_s dw3000_spi_fct;
extern const struct dwt_driver_s dw3000_driver;
extern void dw3000_hw_wakeup(void);

// ==================== TIME CONVERSION ====================

static uint64_t dw3000_us_to_timestamp(const struct device *dev, uint32_t us)
{
    return (uint64_t)us * DW3000_TIME_UNITS_PER_US;  /* Cast to uint64_t BEFORE multiplication to avoid 32-bit overflow */
}

static uint32_t dw3000_timestamp_to_us(const struct device *dev, uint64_t ts)
{
    return (uint32_t)(ts / DW3000_TIME_UNITS_PER_US);
}

static uint64_t dw3000_system_timestamp(const struct device *dev)
{
    uint8_t ts_buffer[4];  // dwt_readsystime reads 4 bytes (SYS_TIME_LEN = 4)
    dwt_readsystime(ts_buffer);

    // Convert 4-byte little-endian buffer to uint64_t
    uint64_t timestamp = ((uint64_t)ts_buffer[3] << 24) |
                        ((uint64_t)ts_buffer[2] << 16) |
                        ((uint64_t)ts_buffer[1] << 8) |
                        ((uint64_t)ts_buffer[0]);

    return (timestamp << 8) & DW3000_TIMESTAMP_MASK;
}

// ==================== TRANSCEIVER CONTROL ====================

static int dw3000_enable_rx(const struct device *dev, uint32_t timeout_us, uint64_t delayed_timestamp)
{
    // Set bias_trim to 7 for receiving (from ull_rxenable)
    uint32_t pll_common_val = RF_PLL_COMMON;
    dwt_writetodevice(PLL_COMMON_ID, 0, 4, (uint8_t*)&pll_common_val);

    if (delayed_timestamp > 0) {
        dwt_setdelayedtrxtime((uint32_t)(delayed_timestamp >> 8));
        dwt_writetodevice(CMD_DRX, 0, 0, NULL);
    } else {
        if (timeout_us > 0) {
            dwt_setrxtimeout(timeout_us);
        }
        dwt_writetodevice(CMD_RX, 0, 0, NULL);
    }

    return 0;
}

static int dw3000_start_tx(const struct device *dev, uint64_t delayed_timestamp)
{
    // Direct TX implementation bypassing ull_starttx for simpler logic
    if (delayed_timestamp > 0) {
        // Set delayed transmission timestamp (upper 32 bits)
        dwt_setdelayedtrxtime((uint32_t)(delayed_timestamp >> 8));
        LOG_DBG("Starting delayed TX at 0x%08x", (uint32_t)(delayed_timestamp >> 8));

        // Delayed TX without response (we don't need expect_response for now)
        dwt_writetodevice(CMD_DTX, 0, 0, NULL);
    } else {
        LOG_DBG("Starting immediate TX");

        // Immediate TX without response (we don't need expect_response for now)
        dwt_writetodevice(CMD_TX, 0, 0, NULL);
    }

    return 0;
}

static void dw3000_force_trx_off(const struct device *dev)
{
    dwt_forcetrxoff();
}

// DW3000 wait_for_phy implementation with semaphore blocking
static inline int wait_for_phy(const struct device *dev) {
    ARG_UNUSED(dev);

    // Block on the PHY semaphore until an interrupt occurs
    if (k_sem_take(&dw3000_ctx.phy_sem, K_FOREVER)) {
        // Timeout or error occurred
        return DW3000_IRQ_NONE;
    }

    // Get the IRQ event that was stored by the interrupt handler
    uint8_t irq_state = dw3000_ctx.phy_irq_event;
    dw3000_ctx.phy_irq_event = DW3000_IRQ_NONE;

    return irq_state;
}

// DW3000 interrupt work handler - processes DW3000 interrupts in work queue context
static void dw3000_irq_work_handler(struct k_work *item)
{
    struct dw3000_context *ctx = CONTAINER_OF(item, struct dw3000_context, irq_cb_work);
    uint32_t sys_stat;
    uint8_t free_phy_sem = 0;

    // Take device lock to read status register
    k_sem_take(&ctx->dev_lock, K_FOREVER);

    // Read the system status register to determine interrupt source
    uint8_t status_buf[4];
    dwt_readfromdevice(SYS_STATUS_ID, 0, 4, status_buf);
    sys_stat = (uint32_t)status_buf[0] | ((uint32_t)status_buf[1] << 8) |
               ((uint32_t)status_buf[2] << 16) | ((uint32_t)status_buf[3] << 24);
    ctx->phy_irq_sys_stat = sys_stat;

    // Track and clear SPI CRC errors for debugging statistics
    if (sys_stat & SYS_STATUS_SPICRCE_BIT_MASK) {
        ctx->spi_crc_error_count++;
        uint32_t clear_mask = SYS_STATUS_SPICRCE_BIT_MASK;
        dwt_writetodevice(SYS_STATUS_ID, 0, 4, (uint8_t*)&clear_mask);
    }

    // Clear unwanted TX status bits that don't have interrupts enabled
    // These bits get set during transmission but we don't want them to accumulate
    if (sys_stat & (SYS_STATUS_TXFRB_BIT_MASK | SYS_STATUS_TXPRS_BIT_MASK | SYS_STATUS_TXPHS_BIT_MASK)) {
        uint32_t clear_mask = (sys_stat & (SYS_STATUS_TXFRB_BIT_MASK | SYS_STATUS_TXPRS_BIT_MASK | SYS_STATUS_TXPHS_BIT_MASK));
        dwt_writetodevice(SYS_STATUS_ID, 0, 4, (uint8_t*)&clear_mask);
    }

    // Process different interrupt types and map to internal IRQ states
    if (sys_stat & SYS_STATUS_TXFRS_BIT_MASK) {
        // TX frame sent
        ctx->phy_irq_event = DW3000_IRQ_TX;
        free_phy_sem = 1;
        // Clear TX interrupt
        uint32_t clear_mask = SYS_STATUS_TXFRS_BIT_MASK;
        dwt_writetodevice(SYS_STATUS_ID, 0, 4, (uint8_t*)&clear_mask);
    } else if (sys_stat & SYS_STATUS_RXFCG_BIT_MASK) {
        // RX frame received successfully
        ctx->phy_irq_event = DW3000_IRQ_RX;
        free_phy_sem = 1;

        // Read RDB_STATUS to determine which buffer contains valid data
        uint8_t rdb_status = 0;
        dwt_readfromdevice(RDB_STATUS_ID, 0, 1, &rdb_status);

        // Determine which buffer has valid RX data based on RXFCG bits
        if (rdb_status & RDB_STATUS_RXFCG1_BIT_MASK) {
            // Buffer 1 has valid data
            ctx->current_rx_buffer = DW3000_BUFFER_ACCESS_1;
            // Clear buffer 1 events in RDB_STATUS
            uint8_t clear_buf1 = DWT_RDB_STATUS_CLEAR_BUFF1_EVENTS;
            dwt_writetodevice(RDB_STATUS_ID, 0, 1, &clear_buf1);
        } else if (rdb_status & RDB_STATUS_RXFCG0_BIT_MASK) {
            // Buffer 0 has valid data
            ctx->current_rx_buffer = DW3000_BUFFER_ACCESS_0;
            // Clear buffer 0 events in RDB_STATUS
            uint8_t clear_buf0 = DWT_RDB_STATUS_CLEAR_BUFF0_EVENTS;
            dwt_writetodevice(RDB_STATUS_ID, 0, 1, &clear_buf0);
        } else {
            // No buffer has valid data - use default
            ctx->current_rx_buffer = DW3000_BUFFER_ACCESS_DEFAULT;
            printk("DW3000_ISR_WARNING: No RXFCG bits set (RDB_STATUS=0x%02x), using default\n", rdb_status);
        }

        // Clear RX good interrupt in main SYS_STATUS register
        uint32_t clear_mask = SYS_STATUS_RXFCG_BIT_MASK;
        dwt_writetodevice(SYS_STATUS_ID, 0, 4, (uint8_t*)&clear_mask);
    } else if (sys_stat & SYS_STATUS_RXFTO_BIT_MASK) {
        // RX frame timeout
        ctx->phy_irq_event = DW3000_IRQ_FRAME_WAIT_TIMEOUT;
        free_phy_sem = 1;
        // Clear RX timeout interrupt
        uint32_t clear_mask = SYS_STATUS_RXFTO_BIT_MASK;
        dwt_writetodevice(SYS_STATUS_ID, 0, 4, (uint8_t*)&clear_mask);
    } else if (sys_stat & SYS_STATUS_RXPTO_BIT_MASK) {
        // RX preamble timeout
        ctx->phy_irq_event = DW3000_IRQ_PREAMBLE_DETECT_TIMEOUT;
        free_phy_sem = 1;
        // Clear preamble timeout interrupt
        uint32_t clear_mask = SYS_STATUS_RXPTO_BIT_MASK;
        dwt_writetodevice(SYS_STATUS_ID, 0, 4, (uint8_t*)&clear_mask);
    } else if (sys_stat & SYS_STATUS_HPDWARN_BIT_MASK) {
        // Half period delay warning
        ctx->phy_irq_event = DW3000_IRQ_HALF_DELAY_WARNING;
        free_phy_sem = 1;
        // Clear half delay warning interrupt
        uint32_t clear_mask = SYS_STATUS_HPDWARN_BIT_MASK;
        dwt_writetodevice(SYS_STATUS_ID, 0, 4, (uint8_t*)&clear_mask);
    } else if (sys_stat & (SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK |
                          SYS_STATUS_RXPHE_BIT_MASK | SYS_STATUS_CIAERR_BIT_MASK)) {
        // RX errors
        ctx->phy_irq_event = DW3000_IRQ_ERR;
        free_phy_sem = 1;
        // Clear error interrupts
        uint32_t error_mask = SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK |
                             SYS_STATUS_RXPHE_BIT_MASK | SYS_STATUS_CIAERR_BIT_MASK;
        dwt_writetodevice(SYS_STATUS_ID, 0, 4, (uint8_t*)&error_mask);
    } else {
        // Unknown interrupt - clear all status bits
        dwt_writetodevice(SYS_STATUS_ID, 0, 4, (uint8_t*)&sys_stat);
    }

    k_sem_give(&ctx->dev_lock);

    // Signal the PHY semaphore if we're in IRQ polling emulation mode
    if (atomic_test_bit(&ctx->state, DW3000_STATE_IRQ_POLLING_EMU) && free_phy_sem) {
        k_sem_give(&ctx->phy_sem);
    }
}

// DW3000 interrupt handler - called by hardware interrupt
static void dw3000_interrupt_handler(void)
{
    // Submit interrupt work to work queue for processing
    k_work_submit(&dw3000_ctx.irq_cb_work);
}

static uwb_irq_state_e dw3000_wait_for_irq(const struct device *dev)
{
    int irq_state = wait_for_phy(dev);

    // Map DW3000 IRQ states to universal UWB IRQ states
    switch (irq_state) {
        case DW3000_IRQ_TX:
            return UWB_IRQ_TX;
        case DW3000_IRQ_RX:
            return UWB_IRQ_RX;
        case DW3000_IRQ_FRAME_WAIT_TIMEOUT:
            return UWB_IRQ_FRAME_WAIT_TIMEOUT;
        case DW3000_IRQ_PREAMBLE_DETECT_TIMEOUT:
            return UWB_IRQ_PREAMBLE_DETECT_TIMEOUT;
        case DW3000_IRQ_ERR:
            return UWB_IRQ_ERR;
        case DW3000_IRQ_HALF_DELAY_WARNING:
            return UWB_IRQ_HALF_DELAY_WARNING;
        case DW3000_IRQ_NONE:
        default:
            return UWB_IRQ_NONE;
    }
}

// ==================== TIMEOUT MANAGEMENT ====================

static void dw3000_setup_frame_timeout(const struct device *dev, uint32_t timeout_us)
{
    dwt_setrxtimeout(timeout_us);
}

static void dw3000_setup_preamble_timeout(const struct device *dev, uint16_t pac_symbols)
{
    dwt_setpreambledetecttimeout(pac_symbols);
}

// ==================== DOUBLE BUFFERING ====================

static void dw3000_enable_double_buffering(const struct device *dev, bool auto_reenable)
{
    dwt_dbl_buff_state_e state = DBL_BUF_STATE_EN;
    dwt_dbl_buff_mode_e mode = auto_reenable ? DBL_BUF_MODE_AUTO : DBL_BUF_MODE_MAN;

    // Use DW3000's direct function to enable hardware double buffering
    ull_setdblrxbuffmode(&dw3000_chip, state, mode);

    // Configure CIA diagnostics for double buffering - set to MID level
    dwt_configciadiag(DW_CIA_DIAG_LOG_MID);

    LOG_INF("DW3000 double buffering enabled: mode=%s",
            auto_reenable ? "AUTO" : "MANUAL");
}

static void dw3000_switch_buffers(const struct device *dev)
{
    // Use DW3000's signal buffer free function which toggles buffers
    ull_signal_rx_buff_free(&dw3000_chip);
}

static void __attribute__((unused)) dw3000_sync_rx_buffer_pointer(const struct device *dev)
{
    // DW3000 handles this automatically
}

static void dw3000_signal_buffer_free(const struct device *dev)
{
    // DW3000 doesn't have explicit buffer free signaling
    // This is handled automatically by buffer switching
}

static void dw3000_align_double_buffering(const struct device *dev)
{
    // Does not need explicit aligning
}

// ==================== FRAME I/O ====================

static void dw3000_setup_tx_frame(const struct device *dev, uint8_t *frame_buffer, uint16_t frame_length)
{
    /* DW3000 UWB radio automatically adds 2-byte CRC/FCS, so we add +2 to the frame length */
    uint16_t tx_length = frame_length + 2;

    dwt_writetxdata(tx_length, frame_buffer, 0);
    dwt_writetxfctrl(tx_length, 0, 0);
}

static void dw3000_read_rx_frame(const struct device *dev, uint8_t *frame_buffer, uint16_t frame_length, uint16_t offset)
{
    struct dw3000_context *ctx = &dw3000_ctx;

    uint32_t rx_buff_addr;
    const char* buffer_name;

    // Use buffer determined in ISR rather than reading RDB_STATUS again
    switch (ctx->current_rx_buffer) {
    case DW3000_BUFFER_ACCESS_1:
        // Buffer 1 has valid data
        rx_buff_addr = RX_BUFFER_1_ID;  // 0x130000UL
        buffer_name = "BUF1";
        break;
    case DW3000_BUFFER_ACCESS_0:
        // Buffer 0 has valid data
        rx_buff_addr = RX_BUFFER_0_ID;  // 0x120000UL
        buffer_name = "BUF0";
        break;
    default:
        // Default/single buffer mode
        rx_buff_addr = RX_BUFFER_0_ID;
        buffer_name = "DEFAULT";
        break;
    }


    // Read from the selected buffer (replicating ull_readrxdata logic)
    // Assume reasonable limits: max frame size ~1024 bytes for 802.15.4
    if ((offset + frame_length) <= 2048) {
        if (offset <= REG_DIRECT_OFFSET_MAX_LEN) {
            // Direct read
            dwt_readfromdevice(rx_buff_addr, offset, frame_length, frame_buffer);
        } else {
            // Indirect read using pointer A registers
            dwt_write_reg(INDIRECT_ADDR_A_ID, (rx_buff_addr >> 16UL));
            dwt_write_reg(ADDR_OFFSET_A_ID, (uint32_t)offset);
            dwt_readfromdevice(INDIRECT_POINTER_A_ID, 0U, frame_length, frame_buffer);
        }
    } else {
        printk("DW3000_RX_FRAME_ERROR: Invalid read parameters (offset=%d + length=%d > 2048)\n",
               offset, frame_length);
    }
}

static uint64_t dw3000_read_rx_timestamp(const struct device *dev, uwb_rx_diagnostics_t *diagnostics)
{
    struct dw3000_context *ctx = &dw3000_ctx;
    uint64_t timestamp;
    uint8_t ts_buffer[5];
    const char* buffer_name;

    // Use buffer determined in ISR rather than reading RDB_STATUS again
    switch (ctx->current_rx_buffer) {
    case DW3000_BUFFER_ACCESS_1:
        // Buffer 1 has valid data - use buffer 1 timestamp
        // Note: BUF1_RX_TIME requires indirect pointer B which was set during double buffer init
        dwt_readfromdevice(INDIRECT_POINTER_B_ID, (uint16_t)(BUF1_RX_TIME - BUF1_RX_FINFO), 5, ts_buffer);
        buffer_name = "BUF1";
        break;
    case DW3000_BUFFER_ACCESS_0:
        // Buffer 0 has valid data - use buffer 0 timestamp
        dwt_readfromdevice(BUF0_RX_TIME, 0U, 5, ts_buffer);
        buffer_name = "BUF0";
        break;
    default:
        // Default/single buffer mode - use standard timestamp register
        dwt_readfromdevice(RX_TIME_0_ID, 0U, 5, ts_buffer);
        buffer_name = "DEFAULT";
        break;
    }

    // Convert 5-byte timestamp to uint64_t
    timestamp = 0;
    for (int i = 4; i >= 0; i--) {
        timestamp = (timestamp << 8) | ts_buffer[i];
    }

    if (diagnostics) {
        dwt_rxdiag_t rx_diag;
        dwt_readdiagnostics(&rx_diag);

        // Map DW3000 diagnostics to universal format
        diagnostics->fp_index = rx_diag.ipatovFpIndex;     // First path index
        diagnostics->fp_ampl1 = rx_diag.ipatovF1;          // F1 amplitude
        diagnostics->fp_ampl2 = rx_diag.ipatovF2;          // F2 amplitude
        diagnostics->fp_ampl3 = rx_diag.ipatovF3;          // F3 amplitude
        diagnostics->rx_pacc = rx_diag.ipatovAccumCount;   // Accumulated symbols
        diagnostics->cir_pwr = rx_diag.ciaDiag1;           // CIA diagnostics
        diagnostics->std_noise = 0;                        // Not available in DW3000
        diagnostics->rx_level = rx_diag.ipatovPower;       // Channel power
        diagnostics->cfo_ppm = rx_diag.xtalOffset;         // Crystal offset
    }

    return timestamp;
}

static uint64_t dw3000_read_tx_timestamp(const struct device *dev)
{
    uint64_t timestamp;
    uint8_t ts_buffer[5];

    dwt_readtxtimestamp(ts_buffer);

    // Convert 5-byte timestamp to uint64_t
    timestamp = 0;
    for (int i = 4; i >= 0; i--) {
        timestamp = (timestamp << 8) | ts_buffer[i];
    }

    return timestamp;
}

// ==================== CIR/DIAGNOSTICS ACCESS ====================

static void dw3000_enable_cir_access(const struct device *dev)
{
    // DW3000 CIR access is always enabled
}

static void dw3000_disable_cir_access(const struct device *dev)
{
    // DW3000 CIR access control not needed
}

static void dw3000_read_cir_memory(const struct device *dev, uint8_t *buffer, uint16_t offset, uint16_t length)
{
    dwt_readaccdata(buffer, length, offset);
}

static void dw3000_read_diagnostics(const struct device *dev, uwb_rx_diagnostics_t *diag)
{
    dwt_rxdiag_t rx_diag;
    dwt_readdiagnostics(&rx_diag);

    // Map DW3000 diagnostics to universal structure
    diag->cir_pwr = rx_diag.ipatovPower;
    diag->rx_pacc = rx_diag.ipatovAccumCount;
    diag->fp_index = rx_diag.ipatovFpIndex;
    diag->fp_ampl1 = (rx_diag.ipatovPeak >> 16) & 0xFFFF; // Peak amplitude
    diag->fp_ampl2 = rx_diag.ipatovPeak & 0xFFFF; // Peak index
    diag->fp_ampl3 = 0; // Not available in DW3000
    diag->std_noise = 0; // Not directly available
    diag->rx_level = 0; // Needs calculation
    diag->cfo_ppm = 0; // Needs calculation from xtal offset
}

static int32_t dw3000_read_carrier_integrator(const struct device *dev)
{
    // DW3000 carrier integrator (simplified)
    return 0; // Would need specific DW3000 register access
}

static int16_t dw3000_read_clock_offset(const struct device *dev)
{
    dwt_rxdiag_t rx_diag;
    dwt_readdiagnostics(&rx_diag);
    return rx_diag.xtalOffset;
}

// ==================== CONFIGURATION ====================

static int dw3000_configure(const struct device *dev, const uwb_config_t *config)
{
    // TODO: Implement uwb_config_t based configuration
    // For now, using default configuration set during initialization
    ARG_UNUSED(dev);
    ARG_UNUSED(config);

    LOG_DBG("DW3000 configure called - using default configuration for now");
    return 0;
}

static void dw3000_set_tx_power(const struct device *dev, uint32_t power)
{
    dwt_settxpower(power);
}

static void __attribute__((unused)) dw3000_set_antenna_delay_rx(const struct device *dev, uint16_t delay)
{
    dwt_setrxantennadelay(delay);
}

static void __attribute__((unused)) dw3000_set_antenna_delay_tx(const struct device *dev, uint16_t delay)
{
    dwt_settxantennadelay(delay);
}

static void __attribute__((unused)) dw3000_configure_sleep(const struct device *dev, uint16_t config)
{
    dwt_configuresleep(config, DWT_PRES_SLEEP | DWT_CONFIG | DWT_TXRX_EN);
}

static void __attribute__((unused)) dw3000_enter_sleep(const struct device *dev)
{
    dwt_entersleep(DWT_DW_IDLE);
}

// ==================== REGISTER ACCESS ====================

static uint32_t dw3000_read_reg_u32(const struct device *dev, uint32_t reg_id, uint16_t offset)
{
    uint32_t value;
    dwt_readfromdevice(reg_id, offset, sizeof(uint32_t), (uint8_t *)&value);
    return sys_le32_to_cpu(value);
}

static uint16_t dw3000_read_reg_u16(const struct device *dev, uint32_t reg_id, uint16_t offset)
{
    uint16_t value;
    dwt_readfromdevice(reg_id, offset, sizeof(uint16_t), (uint8_t *)&value);
    return sys_le16_to_cpu(value);
}

static void dw3000_write_reg_u32(const struct device *dev, uint32_t reg_id, uint16_t offset, uint32_t value)
{
    uint32_t le_value = sys_cpu_to_le32(value);
    dwt_writetodevice(reg_id, offset, sizeof(uint32_t), (uint8_t *)&le_value);
}

static void dw3000_register_read(const struct device *dev, uint32_t reg_id, uint16_t offset, uint16_t length, uint8_t *buffer)
{
    dwt_readfromdevice(reg_id, offset, length, buffer);
}

// ==================== CONFIGURATION FUNCTIONS ====================

static void __attribute__((unused)) dw3000_set_antenna_delay(const struct device *dev, uint16_t tx_delay, uint16_t rx_delay)
{
    // DW3000 antenna delay configuration
    dwt_settxantennadelay(tx_delay);
    dwt_setrxantennadelay(rx_delay);
}

static void dw3000_disable_txrx(const struct device *dev)
{
    // Force transceiver off
    dwt_forcetrxoff();
}

static void dw3000_set_frame_filter(const struct device *dev, uint16_t enable, uint16_t allow_beacon)
{
    if (enable) {
        uint16_t flags = DWT_FF_DATA_EN | DWT_FF_ACK_EN;
        if (allow_beacon) {
            flags |= DWT_FF_BEACON_EN;
        }
        dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, flags);
    } else {
        dwt_configureframefilter(0, 0);
    }
}

static void __attribute__((unused)) dw3000_register_write(const struct device *dev, uint32_t reg_id, uint16_t offset, uint16_t length, const uint8_t *buffer)
{
    dwt_writetodevice(reg_id, offset, length, (uint8_t *)buffer);
}

// ==================== RADIO ABSTRACTION ====================

static const uwb_radio_constants_t* dw3000_get_radio_constants_func(const struct device *dev)
{
    return dw3000_get_radio_constants();
}

// ==================== DWT COMPATIBILITY STUBS ====================

/**
 * @brief Get packet duration in nanoseconds (stub implementation)
 * TODO: Implement proper packet duration calculation for DW3000
 */
uint32_t dwt_get_pkt_duration_ns(const struct device *dev, uint16_t psdu_len)
{
    ARG_UNUSED(dev);

    // Simplified stub calculation - should be refined
    // Approximate: 1000ns per byte (this is very rough)
    uint32_t duration_ns = psdu_len * 1000 + 5000; // 5us overhead

    LOG_DBG("Packet duration stub: %d bytes = %d ns", psdu_len, duration_ns);
    return duration_ns;
}

// Set channel through the abstracted driver API - DW3000 stub for now
static int dw3000_set_channel(const struct device *dev, uint8_t channel)
{
    ARG_UNUSED(dev);
    LOG_ERR("DW3000 set_channel not implemented (requested ch=%u)", channel);
    return -ENOTSUP;
}

// ==================== UWB DRIVER STRUCTURE ====================

// ==================== DEVICE ACCESS MANAGEMENT ====================

static int dw3000_acquire_device(const struct device *dev)
{
    ARG_UNUSED(dev);
    // Take the device lock semaphore
    k_sem_take(&dw3000_ctx.dev_lock, K_FOREVER);
    return 0;
}

static void dw3000_release_device(const struct device *dev)
{
    ARG_UNUSED(dev);
    // Release the device lock semaphore
    k_sem_give(&dw3000_ctx.dev_lock);
}

static void dw3000_get_antenna_delay(const struct device *dev, uwb_antenna_delay_t *delay)
{
    // Get antenna delay from DW3000 hardware
    delay->tx_ant_dly = dwt_gettxantennadelay();
    delay->rx_ant_dly = dwt_getrxantennadelay();
}

static void dw3000_set_antenna_delay_new(const struct device *dev, const uwb_antenna_delay_t *delay)
{
    // Set antenna delay in DW3000 hardware
    dwt_settxantennadelay(delay->tx_ant_dly);
    dwt_setrxantennadelay(delay->rx_ant_dly);
}

static uint16_t dw3000_get_rx_frame_length(const struct device *dev)
{
    // Get frame length from DW3000
    uint8_t rng;
    return dwt_getframelength(&rng);
}

static void dw3000_clear_timeouts(const struct device *dev)
{
    // Clear timeouts in DW3000
    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);
}

static uint8_t dw3000_read_system_state(const struct device *dev)
{
    // Read DW3000 system state from SYS_STATE_LO register at offset 2
    uint8_t state_reg;
    dwt_readfromdevice(SYS_STATE_LO_ID, 2U, 1, &state_reg);
    return state_reg;
}

static const char* dw3000_get_system_state_name(const struct device *dev)
{
    static const char* state_names[] = {
        "IDLE_PLL", "IDLE_RC", "TX_WAIT", "TX", "TX_VERIFY", "RX_WAIT", "RX", "RX_VERIFY"
    };
    uint8_t state = dw3000_read_system_state(dev);
    return (state < 8) ? state_names[state] : "UNKNOWN";
}

static const uwb_driver_t dw3000_uwb_driver = {
    .driver_name = "DW3000",
    .driver_data = NULL,

    // Time conversion
    .us_to_timestamp = dw3000_us_to_timestamp,
    .timestamp_to_us = dw3000_timestamp_to_us,
    .system_timestamp = dw3000_system_timestamp,

    // Transceiver control
    .enable_rx = dw3000_enable_rx,
    .start_tx = dw3000_start_tx,
    .force_trx_off = dw3000_force_trx_off,
    .wait_for_irq = dw3000_wait_for_irq,
    .set_channel = dw3000_set_channel,

    // Timeout management
    .setup_frame_timeout = dw3000_setup_frame_timeout,
    .setup_preamble_timeout = dw3000_setup_preamble_timeout,
    .clear_timeouts = dw3000_clear_timeouts,

    // Double buffering
    .enable_double_buffering = dw3000_enable_double_buffering,
    .switch_buffers = dw3000_switch_buffers,
    .signal_buffer_free = dw3000_signal_buffer_free,
    .align_double_buffering = dw3000_align_double_buffering,

    // Frame I/O
    .setup_tx_frame = dw3000_setup_tx_frame,
    .read_rx_frame = dw3000_read_rx_frame,
    .read_rx_timestamp = dw3000_read_rx_timestamp,
    .read_tx_timestamp = dw3000_read_tx_timestamp,
    .get_rx_frame_length = dw3000_get_rx_frame_length,

    // CIR/Diagnostics access
    .enable_cir_access = dw3000_enable_cir_access,
    .disable_cir_access = dw3000_disable_cir_access,
    .read_cir_data = dw3000_read_cir_memory,
    .read_diagnostics = dw3000_read_diagnostics,
    .read_carrier_integrator = dw3000_read_carrier_integrator,
    .read_clock_offset = dw3000_read_clock_offset,

    // Configuration
    .configure = dw3000_configure,
    .set_tx_power = dw3000_set_tx_power,
    .disable_txrx = dw3000_disable_txrx,
    .set_frame_filter = dw3000_set_frame_filter,

    // Register access
    .read_reg_u32 = dw3000_read_reg_u32,
    .read_reg_u16 = dw3000_read_reg_u16,
    .write_reg_u32 = dw3000_write_reg_u32,
    .register_read = dw3000_register_read,
    .read_system_state = dw3000_read_system_state,
    .get_system_state_name = dw3000_get_system_state_name,

    // Device access management
    .acquire_device = dw3000_acquire_device,
    .release_device = dw3000_release_device,
    .get_antenna_delay = dw3000_get_antenna_delay,
    .set_antenna_delay = dw3000_set_antenna_delay_new,

    // Radio abstraction
    .get_radio_constants = dw3000_get_radio_constants_func,
};

// ==================== INITIALIZATION ====================

int uwb_driver_dw3000_init(const struct device *dev)
{
    int ret;

    LOG_INF("Initializing DW3000 UWB driver");

    // Initialize hardware (GPIO interrupts, reset, wakeup)
    LOG_DBG("Calling dw3000_hw_init()");
    ret = dw3000_hw_init();
    if (ret < 0) {
        LOG_ERR("DW3000 hardware initialization failed: %d", ret);
        return ret;
    }

    // Initialize SPI interface
    LOG_DBG("Calling dw3000_spi_init()");
    ret = dw3000_spi_init();
    if (ret < 0) {
        LOG_ERR("DW3000 SPI initialization failed: %d", ret);
        return ret;
    }

    // Initialize GPIO interrupts
    LOG_DBG("Calling dw3000_hw_init_interrupt()");
    ret = dw3000_hw_init_interrupt();
    if (ret < 0) {
        LOG_ERR("DW3000 interrupt initialization failed: %d", ret);
        return ret;
    }

    // Reset the DW3000 chip
    LOG_DBG("Resetting DW3000 chip");
    dw3000_hw_reset();

    // Wait for chip to stabilize after reset
    LOG_DBG("Waiting 5ms for chip stabilization");
    k_sleep(K_MSEC(5));

    // Setup dwchip_t structure directly (avoiding probe indirection)
    LOG_DBG("Setting up DW3000 chip structure");
    dw3000_chip.SPI = (struct dwt_spi_s*)(void*)&dw3000_spi_fct;
    dw3000_chip.wakeup_device_with_io = dw3000_hw_wakeup;
    dw3000_chip.dwt_driver = (struct dwt_driver_s*)&dw3000_driver;

    // Wake up device
    LOG_DBG("Waking up device");
    dw3000_hw_wakeup();

    // Check device ID directly - COMMENTED OUT FOR TESTING
    // LOG_DBG("Checking DW3000 device ID");
    // uint32_t dev_id;
    // dwt_readfromdevice(DEV_ID_ID, 0, 4, (uint8_t*)&dev_id);
    //
    // // Expected DW3000 device ID (from dw3000_driver definition)
    // uint32_t expected_id = 0xDECA0300UL;  // DW3000 device ID
    // uint32_t id_mask = 0xFFFFFF00UL;      // Mask for device ID check
    //
    // if ((expected_id & id_mask) != (dev_id & id_mask)) {
    //     LOG_ERR("Invalid DW3000 device ID: expected 0x%08x, got 0x%08x",
    //             expected_id & id_mask, dev_id & id_mask);
    //     return -ENODEV;
    // }
    //
    // LOG_DBG("DW3000 device ID verified: 0x%08x", dev_id);
    LOG_DBG("Device ID check temporarily disabled for testing");

    // Update global dw pointer for deca_compat layer
    LOG_DBG("Updating global dw pointer");
    dwt_update_dw(&dw3000_chip);

    // Initialize the DW3000 driver layer with OTP reading
    LOG_DBG("Initializing DW3000 driver layer");
    ret = dwt_initialise(DWT_READ_OTP_PID); // Read part ID from OTP
    if (ret != DWT_SUCCESS) {
        LOG_ERR("DW3000 initialization failed: %d", ret);
        return -ENODEV;
    }

    // Configure DW3000 with default settings to enable system time counter
    // The system time counter only runs when device is in IDLE_PLL state (after dwt_configure)
    LOG_DBG("Configuring DW3000 with default settings to enable system time counter");
    dwt_config_t default_config = {
        .chan = 5,                          // Channel 5
        .txPreambLength = DWT_PLEN_128,     // 128-symbol preamble
        .rxPAC = DWT_PAC8,                  // PAC size 8
        .txCode = 10,                       // TX preamble code 10 (matches DW1000 64MHz PRF)
        .rxCode = 10,                       // RX preamble code 10 (matches DW1000 64MHz PRF)
        .sfdType = DWT_SFD_IEEE_4A,         // Standard IEEE 802.15.4a SFD
        .dataRate = DWT_BR_6M8,             // 6.8 Mbps data rate
        .phrMode = DWT_PHRMODE_EXT,         // Extended PHR mode (non-compliant, matches DW1000)
        .phrRate = DWT_PHRRATE_STD,         // Standard PHR rate
        .sfdTO = 128 + 1 + 8 - 8,          // SFD timeout (preamble + SFD + PHR - margin)
        .stsMode = DWT_STS_MODE_OFF,        // STS mode off
        .stsLength = DWT_STS_LEN_64,        // STS length 64
        .pdoaMode = DWT_PDOA_M0             // PDOA mode 0 (off)
    };

    ret = dwt_configure(&default_config);
    if (ret != DWT_SUCCESS) {
        LOG_ERR("DW3000 default configuration failed: %d", ret);
        return -ENODEV;
    }
    LOG_DBG("DW3000 configured with default settings - system time counter enabled");

    // Enable essential interrupts after hardware initialization (like DW1000 does)
    // This ensures interrupts are enabled regardless of when/if configure() is called
    LOG_DBG("Enabling essential TX/RX interrupts");
    uint32_t interrupt_mask = DWT_INT_TXFRS_BIT_MASK |        // TX frame sent
                             DWT_INT_RXFCG_BIT_MASK |        // RX frame CRC good
                             DWT_INT_RXFTO_BIT_MASK |        // RX frame wait timeout
                             DWT_INT_RXPTO_BIT_MASK |        // RX preamble timeout
                             DWT_INT_HPDWARN_BIT_MASK;       // Half period delay warning

    dwt_setinterrupt(interrupt_mask, 0, DWT_ENABLE_INT_ONLY);


    // Read and verify device ID
    uint32_t verified_dev_id = dwt_readdevid();
    LOG_INF("DW3000 Device ID: 0x%08x", verified_dev_id);

    // Verify it's a valid DW3000/DW3100/DW3200/DW3300 device ID
    // DW3000: 0xDECA0302, DW3100: 0xDECA0312, etc.
    if ((verified_dev_id & 0xFFFF0000) != 0xDECA0000) {
        LOG_ERR("Invalid DW3000 device ID: 0x%08x", verified_dev_id);
        return -ENODEV;
    }

    // Initialize interrupt handling
    LOG_DBG("Initializing DW3000 interrupt handling");
    k_work_init(&dw3000_ctx.irq_cb_work, dw3000_irq_work_handler);

    // Set the custom interrupt handler in the hardware layer
    dw3000_hw_set_interrupt_handler(dw3000_interrupt_handler);

    // Enable GPIO interrupt
    LOG_DBG("Enabling GPIO interrupt");
    dw3000_hw_interrupt_enable();

    // Enable IRQ polling emulation mode for ranging protocol
    atomic_set_bit(&dw3000_ctx.state, DW3000_STATE_IRQ_POLLING_EMU);

    // Enable double buffering for proper frame reception (critical for DW3000)
    LOG_DBG("Enabling DW3000 double buffering");
    dw3000_enable_double_buffering(dev, false);

    LOG_INF("DW3000 chip detected and verified successfully");
    LOG_DBG("Registering DW3000 UWB driver abstraction");
    ret = uwb_driver_register(dev, &dw3000_uwb_driver);
    if (ret < 0) {
        LOG_ERR("DW3000 UWB driver registration failed: %d", ret);
        return ret;
    }
    return ret;
}
