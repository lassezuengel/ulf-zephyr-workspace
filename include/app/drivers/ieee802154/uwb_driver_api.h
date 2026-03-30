#ifndef UWB_DRIVER_API_H
#define UWB_DRIVER_API_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

// IRQ state enumeration compatible with DW1000
typedef enum {
    UWB_IRQ_NONE = 0,
    UWB_IRQ_RX = 1,
    UWB_IRQ_TX = 2,
    UWB_IRQ_FRAME_WAIT_TIMEOUT = 3,
    UWB_IRQ_PREAMBLE_DETECT_TIMEOUT = 4,
    UWB_IRQ_ERR = 5,
    UWB_IRQ_HALF_DELAY_WARNING = 6
} uwb_irq_state_e;

// IRQ state to string conversion utility
static const char* irq_state_names[] = {
    "NONE",                     // 0
    "RX",                       // 1
    "TX",                       // 2
    "FRAME_WAIT_TIMEOUT",       // 3
    "PREAMBLE_DETECT_TIMEOUT",  // 4
    "ERR",                      // 5
    "HALF_DELAY_WARNING"        // 6
};

static inline const char* irq_state_to_string(uwb_irq_state_e state) {
    if (state >= 0 && state < ARRAY_SIZE(irq_state_names)) {
        return irq_state_names[state];
    }
    return "UNKNOWN";
}

// Universal diagnostics structure
typedef struct {
    uint32_t cir_pwr;
    uint32_t rx_pacc;
    uint16_t fp_index;
    uint16_t fp_ampl1;
    uint16_t fp_ampl2;
    uint16_t fp_ampl3;
    uint16_t std_noise;
    int8_t rx_level;
    float cfo_ppm;
    uint8_t rx_phase;
} uwb_rx_diagnostics_t;

// UWB API enums for abstraction across different drivers
// These use meaningful values rather than driver-specific indices
typedef enum {
    UWB_PRF_16MHZ = 16,     // Maps to DWT_PRF_16M (0) for DW1000
    UWB_PRF_64MHZ = 64      // Maps to DWT_PRF_64M (1) for DW1000
} uwb_prf_e;

typedef enum {
    UWB_DATARATE_110K = 110,    // Maps to DWT_BR_110K (0) for DW1000
    UWB_DATARATE_850K = 850,    // Maps to DWT_BR_850K (1) for DW1000
    UWB_DATARATE_6M8 = 6800     // Maps to DWT_BR_6M8 (2) for DW1000
} uwb_datarate_e;

typedef enum {
    UWB_PREAMBLE_64 = 64,       // Maps to DWT_PLEN_64 (0) for DW1000
    UWB_PREAMBLE_128 = 128,     // Maps to DWT_PLEN_128 (1) for DW1000
    UWB_PREAMBLE_256 = 256,     // Maps to DWT_PLEN_256 (2) for DW1000
    UWB_PREAMBLE_512 = 512,     // Maps to DWT_PLEN_512 (3) for DW1000
    UWB_PREAMBLE_1024 = 1024,   // Maps to DWT_PLEN_1024 (4) for DW1000
    UWB_PREAMBLE_2048 = 2048,   // Maps to DWT_PLEN_2048 (5) for DW1000
    UWB_PREAMBLE_4096 = 4096    // Maps to DWT_PLEN_4096 (6) for DW1000
} uwb_preamble_length_e;

typedef enum {
    UWB_PAC_8 = 8,      // Maps to DWT_PAC8 (0) for DW1000
    UWB_PAC_16 = 16,    // Maps to DWT_PAC16 (1) for DW1000
    UWB_PAC_32 = 32,    // Maps to DWT_PAC32 (2) for DW1000
    UWB_PAC_64 = 64     // Maps to DWT_PAC64 (3) for DW1000
} uwb_pac_size_e;

// Universal configuration structure with proper enums
typedef struct {
    uint8_t channel;                        // UWB channel (1-7, 9-11 for DW1000)
    uwb_prf_e prf;                         // Pulse Repetition Frequency
    uwb_datarate_e datarate;               // Data rate
    uwb_preamble_length_e preamble_length; // Preamble length in symbols
    uwb_pac_size_e pac_size;               // Preamble Acquisition Chunk size
    uint8_t sfd_type;                      // Start Frame Delimiter type (driver-specific)
    uint16_t sfd_timeout;                  // SFD timeout in symbols
    bool smart_power;                      // Smart power control enable
} uwb_config_t;

// Radio-specific constants structure for abstraction
typedef struct {
    uint64_t timestamp_mask;
    float cfo_conversion_factor;
    float timestamp_resolution_ps;
    struct {
        float prf16;
        float prf64;
    } rssi_constants;
} uwb_radio_constants_t;

// Forward declaration
struct uwb_driver;

// Radio-agnostic constants
#define UWB_MTM_MAX_FRAMES CONFIG_UWB_MTM_MAX_FRAMES
#define FRAME_LENGTH_ADDITIONAL 2  // CRC length typically

// Hardware-agnostic UWB types (formerly DW1000-specific)
typedef uint16_t deca_short_addr_t;
typedef uint8_t uwb_packed_ts_t[5];
typedef uint64_t uwb_ts_t;

#define DECA_NO_ADDRESS UINT16_MAX
#define DECA_RANGING_FRAME_MAX_FRAME_SIZE 250

// Hardware-agnostic ranging frame structure
struct __attribute__((__packed__)) deca_ranging_frame  {
    uint8_t  msg_id;      // identifier of which message type during the protocol run we are sending
    uint16_t checksum;    // checksum for frame integrity verification
    deca_short_addr_t addr;  // unique identifier of this node for ranging
    uwb_packed_ts_t tx_ts;
    uint8_t  rx_ts_count; // amount of received timestamps
    uint8_t  payload_size;
    uint8_t payload[250]; // payload will be located AFTER reception timestamps
};

// Frame type enumeration
enum deca_frame_type {
    DECA_TRANSMITTED = 0,
    DECA_RECEIVED = 1,
};

enum deca_ranging_frame_status {
    DECA_FRAME_OKAY,
    DECA_FRAME_REJECTED
};

// Hardware-agnostic ranging digest structures
struct deca_ranging_frame_container {
    struct deca_ranging_frame *frame;

    enum deca_frame_type type;
    enum deca_ranging_frame_status status;

    uint32_t rx_pacc;
    uint32_t cir_pwr;
    uint16_t fp_index, fp_ampl1, fp_ampl2, fp_ampl3, std_noise;
    uint8_t slot; // might be unecessary since we currently index the return array by the respective slot number
    float cfo_ppm;
    int8_t rx_level;
    int16_t fp_re, fp_im;
    uint8_t rx_phase; // RCPHASE value for phase correction

    uwb_ts_t timestamp;
};

struct deca_ranging_digest {
	struct deca_ranging_frame_container *frames;
	size_t length;    /* Number of frames currently used */
	size_t capacity;  /* Total number of frames allocated */
};

// MM reference configuration
struct deca_mm_reference_config {
    deca_short_addr_t addr;
    bool isInitiator;
    uint64_t deca_round_start_ts; // only relevant if time sync instant is used
    uint32_t respond_interval_us, guard_period_us;
    uint16_t timeout_us;
};

// Forward declaration for slots
struct deca_slot;

struct deca_schedule {
	uint16_t slot_count;
	struct deca_slot *slots;
};

struct deca_glossy_time_pair {
	uint64_t local, ref;
};

// Forward declarations
typedef int (*cir_memory_callback_t)(int slot, const uint8_t *cir_memory, size_t size);

// Constants from dw1000.h
#define DECA_SHORT_ADDR_MAX UINT16_MAX
#define DWT_TS_TO_US(X) (((X)*15650)/1000000000)
#define UUS_TO_DWT_TS(X) (((uint64_t)X)*(uint64_t)65536)
#define US_TO_DWT_TS(X) (((uint64_t)X)*(uint64_t)63875)
#define NS_TO_DWT_TS(ns) ((((uint64_t)ns*1000*1000)/15650))
#define DWT_MTM_MAX_PAYLOAD 250
#define DWT_RANGING_FRAME_PAYLOAD_OFFSET(FRAME) (FRAME->payload + sizeof(struct deca_tagged_timestamp) * FRAME->rx_ts_count)

// Function declarations
uint64_t correct_overflow(uwb_ts_t end_ts, uwb_ts_t start_ts);

struct deca_ranging_configuration {
	deca_short_addr_t addr;

	struct deca_schedule *schedule;
	struct deca_glossy_time_pair *deca_clock_synchronization_instance;
	uint64_t round_start_offset_us; // only relevant if time sync instant is used
	uint64_t deca_round_start_ts; // only relevant if time sync instant is used

	uint32_t slot_duration_us, guard_period_us;
	uint64_t micro_slot_offset_ns;

	// options
	uint8_t cca, reject_frames, cfo, correct_timestamp_bias;

	cir_memory_callback_t cir_handler;

	uint16_t fp_index_threshold; // if reject frames is set, remove frames below this threshold
	uint16_t timeout_us;
	uint16_t cca_duration;
};

struct deca_glossy_configuration {
	deca_short_addr_t node_addr;
	bool isRoot;
	uint16_t guard_period_us;
	uint16_t max_depth;
	uint8_t *payload;
	size_t payload_size;
	uint16_t transmission_delay_us;
};

struct __attribute__((__packed__)) deca_tagged_timestamp {
	uwb_packed_ts_t ts;
	deca_short_addr_t addr;
	uint16_t slot;
};

struct __attribute__((__packed__)) deca_tagged_mm_timestamp {
    uwb_packed_ts_t ts;
    int16_t im, re;
    deca_short_addr_t addr;
    uint16_t slot;
};

struct deca_glossy_result {
	struct deca_glossy_time_pair rtc_clock_pair;
	struct deca_glossy_time_pair deca_clock_pair;
	uint16_t root_node_id; // Address of the root node that initiated this glossy round
	uint8_t dist_to_root; // aka hop counter
	size_t payload_size;
	uint8_t *payload;
	int32_t measured_constant_delay_us; // Measured constant delay on root node, -1 if not measured
};

enum slot_type {
	DENSE_LOAD_TX_BUFFER,
	DENSE_RX_SLOT,
	DENSE_TX_SLOT,
	DENSE_IDLE_SLOT,
};

struct deca_slot {
	enum slot_type type;

	/* use dwt_calculate_slot_duration to calculate this duration */
	uint16_t duration_us;

	union {
		// Meta information for LOAD_TX_BUFFER
		struct {
			uint8_t *payload;
			size_t payload_size;

			bool load_stored_timestamps;

			/* use this this together with duration_us. If you include a payload in this
			   transmission slot, you should not max out the slot duration to just fit
			   the expected amount of timestamps collected, rather you should also leave
			   some headroom for the payload by decreasing the amount of timestamps to
			   include in the frame. */
			int max_load_timestamps;
		};

		// Meta information for rx
		struct {
			/* for most boards running this code it will not feasible to buffer CIRs for all slots, thus we
			   allow here to active the globally configured CIR handler to process the data directly */
			bool with_cir_handler;
			uint16_t from_index, to_index;
			bool only_first_path; // only read first path sample
		};
	} meta;
};

// Function prototypes
uwb_ts_t from_packed_dwt_ts(const uwb_packed_ts_t ts);
void to_packed_dwt_ts(uwb_packed_ts_t ts, uwb_ts_t value);

int dwt_calculate_slot_duration(const struct device *dev, int timestamps_to_load, int payload_size, int guard_us);
uint32_t dwt_get_pkt_duration_ns(const struct device *dev, uint16_t psdu_len);

int deca_ranging_frame_get_tagged_timestamps(const struct deca_ranging_frame *frame,
					     struct deca_tagged_timestamp **timestamps);
int deca_ranging_frame_get_mm_tagged_timestamps(const struct deca_ranging_frame *frame, struct deca_tagged_mm_timestamp **timestamps);
int      dwt_mtm_ranging_estimate_duration(const struct device *dev, const struct deca_ranging_configuration *conf);
int deca_ranging(const struct device *dev, const struct deca_ranging_configuration *conf,
		 struct deca_ranging_digest *digest);
int      deca_ranging_mm(const struct device *dev, const struct deca_ranging_configuration *conf, struct deca_ranging_digest *digest);
int      deca_mm_reference(const struct device *dev, struct deca_mm_reference_config *conf, struct deca_ranging_digest *digest);
int      deca_glossy_time_synchronization(const struct  device *dev, struct deca_glossy_configuration *conf, struct deca_glossy_result *result);

// Radio-agnostic message IDs for ranging protocols
// UWB_MTM_RANGING_FRAME_ID is defined in uwb_timestamp_utils.h as 0x01
#define UWB_MTM_REF_POLL                    0x61
#define UWB_MTM_REF_RESPONSE               0x62
#define UWB_MTM_REF_FINAL                  0x63
#define UWB_MTM_REF_POST_FINAL            0x64
#define UWB_MTM_REF_MEASUREMENT_EXCHANGE   0x65
#define UWB_MTM_REF_MEASUREMENT_EXCHANGE_ACK 0x66

// Forward declaration for antenna delay structure
typedef struct {
    uint16_t tx_ant_dly;
    uint16_t rx_ant_dly;
} uwb_antenna_delay_t;

typedef struct uwb_driver {
    const char *driver_name;
    void *driver_data;

    // ==================== TIME CONVERSION ====================
    uint64_t (*us_to_timestamp)(const struct device *dev, uint32_t us);
    uint32_t (*timestamp_to_us)(const struct device *dev, uint64_t ts);
    uint64_t (*system_timestamp)(const struct device *dev);

    // ==================== TRANSCEIVER CONTROL ====================
    int (*enable_rx)(const struct device *dev, uint32_t timeout_us, uint64_t delayed_timestamp);
    int (*start_tx)(const struct device *dev, uint64_t delayed_timestamp);
    void (*force_trx_off)(const struct device *dev);
    uwb_irq_state_e (*wait_for_irq)(const struct device *dev);
    // Set UWB channel (abstracted). Return 0 on success, negative error on failure.
    int (*set_channel)(const struct device *dev, uint8_t channel);

    // ==================== TIMEOUT MANAGEMENT ====================
    void (*setup_frame_timeout)(const struct device *dev, uint32_t timeout_us);
    void (*setup_preamble_timeout)(const struct device *dev, uint16_t timeout_symbols);
    void (*clear_timeouts)(const struct device *dev);

    // ==================== DOUBLE BUFFERING ====================
    void (*enable_double_buffering)(const struct device *dev, bool auto_reenable);
    void (*switch_buffers)(const struct device *dev);
    void (*signal_buffer_free)(const struct device *dev);
    void (*align_double_buffering)(const struct device *dev);

    // ==================== FRAME I/O ====================
    void (*setup_tx_frame)(const struct device *dev, uint8_t *buffer, uint16_t length);
    void (*read_rx_frame)(const struct device *dev, uint8_t *buffer, uint16_t length, uint16_t offset);
    uint64_t (*read_rx_timestamp)(const struct device *dev, uwb_rx_diagnostics_t *diag);
    uint64_t (*read_tx_timestamp)(const struct device *dev);
    uint16_t (*get_rx_frame_length)(const struct device *dev);

    // ==================== CIR/DIAGNOSTICS ACCESS ====================
    void (*enable_cir_access)(const struct device *dev);
    void (*disable_cir_access)(const struct device *dev);
    void (*read_cir_data)(const struct device *dev, uint8_t *buffer, uint16_t offset, uint16_t length);
    void (*read_diagnostics)(const struct device *dev, uwb_rx_diagnostics_t *diag);
    int32_t (*read_carrier_integrator)(const struct device *dev);
    int16_t (*read_clock_offset)(const struct device *dev);

    // ==================== CONFIGURATION ====================
    int (*configure)(const struct device *dev, const uwb_config_t *config);
    int (*get_config)(const struct device *dev, uwb_config_t *config);
    void (*set_tx_power)(const struct device *dev, uint32_t power);
    void (*disable_txrx)(const struct device *dev);
    void (*set_frame_filter)(const struct device *dev, uint16_t enable, uint16_t allow_beacon);

    // ==================== RANGE BIAS CORRECTION ====================
    double (*get_range_bias)(const struct device *dev, uint8_t channel, float range, uwb_prf_e prf);

    // ==================== REGISTER ACCESS ====================
    uint32_t (*read_reg_u32)(const struct device *dev, uint32_t reg_id, uint16_t offset);
    uint16_t (*read_reg_u16)(const struct device *dev, uint32_t reg_id, uint16_t offset);
    void (*write_reg_u32)(const struct device *dev, uint32_t reg_id, uint16_t offset, uint32_t value);
    void (*register_read)(const struct device *dev, uint32_t reg_id, uint16_t offset, uint16_t length, uint8_t *buffer);
    uint8_t (*read_system_state)(const struct device *dev);
    const char* (*get_system_state_name)(const struct device *dev);

    // ==================== DEVICE ACCESS MANAGEMENT ====================
    int (*acquire_device)(const struct device *dev);
    void (*release_device)(const struct device *dev);
    void (*get_antenna_delay)(const struct device *dev, uwb_antenna_delay_t *delay);
    void (*set_antenna_delay)(const struct device *dev, const uwb_antenna_delay_t *delay);

    // ==================== RADIO ABSTRACTION ====================
    const uwb_radio_constants_t* (*get_radio_constants)(const struct device *dev);

} uwb_driver_t;

// Driver registration functions
int uwb_driver_register(const struct device *dev, const uwb_driver_t *driver);
const uwb_driver_t *uwb_driver_get(const struct device *dev);

// Helper macros for backward compatibility
#define US_TO_UWB_TS(dev, us) uwb_driver_get(dev)->us_to_timestamp(dev, us)
#define UWB_TS_TO_US(dev, ts) uwb_driver_get(dev)->timestamp_to_us(dev, ts)
#define UWB_SYSTEM_TS(dev) uwb_driver_get(dev)->system_timestamp(dev)

#ifdef __cplusplus
}
#endif

#endif /* UWB_DRIVER_API_H */
