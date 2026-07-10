/**
 * @file muloc.h
 * @brief MULoc (anchor overhearing) block configuration and result types
 *
 * MULoc is an N-party round-robin UWB protocol where anchors broadcast
 * sequentially, embedding CIR/phase/timestamp data from overheard
 * transmissions. Tags passively collect all anchor measurements.
 * Raw data is exported for offline processing (no on-device ranging).
 */

#ifndef MULOC_BLOCK_H
#define MULOC_BLOCK_H

#include <stdint.h>

#define MULOC_MAX_ANCHORS 8
#define MULOC_MAX_ROUNDS  8

/**
 * @brief Per-reception anchor overhearing record (13 bytes)
 *
 * Matches the MULoc wire format: CIR first-path complex sample,
 * RCPHASE correction, preamble accumulation count, max CIR gain,
 * and 40-bit RX timestamp.
 */
struct muloc_ao_record {
	uint16_t cir_re;          /**< CIR first-path real (unsigned) */
	uint16_t cir_im;          /**< CIR first-path imaginary (unsigned) */
	uint8_t  rcphase;         /**< RCPHASE correction */
	uint8_t  rx_pacc;         /**< Preamble accumulation count */
	uint16_t max_growth_cir;  /**< Max CIR gain for RSSI calculation */
	uint8_t  rx_ts[5];        /**< 40-bit RX timestamp (little-endian) */
};

/**
 * @brief Tag overhearing record (AO + carrier integrator)
 */
struct muloc_to_record {
	struct muloc_ao_record ao;     /**< Base AO fields */
	int32_t carrier_integrator;    /**< CFO for clock drift estimation */
};

/**
 * @brief Configuration for the MULoc ranging driver
 */
struct muloc_ranging_config {
	uint8_t  anchor_count;     /**< Number of anchors in the system (2-8) */
	uint8_t  anchor_id;        /**< This node's anchor ID (0..N-1), 0xFF = tag/listener */
	uint8_t  num_rounds;       /**< Rounds per block execution (1-8) */
	uint16_t delay_time_us;    /**< Inter-anchor TX delay in us (default 600) */
	uint16_t turn_delay_us;    /**< Delay between rounds in us (default 600) */
	uint16_t rx_timeout_us;    /**< RX timeout per slot in us (default 2500) */
	uint64_t round_start_ts;   /**< DWT timestamp for first round start */
	uint8_t  start_channel;    /**< Initial UWB channel */
	uint8_t  hop_channel;      /**< Frequency hop target channel */
};

/**
 * @brief Result of one MULoc round
 */
struct muloc_round_result {
	uint8_t round_index;
	uint8_t channel;                                    /**< UWB channel used */
	uint8_t anchor_count;
	struct muloc_ao_record ao_records[MULOC_MAX_ANCHORS]; /**< Anchor mode: AO data */
	uint8_t ao_valid_mask;                                /**< Bitmask of valid AO records */
	struct muloc_to_record to_records[MULOC_MAX_ANCHORS]; /**< Tag mode: TO data */
	uint8_t to_valid_mask;                                /**< Bitmask of valid TO records */
	uint8_t frame_seq;
};

/**
 * @brief MULoc block result passed via callback
 */
struct muloc_block_result {
	struct muloc_round_result *rounds;
	uint8_t round_count;
	uint8_t anchor_id;
	uint64_t rtc;
	uint8_t start_channel;
};

typedef enum {
	MULOC_STATUS_SUCCESS = 0,
	MULOC_STATUS_ERROR = 1,
} muloc_status_t;

typedef void (*muloc_cb_t)(muloc_status_t status,
			   const struct muloc_block_result *result,
			   void *user_data);

/**
 * @brief Runtime configuration for the MULoc block handler
 */
struct muloc_block_config {
	uint8_t  anchor_count;
	uint8_t  anchor_id;        /**< 0xFF = listener/tag mode */
	uint8_t  num_rounds;
	uint16_t delay_time_us;
	uint8_t  start_channel;
	uint8_t  hop_channel;

	muloc_cb_t muloc_cb;
	void *cb_user_data;
};

/**
 * @brief Low-level MULoc ranging driver entry point
 *
 * Executes num_rounds of the MULoc round-robin protocol, storing
 * raw AO/TO measurement data in the results array.
 *
 * @param dev UWB device
 * @param conf Ranging configuration
 * @param results Array to store round results
 * @param max_results Size of results array
 * @return Number of completed rounds on success, negative errno on error
 */
int deca_muloc_ranging(const struct device *dev,
		       struct muloc_ranging_config *conf,
		       struct muloc_round_result *results,
		       size_t max_results);

#endif /* MULOC_BLOCK_H */
