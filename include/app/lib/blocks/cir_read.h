/**
 * @file cir_read.h
 * @brief CIR (Channel Impulse Response) read block configuration and result types
 *
 * CIR capture block: configured as either sender or receiver per node.
 * Senders transmit a probe frame at a synchronized time, receivers listen
 * and read the DW1000 accumulator memory. Timing is derived from glossy
 * time synchronization. CIR data is transferred over BLE asynchronously.
 */

#ifndef CIR_READ_BLOCK_H
#define CIR_READ_BLOCK_H

#include <stdint.h>
#include <stdbool.h>

/** Maximum CIR samples (DW1000 Ipatov accumulator, PRF64) */
#define CIR_READ_MAX_SAMPLES 1016

/** Frame ID for the CIR probe message */
#define CIR_PROBE_FRAME_ID 0x80

/** CIR block mode */
typedef enum {
	CIR_MODE_SENDER = 0,
	CIR_MODE_RECEIVER = 1,
} cir_read_mode_t;

/**
 * @brief Result of a CIR read operation
 */
struct cir_read_block_result {
	uint8_t *cir_data;          /**< CIR sample buffer (int16 re,im pairs, 4 bytes/sample) */
	uint16_t sample_count;      /**< Number of complex samples read */
	uint16_t first_path_index;  /**< First path sample index from diagnostics */
	uint16_t from_index;        /**< Actual absolute start index used */
	uint8_t  channel;
};

typedef enum {
	CIR_READ_STATUS_SUCCESS = 0,
	CIR_READ_STATUS_ERROR = 1,
} cir_read_status_t;

typedef void (*cir_read_cb_t)(cir_read_status_t status,
			      const struct cir_read_block_result *result,
			      void *user_data);

/**
 * @brief Runtime configuration for the CIR read block handler
 */
struct cir_read_block_config {
	uint8_t  mode;              /**< cir_read_mode_t: 0=sender, 1=receiver */
	uint8_t  channel;           /**< UWB channel (0 = keep current) */
	uint16_t tx_delay_dtu;      /**< Sender: delayed TX offset in short_ts units (~4ns/step) */
	uint16_t from_index;        /**< Receiver: start sample index (or offset before FP) */
	uint16_t to_index;          /**< Receiver: end sample index (or offset after FP) */
	uint8_t  only_first_path;   /**< Receiver: if true, from/to relative to fp_index */

	cir_read_cb_t cir_cb;
	void *cb_user_data;
};

/**
 * @brief CIR read block handler (called by scheduler)
 *
 * Sender: transmits probe at synchronized time + setup guard + tx_delay_dtu.
 * Receiver: listens at synchronized time + setup guard, reads CIR accumulator.
 */
void cir_read_block_handler(uint64_t event_time, void *user_data);

#endif /* CIR_READ_BLOCK_H */
