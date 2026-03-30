/**
 * @file connector.h
 * @brief Crazyflie SPI Slave Connector API
 *
 * This module provides an interface for sending ranging measurements from the
 * UWB deck to the Crazyflie STM32 via SPI slave interface.
 *
 * Architecture:
 * - Callers build up measurements using crazyflie_measurements_add_*() functions
 * - send_measurement_to_crazyflie() queues measurements for transmission (non-blocking)
 * - A dedicated background thread handles the actual SPI communication
 * - The thread blocks waiting for the Crazyflie master to initiate transfers
 *
 * Usage:
 *   crazyflie_measurements_t *m;
 *   crazyflie_measurements_alloc(10, &m);
 *   crazyflie_measurements_add_tof(addr, tof, m);
 *   crazyflie_measurements_add_position(addr, pos, m);
 *   send_measurement_to_crazyflie(m, K_NO_WAIT);  // Non-blocking, queues data
 *   crazyflie_measurements_free(m);
 */

#ifndef SYNCHROFLY_CRAZYFLIE_CONNECTOR_H
#define SYNCHROFLY_CRAZYFLIE_CONNECTOR_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <app/lib/ranging/twr.h>

/**
 * @brief Opaque measurement container type
 */
typedef struct crazyflie_measurements crazyflie_measurements_t;

/**
 * @brief Allocate a measurement container
 *
 * @param capacity Maximum number of measurements (clamped to internal max)
 * @param measurements Pointer to receive allocated container
 * @return Actual capacity on success, negative error code on failure
 *
 * @note Uses static allocation internally - only one container can be in use at a time
 */
int crazyflie_measurements_alloc(int capacity, crazyflie_measurements_t **measurements);

/**
 * @brief Free a measurement container
 *
 * @param measurements Container to free
 * @return 0 on success
 */
int crazyflie_measurements_free(crazyflie_measurements_t *measurements);

/**
 * @brief Get the number of measurements in a container
 *
 * @param measurements Container to query
 * @return Number of measurements
 */
size_t crazyflie_measurements_length(const crazyflie_measurements_t *measurements);

/**
 * @brief Add a ToF (Time-of-Flight) distance measurement
 *
 * @param addr Node address of the ranging responder
 * @param tof Time-of-flight value (converted to distance internally)
 * @param measurements Container to add to
 * @return 0 on success, -ENOMEM if container full
 */
int crazyflie_measurements_add_tof(deca_short_addr_t addr, float tof,
                                   crazyflie_measurements_t *measurements);

/**
 * @brief Add a phase measurement
 *
 * @param addr Node address
 * @param phase Phase value
 * @param measurements Container to add to
 * @return 0 on success, -ENOMEM if container full
 */
int crazyflie_measurements_add_phase(deca_short_addr_t addr, float phase,
                                     crazyflie_measurements_t *measurements);

/**
 * @brief Add a CFO (Carrier Frequency Offset) measurement
 *
 * @param addr Node address
 * @param cfo CFO value in ppm
 * @param measurements Container to add to
 * @return 0 on success, -ENOMEM if container full
 */
int crazyflie_measurements_add_cfo(deca_short_addr_t addr, float cfo,
                                   crazyflie_measurements_t *measurements);

/**
 * @brief Add a position measurement
 *
 * @param addr Node address (typically self)
 * @param position 3D position [x, y, z] in meters
 * @param measurements Container to add to
 * @return 0 on success, -ENOMEM if container full
 */
int crazyflie_measurements_add_position(deca_short_addr_t addr, float position[3],
                                        crazyflie_measurements_t *measurements);

/**
 * @brief Add an mm-accurate dual-channel measurement
 *
 * @param addr Node address
 * @param distance_mm MM-accurate distance in meters
 * @param distance_twr Coarse TWR distance in meters
 * @param confidence Confidence level (0-1)
 * @param phase_chan_5 Channel 5 phase
 * @param phase_chan_3 Channel 3 phase
 * @param channels Channel bitmask
 * @param measurements Container to add to
 * @return 0 on success, -ENOMEM if container full
 */
int crazyflie_measurements_add_mm_accurate(deca_short_addr_t addr,
                                           float distance_mm, float distance_twr,
                                           float confidence, float phase_chan_5,
                                           float phase_chan_3, uint8_t channels,
                                           crazyflie_measurements_t *measurements);

/**
 * @brief Add a dual-channel distance measurement
 *
 * @param addr Node address
 * @param distance Distance in meters
 * @param confidence Confidence level (0-1)
 * @param phase_diff Phase difference between channels
 * @param method Measurement method (0=single, 1=dual-channel)
 * @param measurements Container to add to
 * @return 0 on success, -ENOMEM if container full
 */
int crazyflie_measurements_add_dual_channel(deca_short_addr_t addr,
                                            float distance, float confidence,
                                            float phase_diff, uint8_t method,
                                            crazyflie_measurements_t *measurements);

/**
 * @brief Queue measurements for transmission to Crazyflie
 *
 * This function is non-blocking. It queues the measurements for the background
 * connector thread to transmit. If the queue is full, measurements are dropped.
 *
 * @param measurements Container with measurements to send
 * @param timeout Ignored (kept for API compatibility) - always non-blocking
 * @return 0 on success, -EAGAIN if queue full (measurements dropped),
 *         -EINVAL if measurements is NULL or empty
 *
 * @note After calling this function, the measurement container is cleared and
 *       can be reused immediately.
 */
int send_measurement_to_crazyflie(crazyflie_measurements_t *measurements,
                                  k_timeout_t timeout);

/**
 * @brief Send a CRTP packet to the Crazyflie
 *
 * This function queues a CRTP packet for transmission to the Crazyflie STM32.
 * The packet will be forwarded via the SPI HCI protocol and injected into the
 * Crazyflie's CRTP router.
 *
 * This is non-blocking - the packet is queued and the connector thread handles
 * the actual transmission. If there are no pending measurements, this will
 * trigger an immediate SPI exchange.
 *
 * @param data CRTP packet data: [header (1 byte)] [payload (0-30 bytes)]
 *             Header format: port(4 bits) | channel(2 bits) | reserved(2 bits)
 * @param len Total length of data (1-31 bytes)
 * @return 0 on success, -EAGAIN if queue full, -EINVAL if invalid parameters
 *
 * @note CRTP ports: 0=Console, 2=Param, 3=Setpoint, 5=Log, 7=SetpointGeneric
 *
 * Example - send console message:
 *   uint8_t pkt[] = {0x00, 'H', 'i', '!', '\n'};  // Port 0, channel 0
 *   crazyflie_send_crtp(pkt, sizeof(pkt));
 */
int crazyflie_send_crtp(const uint8_t *data, size_t len);

#endif /* SYNCHROFLY_CRAZYFLIE_CONNECTOR_H */
