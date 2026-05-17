/*
 * Copyright (c) 2026 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UWB IRQ Broker
 * ==============
 * Singleton that owns the wait_for_irq() call loop and routes interrupt
 * events to either the IEEE 802.15.4 driver queues or the Glossy flood
 * caller, depending on which consumer currently holds the lease.
 *
 * Lifecycle
 * ---------
 * 1. dw3000_init() calls uwb_broker_init() after both 802.15.4 msgqs are
 *    initialized.  uwb_broker_init() starts the broker IRQ thread.
 * 2. The broker thread runs forever, calling wait_for_irq() and routing
 *    events.  It replaces dw3000_irq_dispatch_thread_fn entirely.
 * 3. Glossy calls uwb_broker_acquire_lease() before uwb_glossy_flood() and
 *    uwb_broker_release_lease() afterwards (even on error).
 * 4. While the lease is held by Glossy, uwb_broker_glossy_wait() delivers
 *    IRQ events to Glossy's caller thread via an internal msgq.
 *    The 802.15.4 RX/TX threads receive UWB_IRQ_CANCELLED so they park
 *    cleanly without spinning.
 */

#ifndef UWB_IRQ_BROKER_H
#define UWB_IRQ_BROKER_H

#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the UWB IRQ broker.
 *
 * Must be called exactly once from dw3000_init(), after both
 * ieee_rx_msgq and ieee_tx_msgq have been initialized with k_msgq_init().
 * Starts the broker IRQ thread internally.
 *
 * The broker does NOT take ownership of the 802.15.4 queues — it only
 * borrows pointers to push events into them.
 *
 * @param dev          UWB device handle.
 * @param ieee_rx_msgq Pointer to dw3000_data.rx_irq_msgq (borrowed).
 * @param ieee_tx_msgq Pointer to dw3000_data.tx_irq_msgq (borrowed).
 * @param ieee_tx_waiting Pointer to dw3000_data.tx_waiting atomic (borrowed).
 */
void uwb_broker_init(const struct device *dev,
                     struct k_msgq *ieee_rx_msgq,
                     struct k_msgq *ieee_tx_msgq,
                     atomic_t *ieee_tx_waiting);

/**
 * @brief Try to acquire the radio lease for Glossy.
 *
 * Grants exclusive IRQ routing to the Glossy consumer.  While the lease
 * is held, all IRQ events are delivered to the internal Glossy msgq and
 * UWB_IRQ_CANCELLED is pushed to the 802.15.4 queues so those threads
 * park cleanly.
 *
 * The call is non-blocking: it returns -EBUSY immediately if a 802.15.4
 * TX is in progress (ieee_tx_waiting != 0).
 *
 * @retval 0        Lease granted; caller may proceed with uwb_glossy_flood().
 * @retval -EBUSY   802.15.4 TX active; Glossy round should be skipped.
 */
int uwb_broker_acquire_lease(const struct device *dev);

/**
 * @brief Release the Glossy lease and return IRQ routing to 802.15.4.
 *
 * Must be called after uwb_glossy_flood() returns, regardless of success
 * or failure.  The 802.15.4 RX thread will re-enable RX on its next loop
 * iteration via its existing UWB_IRQ_CANCELLED handling.
 */
void uwb_broker_release_lease(const struct device *dev);

/**
 * @brief Wait for the next IRQ event while the Glossy lease is held.
 *
 * Replaces direct uwb_driver->wait_for_irq() calls inside
 * uwb_glossy_flood().  Blocks on the broker's internal Glossy msgq.
 *
 * If the timeout expires before an event arrives, returns
 * UWB_IRQ_FRAME_WAIT_TIMEOUT.
 *
 * @param dev     UWB device handle.
 * @param timeout Maximum time to wait (use K_USEC(), K_MSEC(), etc.).
 * @return        IRQ state delivered by the broker thread.
 */
uwb_irq_state_e uwb_broker_glossy_wait(const struct device *dev,
                                       k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* UWB_IRQ_BROKER_H */