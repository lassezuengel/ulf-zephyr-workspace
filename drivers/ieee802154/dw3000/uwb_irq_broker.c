/*
 * Copyright (c) 2026 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/drivers/ieee802154/uwb_irq_broker.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "dw3000.h"

LOG_MODULE_REGISTER(uwb_irq_broker, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */
/* Configuration                                                               */
/* -------------------------------------------------------------------------- */

#define BROKER_GLOSSY_MSGQ_LEN 8
#define BROKER_THREAD_STACK_SIZE 1024
#define BROKER_THREAD_PRIORITY 7 /* same as former dw3000_irq thread */

/* -------------------------------------------------------------------------- */
/* Internal state — all private to this file                                  */
/* -------------------------------------------------------------------------- */

typedef enum {
  BROKER_OWNER_IEEE802154 = 0, /* default; 802.15.4 receives all events */
  BROKER_OWNER_GLOSSY,
} broker_owner_e;

static struct {
  /* UWB device — set once in uwb_broker_init() */
  const struct device *dev;
  const uwb_driver_t *uwb;

  /* Lease state */
  struct k_mutex lease_mutex;
  atomic_t owner; /* broker_owner_e */
  atomic_t ieee_paused;

  /* Borrowed pointers to 802.15.4 driver queues */
  struct k_msgq *ieee_rx_msgq;
  struct k_msgq *ieee_tx_msgq;
  atomic_t *ieee_tx_waiting; /* dw3000_data.tx_waiting */

  /* Glossy msgq — owned by broker */
  struct k_msgq glossy_msgq;
  char glossy_msgq_buf[BROKER_GLOSSY_MSGQ_LEN *
                       sizeof(uwb_irq_state_e)];

  /* Broker IRQ thread */
  struct k_thread thread;
  bool initialized;
} broker = {
    .lease_mutex = Z_MUTEX_INITIALIZER(broker.lease_mutex),
};

K_THREAD_STACK_DEFINE(broker_thread_stack, BROKER_THREAD_STACK_SIZE);

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

/*
 * Push to a msgq, evicting the oldest entry if full (keep-latest policy).
 * Identical to dw3000_irq_queue_push_latest() in the 802.15.4 driver.
 */
static void broker_msgq_push(struct k_msgq *q, uwb_irq_state_e state) {
  uwb_irq_state_e discard;

  if (k_msgq_put(q, &state, K_NO_WAIT) == 0) {
    return;
  }
  /* Queue full — drop oldest, insert latest */
  if (k_msgq_get(q, &discard, K_NO_WAIT) == 0) {
    (void)k_msgq_put(q, &state, K_NO_WAIT);
  }
}

static void broker_msgq_purge(struct k_msgq *q) {
  uwb_irq_state_e discard;

  while (k_msgq_get(q, &discard, K_NO_WAIT) == 0) {
  }
}

/*
 * Route an IRQ event to the 802.15.4 driver's rx or tx queue,
 * replicating the logic that previously lived in
 * dw3000_irq_dispatch_thread_fn().
 */
static void broker_route_to_ieee(uwb_irq_state_e state) {
  bool tx_pending = atomic_get(broker.ieee_tx_waiting) != 0;

  switch (state) {
  case UWB_IRQ_RX:
    LOG_DBG("RX IRQ in broker IEEE path");
    broker_msgq_push(broker.ieee_rx_msgq, state);
    break;

  case UWB_IRQ_TX:
    LOG_DBG("TX IRQ in broker IEEE path");
    if (tx_pending) {
      broker_msgq_push(broker.ieee_tx_msgq, state);
    } else {
      broker_msgq_push(broker.ieee_rx_msgq, state);
    }
    break;

  case UWB_IRQ_CCA_BUSY:
    LOG_DBG("CCA busy IRQ in broker IEEE path");
    if (tx_pending) {
      broker_msgq_push(broker.ieee_tx_msgq, state);
    } else {
      broker_msgq_push(broker.ieee_rx_msgq, state);
    }
    break;

  case UWB_IRQ_HALF_DELAY_WARNING:
    if (tx_pending) {
      broker_msgq_push(broker.ieee_tx_msgq, state);
    } else {
      broker_msgq_push(broker.ieee_rx_msgq, state);
    }
    break;

  case UWB_IRQ_CANCELLED:
    LOG_DBG("CANCEL IRQ in broker IEEE path");
    /* Wake both — used during stop transitions */
    broker_msgq_push(broker.ieee_tx_msgq, state);
    broker_msgq_push(broker.ieee_rx_msgq, state);
    break;

  case UWB_IRQ_FRAME_WAIT_TIMEOUT:
  case UWB_IRQ_PREAMBLE_DETECT_TIMEOUT:
  case UWB_IRQ_ERR:
  case UWB_IRQ_NONE:
  default:
    if (tx_pending) {
      broker_msgq_push(broker.ieee_tx_msgq, state);
    } else {
      broker_msgq_push(broker.ieee_rx_msgq, state);
    }
    break;
  }
}

/* -------------------------------------------------------------------------- */
/* Broker IRQ thread                                                           */
/* -------------------------------------------------------------------------- */

/*
 * This thread replaces dw3000_irq_dispatch_thread_fn() entirely.
 * It is the sole caller of wait_for_irq() in the system.
 */
static void broker_thread_fn(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  LOG_INF("UWB IRQ broker thread started");

  while (true) {
    uwb_irq_state_e state = broker.uwb->wait_for_irq(broker.dev);
    broker_owner_e owner = (broker_owner_e)atomic_get(&broker.owner);

    // if(state == UWB_IRQ_RX) {
    //   LOG_ERR("IRQ received: state=%d, owner=%d", state, owner);
    // }

    switch (owner) {
    case BROKER_OWNER_GLOSSY:
      /*
       * Glossy holds the lease.  Deliver the event to the Glossy
       * msgq; uwb_broker_glossy_wait() is blocking on it.
       */
      LOG_DBG("BROKER -> GLOSSY: irq_state=%d", state);
      broker_msgq_push(&broker.glossy_msgq, state);
      break;

    case BROKER_OWNER_IEEE802154:
    default:
      /*
       * Normal operation — route to the 802.15.4 driver queues
       * using the same tx_waiting heuristic as before.
       */
      LOG_DBG("BROKER -> IEEE802154: irq_state=%d", state);
      broker_route_to_ieee(state);
      break;
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void uwb_broker_init(const struct device *dev,
                     struct k_msgq *ieee_rx_msgq,
                     struct k_msgq *ieee_tx_msgq,
                     atomic_t *ieee_tx_waiting) {
  __ASSERT(!broker.initialized, "uwb_broker_init() called more than once");
  __ASSERT(dev != NULL, "dev is NULL");
  __ASSERT(ieee_rx_msgq != NULL, "ieee_rx_msgq is NULL");
  __ASSERT(ieee_tx_msgq != NULL, "ieee_tx_msgq is NULL");
  __ASSERT(ieee_tx_waiting != NULL, "ieee_tx_waiting is NULL");

  broker.dev = dev;
  broker.uwb = uwb_driver_get(dev);
  broker.ieee_rx_msgq = ieee_rx_msgq;
  broker.ieee_tx_msgq = ieee_tx_msgq;
  broker.ieee_tx_waiting = ieee_tx_waiting;

  __ASSERT(broker.uwb != NULL, "No UWB driver registered for device");

  k_msgq_init(&broker.glossy_msgq,
              broker.glossy_msgq_buf,
              sizeof(uwb_irq_state_e),
              BROKER_GLOSSY_MSGQ_LEN);

  atomic_set(&broker.owner, BROKER_OWNER_IEEE802154);
  atomic_set(&broker.ieee_paused, 0);

  k_thread_create(&broker.thread,
                  broker_thread_stack,
                  K_THREAD_STACK_SIZEOF(broker_thread_stack),
                  broker_thread_fn,
                  NULL, NULL, NULL,
                  BROKER_THREAD_PRIORITY,
                  0,
                  K_NO_WAIT);
  k_thread_name_set(&broker.thread, "uwb_irq_broker");

  broker.initialized = true;
  LOG_INF("UWB IRQ broker initialized");
}

int uwb_broker_acquire_lease(const struct device *dev) {
  __ASSERT(broker.initialized, "uwb_broker_acquire_lease() before init");

  k_mutex_lock(&broker.lease_mutex, K_FOREVER);
  atomic_set(&broker.ieee_paused, 1);

  /*
   * Refuse if the 802.15.4 driver is mid-TX.  A TX is considered active
   * from the moment dw3000_tx() sets tx_waiting=1 until it clears it
   * after receiving the TX IRQ (or timing out).  We must not steal the
   * radio while a delayed TX is programmed into the DW3000 scheduler.
   */
  if (atomic_get(broker.ieee_tx_waiting) != 0) {
    LOG_WRN("BROKER: acquire_lease rejected - 802.15.4 TX in progress");
    atomic_set(&broker.ieee_paused, 0);
    k_mutex_unlock(&broker.lease_mutex);
    return -EBUSY;
  }

  /*
   * Park the 802.15.4 threads.  Pushing CANCELLED to both queues causes:
   *   - dw3000_rx_thread_fn:  exits its UWB_IRQ_RX/timeout handling,
   *     hits the UWB_IRQ_CANCELLED case, calls continue — it will
   *     block on k_msgq_get() waiting for the next event.
   *   - dw3000_tx() (if blocking on tx_irq_msgq): unblocks, sees
   *     UWB_IRQ_CANCELLED, falls through to the error return path.
   *     Since tx_waiting was 0 above, dw3000_tx() is not currently
   *     blocking, so this push is just a defensive flush.
   *
   * After this point, the broker thread will route all new IRQ events
   * to the Glossy msgq, so the 802.15.4 threads will receive no further
   * events until the lease is released.
   */
  broker_msgq_push(broker.ieee_rx_msgq, UWB_IRQ_CANCELLED);
  broker_msgq_push(broker.ieee_tx_msgq, UWB_IRQ_CANCELLED);
  k_sleep(K_MSEC(2)); // let RX thread reach k_msgq_get() before Glossy grabs the radio

  /*
   * Park the radio and drain stale IRQs before switching ownership.  This
   * matters now that the UWB driver preserves a queue of IRQ events: an
   * 802.15.4 RX/TX/timeout event queued before the lease must not become
   * the first event observed by Glossy.
   */
  broker.uwb->acquire_device(dev);
  broker.uwb->force_trx_off(dev);
  broker.uwb->clear_timeouts(dev);
  if (broker.uwb->flush_irq != NULL) {
    broker.uwb->flush_irq(dev);
  }
  broker.uwb->release_device(dev);

  /* Purge any stale events from a previous Glossy round */
  broker_msgq_purge(&broker.glossy_msgq);

  atomic_set(&broker.owner, BROKER_OWNER_GLOSSY);

  k_mutex_unlock(&broker.lease_mutex);
  LOG_DBG("BROKER: Glossy lease acquired");
  return 0;
}

// TODO: This stuff is already defined in the 802154 driver .c file.
// Move to some common header if needed by both.
#define DW3000_INT_MASK (DWT_INT_TXFRS_BIT_MASK | \
                         DWT_INT_RXFCG_BIT_MASK | \
                         DWT_INT_RXFTO_BIT_MASK | \
                         DWT_INT_RXPTO_BIT_MASK | \
                         DWT_INT_HPDWARN_BIT_MASK)
#define DW3000_INT_MASK_HI DWT_INT_HI_CCA_FAIL_BIT_MASK

void uwb_broker_release_lease(const struct device *dev) {
  k_mutex_lock(&broker.lease_mutex, K_FOREVER);

  // TODO: Do this cleanly either in the glossy code (breaks this)
  // or in the ieee802154 driver (needs this)
  broker.uwb->acquire_device(dev);  // need the lock for hw access
  broker.uwb->force_trx_off(dev);   // bring radio to clean idle first
  broker.uwb->clear_timeouts(dev);
  if (broker.uwb->flush_irq != NULL) {
    broker.uwb->flush_irq(dev);
  }
  broker_msgq_purge(&broker.glossy_msgq);

  // TODO: Could be an issue: the third argument is evaluated as bool in the UWB driver
  // but we assume bit flags here. Maybe need to split into two separate calls in the UWB vtable?
  broker.uwb->set_frame_filter(dev, // restore frame filter
                               DWT_FF_ENABLE_802_15_4,
                               DWT_FF_BEACON_EN | DWT_FF_DATA_EN | DWT_FF_ACK_EN |
                                   DWT_FF_MAC_EN | DWT_FF_MULTI_EN);
  dwt_setinterrupt(DW3000_INT_MASK, DW3000_INT_MASK_HI, DWT_ENABLE_INT_ONLY); // restore int mask
  broker.uwb->align_double_buffering(dev);
  broker.uwb->enable_rx(dev, 0, 0);
  broker.uwb->release_device(dev);

  atomic_set(&broker.owner, BROKER_OWNER_IEEE802154);
  atomic_set(&broker.ieee_paused, 0);
  broker_msgq_push(broker.ieee_rx_msgq, UWB_IRQ_NONE);
  k_mutex_unlock(&broker.lease_mutex);
}

bool uwb_broker_ieee_active(void) {
  return broker.initialized &&
         (broker_owner_e)atomic_get(&broker.owner) == BROKER_OWNER_IEEE802154 &&
         atomic_get(&broker.ieee_paused) == 0;
}

uwb_irq_state_e uwb_broker_glossy_wait(const struct device *dev,
                                       k_timeout_t timeout) {
  ARG_UNUSED(dev);
  __ASSERT(broker.initialized, "uwb_broker_glossy_wait() before init");
  __ASSERT((broker_owner_e)atomic_get(&broker.owner) == BROKER_OWNER_GLOSSY,
           "uwb_broker_glossy_wait() called without holding the lease");

  uwb_irq_state_e state;

  if (k_msgq_get(&broker.glossy_msgq, &state, timeout) != 0) {
    /* Timed out — return the most appropriate synthetic state */
    return UWB_IRQ_FRAME_WAIT_TIMEOUT;
  }

  return state;
}
