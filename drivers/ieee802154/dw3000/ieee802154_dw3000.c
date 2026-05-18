/*
 * Copyright (c) 2026 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IEEE 802.15.4 driver for DW3000 UWB radio.
 *
 * Design contract with the UWB driver layer
 * ==========================================
 * The UWB driver (uwb_driver_dw3000) owns ALL direct hardware register
 * access: SYS_STATUS clearing, RDB_STATUS buffer selection, double-buffer
 * mode, and interrupt signalling via the PHY semaphore.  This layer MUST NOT
 * call dwt_writesysstatuslo/hi, dwt_readsysstatuslo, dw3000_clear_status_all,
 * dwt_forcetrxoff, dwt_rxenable, or any buffer-mode functions directly.
 * All transceiver state changes go through the uwb_driver_t vtable.
 *
 * Single-buffer mode
 * ==================
 * The UWB driver init enables double buffering unconditionally.  We override
 * that immediately after init with dwt_setdblrxbuffmode(DBL_BUF_STATE_DIS,
 * DBL_BUF_MODE_MAN).  The UWB ISR buffer-select logic still works correctly
 * in single-buffer mode: RDB_STATUS RXFCG0/1 will not be set, so
 * current_rx_buffer falls back to DW3000_BUFFER_ACCESS_DEFAULT, which maps
 * to RX_BUFFER_0_ID — the single hardware buffer used in this mode.
 *
 * Interrupt / RX flow
 * ====================
 * 1. Hardware IRQ fires -> dw3000_interrupt_handler -> k_work_submit.
 * 2. Work handler reads/clears SYS_STATUS, sets phy_irq_event, gives phy_sem.
 * 3. Dedicated IRQ dispatch thread is the ONLY waiter on wait_for_irq(); it
 *    routes IRQ events to RX and TX event queues.
 * 4. RX thread dequeues RX events, acquires device lock, reads frame via UWB
 *    vtable (read_rx_frame / get_rx_frame_length), re-enables RX via
 *    enable_rx(), releases lock.
 * 5. Frame is dispatched to the net stack outside the lock.
 *
 * TX flow
 * ========
 * 1. force_trx_off (via vtable).
 * 2. setup_tx_frame + start_tx (via vtable).
 * 3. Block on TX event queue populated by the IRQ dispatch thread.
 */

// Controls whether or not to use single buffering mode. Double-buffering seems to cause issues here.
//
// TODO: This can be observed both with single and double buffering,
// but seems to be more prevalent with double-buffering.
//
//     [00:00:02.287,607] <wrn> ieee802154_dw3000: RX: Stray TX IRQ on RX thread, restarting RX
//     [00:00:02.297,271] <wrn> ieee802154_dw3000: TX timeout detected
//     [00:00:02.297,271] <wrn> ieee802154_dw3000: TX unexpected irq_state=0 (because of timeout)
//
// The issue seems to be that some TX IRQ is received by the IRQ scheduler before the TX function
// even set the tx_waiting flag, causing the IRQ to be misrouted to the RX thread and cause an RX restart.
// What is going on here?
#define USE_SINGLE_BUFFERING 1
// Controls whether the driver runs (i.e., starts IRQ and RX threads, allows RX/TX)
// or returns early from init and rejects all RX/TX attempts. Useful for testing the
// rest of the system without running the DW3000 driver.
#define DO_NOT_RUN           0

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ieee802154.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>

#include "dw3000.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ieee802154_dw3000, LOG_LEVEL_WRN);

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define DW3000_FCS_LEN             2U
#define DW3000_MAX_PHY_PACKET_SIZE 127U
#define DW3000_ACK_PKT_LEN         3U
#define DW3000_TX_TIMEOUT_MS       10
#define DW3000_CSMA_MAX_BACKOFFS   4U
#define DW3000_CSMA_MIN_BE         3U
#define DW3000_CSMA_MAX_BE         5U
#define DW3000_UNIT_BACKOFF_US     320U
#define DW3000_CCA_PTO_SYMBOLS     16U
#define DW3000_TX_PG_DELAY         0x34U
#define DW3000_TX_POWER_CH5        0xFDFDFDFDUL
#define DW3000_RX_IRQ_QUEUE_LEN    16U
#define DW3000_TX_IRQ_QUEUE_LEN    4U

/* ------------------------------------------------------------------ */
/* Device data / config                                                */
/* ------------------------------------------------------------------ */

struct dw3000_data {
	struct net_if         *iface;
	const uwb_driver_t    *uwb;
	struct k_thread        rx_thread;
	atomic_t               started;
	atomic_t               tx_waiting;
	uint8_t                mac_addr[8];
	uint16_t               pan_id;
	uint16_t               short_addr;
	uint16_t               channel;
	bool                   promiscuous;
	struct k_msgq          rx_irq_msgq;
	struct k_msgq          tx_irq_msgq;
	char                   rx_irq_msgq_buffer[DW3000_RX_IRQ_QUEUE_LEN *
					      sizeof(uwb_irq_state_e)];
	char                   tx_irq_msgq_buffer[DW3000_TX_IRQ_QUEUE_LEN *
					      sizeof(uwb_irq_state_e)];
	/* Staging buffer for received frames — written under device lock,
	 * read outside it after RX restart has been issued. */
	uint8_t                rx_stage[DW3000_MAX_PHY_PACKET_SIZE];
};

struct dw3000_config {
	k_thread_stack_t *rx_stack;
	size_t            rx_stack_size;
};

/*
 * Static ACK packet wrapper — avoids consuming net_pkt pool entries for
 * pure ACK frames during bursts.
 */
static uint8_t        dw3000_ack_psdu[DW3000_ACK_PKT_LEN];
static struct net_buf dw3000_ack_frame = {
	.data   = dw3000_ack_psdu,
	.size   = DW3000_ACK_PKT_LEN,
	.len    = DW3000_ACK_PKT_LEN,
	.__buf  = dw3000_ack_psdu,
	.frags  = NULL,
};
static struct net_pkt dw3000_ack_pkt = {
	.buffer = &dw3000_ack_frame,
};

/* Forward declaration */
static int dw3000_start(const struct device *dev);

/* ------------------------------------------------------------------ */
/* Helpers — all hardware access goes through the UWB vtable          */
/* ------------------------------------------------------------------ */

/**
 * @brief Re-enable immediate RX through the UWB abstraction layer.
 *
 * No timeout, no delay.  Must be called with the device lock held.
 */
static inline void dw3000_reenable_rx_locked(const struct device *dev,
					     struct dw3000_data *data)
{
  /*
	 * align_double_buffering() is a no-op on DW3000 hardware but is
	 * included for documentation and forward-compatibility: it marks the
	 * point where the double-buffer state machine is re-synchronised
	 * before a new RX cycle begins.
	 */
	data->uwb->align_double_buffering(dev);
	data->uwb->enable_rx(dev, 0, 0);
}

static void dw3000_irq_queue_purge(struct k_msgq *msgq)
{
	uwb_irq_state_e discarded;

	while (k_msgq_get(msgq, &discarded, K_NO_WAIT) == 0) {
	}
}

/* Keep the latest IRQ if queue pressure occurs under bursty traffic. */
static void dw3000_irq_queue_push_latest(struct k_msgq *msgq,
					 uwb_irq_state_e irq_state)
{
	uwb_irq_state_e discarded;

	if (k_msgq_put(msgq, &irq_state, K_NO_WAIT) == 0) {
		return;
	}

	if (k_msgq_get(msgq, &discarded, K_NO_WAIT) == 0) {
		(void)k_msgq_put(msgq, &irq_state, K_NO_WAIT);
	}
}

/**
 * @brief Apply hardware frame filter settings.
 *
 * Must be called with the device lock held.
 */
static int dw3000_apply_filter_hw(struct dw3000_data *data)
{
	uint16_t ff = DWT_FF_BEACON_EN | DWT_FF_DATA_EN | DWT_FF_ACK_EN |
		      DWT_FF_MAC_EN | DWT_FF_MULTI_EN;

	if (data->promiscuous) {
		dwt_configureframefilter(0, 0);
		return 0;
	}

	dwt_setpanid(data->pan_id);
	dwt_setaddress16(data->short_addr);
	dwt_seteui(data->mac_addr);
	dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, ff);

	return 0;
}

/*
 * Interrupt mask applied (and re-applied) after every dwt_configure() call.
 *
 * dwt_configure() is documented to reset the SYS_ENABLE registers as a side
 * effect of PHY reconfiguration.  Any call to dw3000_apply_phy_config_locked()
 * must therefore restore the interrupt mask or the UWB ISR work handler will
 * never see TXFRS/RXFCG and the PHY semaphore will never be given, causing
 * wait_for_irq() to block forever.
 */
#define DW3000_INT_MASK (DWT_INT_TXFRS_BIT_MASK  | \
			 DWT_INT_RXFCG_BIT_MASK  | \
			 DWT_INT_RXFTO_BIT_MASK  | \
			 DWT_INT_RXPTO_BIT_MASK  | \
			 DWT_INT_HPDWARN_BIT_MASK)

/**
 * @brief Apply default PHY configuration for channel 5.
 *
 * Must be called with the device lock held.
 *
 * IMPORTANT: dwt_configure() resets SYS_ENABLE (interrupt mask) as a side
 * effect.  We restore DW3000_INT_MASK unconditionally at the end so that the
 * UWB ISR continues to fire after every PHY reconfiguration.
 */
static int dw3000_apply_phy_config_locked(struct dw3000_data *data)
{
	dwt_txconfig_t tx_cfg;
	dwt_config_t phy_cfg = {
		.chan           = 5U,
		.txPreambLength = DWT_PLEN_128,
		.rxPAC          = DWT_PAC8,
		.txCode         = 10,
		.rxCode         = 10,
		.sfdType        = DWT_SFD_IEEE_4A,
		.dataRate       = DWT_BR_6M8,
		.phrMode        = DWT_PHRMODE_EXT,
		.phrRate        = DWT_PHRRATE_STD,
		.sfdTO          = 128 + 1 + 8 - 8,
		.stsMode        = DWT_STS_MODE_OFF,
		.stsLength      = DWT_STS_LEN_64,
		.pdoaMode       = DWT_PDOA_M0,
	};

	if (dwt_configure(&phy_cfg) != DWT_SUCCESS) {
		return -EIO;
	}

	dwt_configmrxlut(5);

	tx_cfg.PGdly   = DW3000_TX_PG_DELAY;
	tx_cfg.power   = DW3000_TX_POWER_CH5;
	tx_cfg.PGcount = 0U;
	dwt_configuretxrf(&tx_cfg);

	/*
	 * Restore interrupt enables wiped by dwt_configure().
	 * DWT_ENABLE_INT_ONLY leaves already-set bits untouched and adds ours.
	 */
	dwt_setinterrupt(DW3000_INT_MASK, 0U, DWT_ENABLE_INT_ONLY);

	LOG_INF("Applied PHY: ch=5 code=10 tx_pwr=0x%08x", DW3000_TX_POWER_CH5);
	return 0;
}

/* ------------------------------------------------------------------ */
/* RX capture — called from RX thread with device lock held           */
/* ------------------------------------------------------------------ */

/**
 * @brief Read an RX frame from hardware into the staging buffer.
 *
 * Returns 1 if a frame was captured into data->rx_stage and *pkt_len_out
 * was set.  Returns 0 if the frame was handled internally (ACK) or dropped.
 *
 * Precondition: device lock held, irq_state == UWB_IRQ_RX.
 *
 * After reading the frame data this function immediately re-enables RX so
 * that the hardware is not left idle while the net stack processes the frame
 * outside the lock.
 */
static int dw3000_rx_capture_locked(const struct device *dev,
				    struct dw3000_data *data,
				    bool *ack_handled,
				    uint16_t *pkt_len_out)
{
	enum net_verdict  ack_verdict;
	uint16_t          phy_len;
	uint16_t          pkt_len;

	if (ack_handled != NULL) {
		*ack_handled = false;
	}
	if (pkt_len_out != NULL) {
		*pkt_len_out = 0U;
	}

	/*
	 * get_rx_frame_length goes through the UWB vtable -> dwt_getframelength.
	 * The UWB driver owns SYS_STATUS at this point; we must not touch it.
	 */
	phy_len = data->uwb->get_rx_frame_length(dev);
	pkt_len = phy_len;

	if (phy_len < DW3000_FCS_LEN || phy_len > DW3000_MAX_PHY_PACKET_SIZE) {
		LOG_DBG("Drop invalid frame len=%u", phy_len);
    /*
		 * Even for dropped frames we must complete the buffer handshake
		 * so the hardware can reuse the buffer.
		 */
		data->uwb->switch_buffers(dev);
    /* Re-enable RX; UWB ISR already cleared the relevant status bits. */
		dw3000_reenable_rx_locked(dev, data);
		return 0;
	}

#if !defined(CONFIG_IEEE802154_RAW_MODE)
	pkt_len -= DW3000_FCS_LEN;
#endif

	/*
	 * Fast-path minimal ACK frames through the static wrapper to avoid
	 * consuming net_pkt pool entries.
	 */
	if (pkt_len == DW3000_ACK_PKT_LEN) {
    LOG_DBG("RX: Received ACK frame, handling with static wrapper");
		/*
		 * read_rx_frame goes through the UWB vtable which selects the
		 * correct hardware buffer (set by the ISR's current_rx_buffer).
		 */
		data->uwb->read_rx_frame(dev, dw3000_ack_psdu,
					 DW3000_ACK_PKT_LEN, 0);
		data->uwb->switch_buffers(dev);

		dw3000_reenable_rx_locked(dev, data);

		net_pkt_cursor_init(&dw3000_ack_pkt);
		ack_verdict = ieee802154_handle_ack(data->iface, &dw3000_ack_pkt);
		if (ack_handled != NULL && ack_verdict == NET_OK) {
			*ack_handled = true;
		}
		return 0;
	}

	/* Read full frame via UWB vtable. */
	data->uwb->read_rx_frame(dev, data->rx_stage, pkt_len, 0);
  data->uwb->switch_buffers(dev);

	/*
	 * Re-enable RX immediately so hardware is ready for the next frame
	 * while we dispatch the captured data outside the lock.
	 */
	dw3000_reenable_rx_locked(dev, data);

	if (pkt_len_out != NULL) {
		*pkt_len_out = pkt_len;
	}

	return 1;
}

/* ------------------------------------------------------------------ */
/* RX thread                                                           */
/* ------------------------------------------------------------------ */

static void dw3000_rx_thread_fn(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev  = arg1;
	struct dw3000_data  *data = dev->data;
	uwb_irq_state_e      irq_state;
	struct net_pkt      *pkt;
	uint16_t             pkt_len;
	bool                 ack_handled;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		if (k_msgq_get(&data->rx_irq_msgq, &irq_state, K_FOREVER) != 0) {
			continue;
		}

		if (!atomic_get(&data->started) || data->iface == NULL) {
      k_usleep(500000);
			LOG_WRN("RX thread: waiting for radio start / iface…");
			continue;
		}

		pkt_len     = 0U;
		ack_handled = false;

		data->uwb->acquire_device(dev);

		switch (irq_state) {
		case UWB_IRQ_RX:
      LOG_WRN("RX: rxing stuff UWB_IRQ_RX.");
			/*
			 * A good frame has been received.  Capture it into the
			 * staging buffer and restart RX — all under the device lock.
			 * The UWB ISR has already cleared RXFCG in SYS_STATUS.
			 */
			(void)dw3000_rx_capture_locked(dev, data,
						       &ack_handled, &pkt_len);
			break;

		case UWB_IRQ_FRAME_WAIT_TIMEOUT:
		case UWB_IRQ_PREAMBLE_DETECT_TIMEOUT:
		case UWB_IRQ_ERR:
			/*
			 * Transient RX failure — restart receiver.
			 * The UWB ISR cleared the corresponding status bits.
			 */
			LOG_DBG("RX event=%d, restarting receiver", irq_state);

      /* TODO: Do we need to switch buffers here? */
		  data->uwb->switch_buffers(dev);

      dw3000_reenable_rx_locked(dev, data);
			break;

		case UWB_IRQ_TX:
			/*
			 * TX completion observed on the RX thread.  This can
			 * happen if the TX caller was cancelled or timed out.
			 * Simply restart the receiver so we don't stay deaf.
			 */
			LOG_WRN("RX: Stray TX IRQ on RX thread, restarting RX");
			dw3000_reenable_rx_locked(dev, data);
			break;

		case UWB_IRQ_CANCELLED:
			/* Driver is stopping — let the loop re-evaluate started. */
			data->uwb->release_device(dev);
			continue;

		case UWB_IRQ_NONE:
		default:
      /* Check if a real RX event is already queued — process it instead */
      {
          uwb_irq_state_e pending;
          if (k_msgq_get(&data->rx_irq_msgq, &pending, K_NO_WAIT) == 0
              && pending == UWB_IRQ_RX) {
              /* Fall through to RX capture with the real event */
              (void)dw3000_rx_capture_locked(dev, data, &ack_handled, &pkt_len);
          } else {
              dw3000_reenable_rx_locked(dev, data);
          }
      }
			break;
		}

		data->uwb->release_device(dev);

		/* Fast-path: ACK already handled inside capture. */
		if (ack_handled || pkt_len == 0U) {
			continue;
		}

		/*
		 * Allocate net_pkt and dispatch OUTSIDE the device lock so that
		 * pool pressure does not stall the hardware.
		 */
		pkt = net_pkt_rx_alloc_with_buffer(data->iface, pkt_len,
						   AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_WRN("RX: net_pkt alloc failed (len=%u)", pkt_len);
			continue;
		}

		net_pkt_cursor_init(pkt);
		if (net_pkt_write(pkt, data->rx_stage, pkt_len) < 0) {
			LOG_WRN("RX: Failed to write frame to net_pkt");
			net_pkt_unref(pkt);
			continue;
		}
		net_pkt_cursor_init(pkt);

		if (ieee802154_handle_ack(data->iface, pkt) == NET_OK) {
      LOG_DBG("RX: Frame was an ACK, handled internally");
			net_pkt_unref(pkt);
			continue;
		}

		if (net_recv_data(data->iface, pkt) != NET_OK) {
      LOG_WRN("RX: net_recv_data failed, dropping packet");
			net_pkt_unref(pkt);
		}

    LOG_WRN("RX: rxing done, net rcvd .");
	}
}

/* ------------------------------------------------------------------ */
/* IEEE 802.15.4 radio API                                             */
/* ------------------------------------------------------------------ */

static void dw3000_iface_api_init(struct net_if *iface)
{
	const struct device *dev  = net_if_get_device(iface);
	struct dw3000_data  *data = dev->data;
	int ret;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
			     NET_LINK_IEEE802154);
	data->iface = iface;
	ieee802154_init(iface);

	/*
	 * Zephyr's net stack calls set_channel() and filter() during
	 * ieee802154_init(), which in turn calls dw3000_apply_phy_config_locked()
	 * and dwt_configure() — both of which wipe SYS_ENABLE.  Those paths now
	 * restore DW3000_INT_MASK at their end, so by the time we reach here the
	 * interrupt mask is correct.
	 *
	 * Auto-start so the radio is in RX before main() runs.
	 */
	if (!atomic_get(&data->started)) {
		ret = dw3000_start(dev);
		if (ret) {
			LOG_WRN("Auto-start from iface init failed: %d", ret);
		} else {
			LOG_INF("Auto-started radio from iface init");
		}
	}
}

static void dw3000_generate_mac(uint8_t *mac)
{
	uint32_t lo = sys_rand32_get();
	uint32_t hi = sys_rand32_get();

	UNALIGNED_PUT(lo, (uint32_t *)&mac[0]);
	UNALIGNED_PUT(hi, (uint32_t *)&mac[4]);
	mac[0] = (mac[0] & ~0x01U) | 0x02U; /* locally administered, unicast */
}

static enum ieee802154_hw_caps dw3000_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return IEEE802154_HW_FCS | IEEE802154_HW_FILTER |
	       IEEE802154_HW_TXTIME | IEEE802154_HW_CSMA;
}

static int dw3000_cca(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int dw3000_set_channel(const struct device *dev, uint16_t channel)
{
	struct dw3000_data *data = dev->data;
	int ret;

	if (channel == 9U) {
		LOG_ERR("Channel 9 not supported: no RX preamble detection observed");
		return -ENOTSUP;
	}
	if (channel != 5U) {
		return -EINVAL;
	}

	data->uwb->acquire_device(dev);
	data->channel = channel;

	/*
	 * dw3000_apply_phy_config_locked() calls dwt_configure() which wipes
	 * SYS_ENABLE, then restores DW3000_INT_MASK at the end.
	 */
	ret = dw3000_apply_phy_config_locked(data);
	if (ret != 0) {
		LOG_WRN("PHY reconfigure failed for channel=%u", channel);
		data->uwb->release_device(dev);
		return ret;
	}

	if (dwt_setchannel(DWT_CH5) != DWT_SUCCESS) {
		data->uwb->release_device(dev);
		return -EIO;
	}
	/* Restore interrupt mask after dwt_setchannel (precaution). */
	dwt_setinterrupt(DW3000_INT_MASK, 0U, DWT_ENABLE_INT_ONLY);

	data->uwb->release_device(dev);
	return 0;
}

static int dw3000_filter(const struct device *dev,
			 bool set,
			 enum ieee802154_filter_type type,
			 const struct ieee802154_filter *filter)
{
	struct dw3000_data *data = dev->data;

	if (!filter) {
		return -EINVAL;
	}

	data->uwb->acquire_device(dev);

	switch (type) {
	case IEEE802154_FILTER_TYPE_IEEE_ADDR:
		if (set) {
			memcpy(data->mac_addr, filter->ieee_addr,
			       sizeof(data->mac_addr));
		}
		break;
	case IEEE802154_FILTER_TYPE_SHORT_ADDR:
		data->short_addr = set ? filter->short_addr : 0xffffU;
		break;
	case IEEE802154_FILTER_TYPE_PAN_ID:
		data->pan_id = set ? filter->pan_id : 0xffffU;
		break;
	default:
		data->uwb->release_device(dev);
		return -ENOTSUP;
	}

	(void)dw3000_apply_filter_hw(data);
	data->uwb->release_device(dev);
	return 0;
}

static int dw3000_set_txpower(const struct device *dev, int16_t dbm)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dbm);
	return 0;
}

/* ------------------------------------------------------------------ */
/* TX                                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Transmit a frame.
 *
 * TX completion is detected via a TX IRQ queue fed by the dedicated IRQ
 * dispatch thread.  This avoids races with the RX path by keeping
 * wait_for_irq() in one place only.
 *
 * TODO: CSMA-CA backoff retries should be driven by IRQ outcomes from that queue.
 * Not yet supported.
 */
static int dw3000_tx(const struct device *dev,
		     enum ieee802154_tx_mode mode,
		     struct net_pkt *pkt,
		     struct net_buf *frag)
{
  LOG_WRN("TX: transmitting stuff.");
#if DO_NOT_RUN
  return -EBUSY;
#endif

  struct dw3000_data *data = dev->data;
	uwb_irq_state_e     irq_state;
	bool                use_csma  = false;
	uint8_t             be        = DW3000_CSMA_MIN_BE;
	uint8_t             max_tries = 1U;
	uint8_t             tx_try;
	bool                cca_mode  = false;

	ARG_UNUSED(pkt);

  if(atomic_get(&data->tx_waiting) != 0) {
    // TODO: This is super brittle. Also not really an error — the caller can retry later.
    // Not sure what is going on here.
    LOG_ERR("TX reject: already waiting on previous TX");
    return -EBUSY;
  }

	if (!frag || frag->len == 0U) {
		LOG_WRN("TX reject: empty fragment");
		return -EINVAL;
	}
	if (!atomic_get(&data->started)) {
		LOG_WRN("TX reject: radio not started");
		return -ENETDOWN;
	}
	if ((frag->len + DW3000_FCS_LEN) > DW3000_MAX_PHY_PACKET_SIZE) {
		LOG_WRN("TX reject: frame too large len=%u", frag->len);
		return -EMSGSIZE;
	}

	switch (mode) {
	case IEEE802154_TX_MODE_DIRECT:
	case IEEE802154_TX_MODE_TXTIME:
		cca_mode  = false;
		use_csma  = false;
		max_tries = 1U;
		break;
	case IEEE802154_TX_MODE_CCA:
	case IEEE802154_TX_MODE_TXTIME_CCA:
		cca_mode  = true;
		use_csma  = false;
		max_tries = 1U;
		break;
	case IEEE802154_TX_MODE_CSMA_CA:
		cca_mode  = true;
		use_csma  = true;
		max_tries = DW3000_CSMA_MAX_BACKOFFS + 1U;
		break;
	default:
		LOG_WRN("Unsupported TX mode=%d", mode);
		return -ENOTSUP;
	}

	for (tx_try = 0U; tx_try < max_tries; tx_try++) {
		/* CSMA-CA: random backoff before retry. */
		if (use_csma && tx_try > 0U) {
			uint32_t bo_slots  = 1U << MIN(be, DW3000_CSMA_MAX_BE);
			uint32_t rand_slots = sys_rand32_get() % bo_slots;

			k_usleep(rand_slots * DW3000_UNIT_BACKOFF_US);
			if (be < DW3000_CSMA_MAX_BE) {
				be++;
			}
		}

		data->uwb->acquire_device(dev);

		/*
		 * Stop any ongoing RX before configuring TX.
		 * Goes through the UWB vtable — the vtable calls dwt_forcetrxoff
		 * which is safe here because we hold the device lock that the
		 * UWB ISR work handler also waits on.
		 */
		data->uwb->force_trx_off(dev);

		/*
		 * For CCA mode, configure a short preamble detection timeout so
		 * the DW3000 can sense the channel.  The UWB driver's preamble
		 * timeout helper goes through dwt_setpreambledetecttimeout.
		 */
		if (cca_mode) {
			data->uwb->setup_preamble_timeout(dev,
							  DW3000_CCA_PTO_SYMBOLS);
		} else {
			data->uwb->setup_preamble_timeout(dev, 0U);
		}

		data->uwb->clear_timeouts(dev); /* clear frame timeout */
		dw3000_irq_queue_purge(&data->tx_irq_msgq);
		atomic_set(&data->tx_waiting, 1);

		/*
		 * Load frame into TX buffer and start transmission.
		 * setup_tx_frame adds the 2-byte FCS length internally.
		 * start_tx(dev, 0) -> immediate TX.
		 */
		data->uwb->setup_tx_frame(dev, frag->data, (uint16_t)frag->len);

		if (cca_mode) {
			/*
			 * CCA TX: start with DWT_START_TX_CCA.  The DW3000 will
			 * listen for a preamble first; if the channel is busy it
			 * raises CCA_FAIL instead of TXFRS.  We use the UWB
			 * vtable's start_tx for the delayed path but need CCA
			 * mode — fall back to the direct deca API for this flag.
       *
       * TODO: This can break things in the UWB driver!!!
			 */
			if (dwt_starttx(DWT_START_TX_CCA) != DWT_SUCCESS) {
        LOG_ERR("1 Setting tx_waiting=0");
				atomic_set(&data->tx_waiting, 0);
				data->uwb->setup_preamble_timeout(dev, 0U);
				dw3000_reenable_rx_locked(dev, data);
				data->uwb->release_device(dev);

				if (use_csma && (tx_try + 1U) < max_tries) {
					continue;
				}
				return -EIO;
			}
		} else {
			// LOG_WRN("TX: starting trans, tx_waiting=%d", atomic_get(&data->tx_waiting));
      if (data->uwb->start_tx(dev, 0) != 0) {
        LOG_ERR("2 Setting tx_waiting=0");
				atomic_set(&data->tx_waiting, 0);
				dw3000_reenable_rx_locked(dev, data);
				data->uwb->release_device(dev);
				return -EIO;
			}
		}

		/*
		 * Release the device lock BEFORE blocking on TX IRQ queue so
		 * that the UWB ISR work handler and IRQ dispatch thread can run.
		 */
		data->uwb->release_device(dev);

		if (k_msgq_get(&data->tx_irq_msgq, &irq_state,
			      K_MSEC(DW3000_TX_TIMEOUT_MS)) != 0) {
			LOG_ERR("TX timed out waiting for IRQ, tx_waiting=%d", atomic_get(&data->tx_waiting));
			irq_state = UWB_IRQ_NONE;
		}
		atomic_set(&data->tx_waiting, 0);

		data->uwb->acquire_device(dev);

		switch (irq_state) {
		case UWB_IRQ_TX:
			/* Success — re-enable RX and return. */
			dw3000_reenable_rx_locked(dev, data);
			data->uwb->release_device(dev);
			return 0;

		case UWB_IRQ_HALF_DELAY_WARNING:
			/*
			 * Delayed TX scheduled too close to current time.
			 * Treat as failure; caller should retry if applicable.
			 */
			LOG_WRN("TX half-delay warning");
			dw3000_reenable_rx_locked(dev, data);
			data->uwb->release_device(dev);
			return -EAGAIN;

		case UWB_IRQ_ERR:
			/*
			 * CCA failure is reported as an RX error (channel busy)
			 * on the DW3000 when using DWT_START_TX_CCA.
			 */
			if (cca_mode) {
				dw3000_reenable_rx_locked(dev, data);
				data->uwb->release_device(dev);
				if (use_csma && (tx_try + 1U) < max_tries) {
					break; /* outer for loop — backoff */
				}
				return -EBUSY;
			}
			/* Fall through for non-CCA errors. */
			__fallthrough;

		case UWB_IRQ_FRAME_WAIT_TIMEOUT:
		case UWB_IRQ_PREAMBLE_DETECT_TIMEOUT:
		case UWB_IRQ_CANCELLED:
		case UWB_IRQ_NONE:
		default:
			LOG_WRN("TX unexpected irq_state=%d", irq_state);
			dw3000_reenable_rx_locked(dev, data);
			data->uwb->release_device(dev);
			if (use_csma && (tx_try + 1U) < max_tries) {
				break; /* outer for loop — backoff */
			}
			return -EIO;
		}
	}

	LOG_ERR("TX failed after %u tries", max_tries);
	return -EBUSY;
}

/* ------------------------------------------------------------------ */
/* Start / Stop                                                        */
/* ------------------------------------------------------------------ */

static int dw3000_start(const struct device *dev)
{
	struct dw3000_data *data = dev->data;

	LOG_DBG("dw3000_start: %s", dev->name);

	data->uwb->acquire_device(dev);
	atomic_set(&data->started, 1);
  LOG_ERR("dw3000_start: Setting tx_waiting=0");
	atomic_set(&data->tx_waiting, 0);
	dw3000_irq_queue_purge(&data->rx_irq_msgq);
	dw3000_irq_queue_purge(&data->tx_irq_msgq);

	/*
	 * Ensure the transceiver is off before starting fresh, then enable RX.
	 * We go through the UWB vtable for both operations.
	 */
	data->uwb->force_trx_off(dev);
	data->uwb->clear_timeouts(dev);
	/*
	 * Align the double-buffer state machine before the first RX enable
	 * so both the host and the DW3000 agree on which buffer is "current".
	 */
	data->uwb->align_double_buffering(dev);

	dw3000_reenable_rx_locked(dev, data);

	data->uwb->release_device(dev);
	return 0;
}

static int dw3000_stop(const struct device *dev)
{
	struct dw3000_data *data = dev->data;

	data->uwb->acquire_device(dev);
	atomic_set(&data->started, 0);
  LOG_ERR("dw3000_stop: Setting tx_waiting=0");
	atomic_set(&data->tx_waiting, 0);
	data->uwb->disable_txrx(dev);
	data->uwb->release_device(dev);

	/*
	 * Unblock the RX thread so it can observe started=0 and park cleanly.
	 * cancel_wait() sets phy_irq_event = DW3000_IRQ_CANCELLED and gives
	 * the PHY semaphore.
	 */
	data->uwb->cancel_wait(dev);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Configure / Attributes                                              */
/* ------------------------------------------------------------------ */

static int dw3000_configure(const struct device *dev,
			    enum ieee802154_config_type type,
			    const struct ieee802154_config *config)
{
	struct dw3000_data *data = dev->data;

	switch (type) {
	case IEEE802154_CONFIG_PROMISCUOUS:
		if (config == NULL) {
			return -EINVAL;
		}
		data->uwb->acquire_device(dev);
		data->promiscuous = config->promiscuous;
		(void)dw3000_apply_filter_hw(data);
		data->uwb->release_device(dev);
		return 0;

	default:
		return -ENOTSUP;
	}
}

static const struct {
	const struct ieee802154_phy_channel_range      phy_channel_range[1];
	const struct ieee802154_phy_supported_channels phy_supported_channels;
} dw3000_drv_attr = {
	.phy_channel_range = {
		{ .from_channel = 5, .to_channel = 5 },
	},
	.phy_supported_channels = {
		.ranges     = dw3000_drv_attr.phy_channel_range,
		.num_ranges = 1U,
	},
};

static int dw3000_attr_get(const struct device *dev,
			   enum ieee802154_attr attr,
			   struct ieee802154_attr_value *value)
{
	ARG_UNUSED(dev);

	if (ieee802154_attr_get_channel_page_and_range(
		    attr,
		    IEEE802154_ATTR_PHY_CHANNEL_PAGE_FOUR_HRP_UWB,
		    &dw3000_drv_attr.phy_supported_channels,
		    value) == 0) {
		return 0;
	}
	return -ENOENT;
}

static const struct ieee802154_radio_api dw3000_radio_api = {
	.iface_api.init   = dw3000_iface_api_init,
	.get_capabilities = dw3000_get_capabilities,
	.cca              = dw3000_cca,
	.set_channel      = dw3000_set_channel,
	.filter           = dw3000_filter,
	.set_txpower      = dw3000_set_txpower,
	.tx               = dw3000_tx,
	.start            = dw3000_start,
	.stop             = dw3000_stop,
	.configure        = dw3000_configure,
	.attr_get         = dw3000_attr_get,
};

/* ------------------------------------------------------------------ */
/* Driver initialization                                               */
/* ------------------------------------------------------------------ */

static int dw3000_init(const struct device *dev)
{
	struct dw3000_data        *data = dev->data;
	const struct dw3000_config *cfg = dev->config;
	int ret;
	int irq_prio = 7;
	int rx_prio = 8;

	/* Initialize the UWB hardware layer (resets chip, configures SPI,
	 * enables interrupts, enables double buffering at the end). */
	ret = uwb_driver_dw3000_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize DW3000 core: %d", ret);
		return ret;
	}

	data->uwb = uwb_driver_get(dev);
	if (data->uwb == NULL) {
		LOG_ERR("No registered UWB driver for %s", dev->name);
		return -ENODEV;
	}

	atomic_set(&data->started, 0);
  LOG_ERR("dw3000_init: Setting tx_waiting=0");
	atomic_set(&data->tx_waiting, 0);
	k_msgq_init(&data->rx_irq_msgq,
		    data->rx_irq_msgq_buffer,
		    sizeof(uwb_irq_state_e),
		    DW3000_RX_IRQ_QUEUE_LEN);
	k_msgq_init(&data->tx_irq_msgq,
		    data->tx_irq_msgq_buffer,
		    sizeof(uwb_irq_state_e),
		    DW3000_TX_IRQ_QUEUE_LEN);
	dw3000_generate_mac(data->mac_addr);
	data->pan_id     = 0xffffU;
	data->short_addr = 0xffffU;
	data->channel    = 5U;

#if defined(CONFIG_NET_CONFIG_IEEE802154_CHANNEL)
	if (CONFIG_NET_CONFIG_IEEE802154_CHANNEL == 9) {
		LOG_ERR("Channel 9 is not supported");
		return -ENOTSUP;
	}
	if (CONFIG_NET_CONFIG_IEEE802154_CHANNEL == 5) {
		data->channel = (uint16_t)CONFIG_NET_CONFIG_IEEE802154_CHANNEL;
	}
#endif

	data->promiscuous = false;

	data->uwb->acquire_device(dev);

	/*
	 * Override the double-buffer mode that uwb_driver_dw3000_init()
	 * enabled at the end of its initialization sequence.
	 *
	 * We use single-buffer mode because this driver's RX thread has
	 * sufficient time to call enable_rx() between frame arrivals, making
	 * double-buffering complexity unnecessary.
	 *
	 * The UWB ISR's buffer-select logic handles this gracefully: when
	 * RDB_STATUS RXFCG0/1 bits are not set (as in single-buffer mode),
	 * current_rx_buffer falls back to DW3000_BUFFER_ACCESS_DEFAULT, which
	 * reads from RX_BUFFER_0_ID — the correct location in single-buffer
	 * mode.
	 */
	/* dwt_setdblrxbuffmode() does NOT touch SYS_ENABLE. */
#if USE_SINGLE_BUFFERING
  // dwt_setdblrxbuffmode(DBL_BUF_STATE_DIS, DBL_BUF_MODE_MAN);
  data->uwb->enable_single_buffering(dev);
#endif

	/*
	 * Apply PHY config (channel 5, preamble, TX power).
	 * dwt_configure() wipes SYS_ENABLE internally; the helper restores
	 * DW3000_INT_MASK at the end so interrupts keep working.
	 */
	ret = dw3000_apply_phy_config_locked(data);
	if (ret != 0) {
		data->uwb->release_device(dev);
		LOG_ERR("Failed to configure DW3000 PHY: %d", ret);
		return ret;
	}

	/*
	 * dwt_setchannel() may update RF calibration tables; restore the
	 * interrupt mask afterwards as a precaution.
	 */
	if (dwt_setchannel(DWT_CH5) != DWT_SUCCESS) {
		data->uwb->release_device(dev);
		LOG_ERR("dwt_setchannel failed during init");
		return -EIO;
	}
	dwt_setinterrupt(DW3000_INT_MASK, 0U, DWT_ENABLE_INT_ONLY);

	(void)dw3000_apply_filter_hw(data);
	data->uwb->clear_timeouts(dev);

	data->uwb->release_device(dev);

#if defined(CONFIG_NET_CONFIG_IEEE802154_CHANNEL)
	LOG_WRN("DW3000 IEEE802154 init channel=%u (kconfig=%d)",
		data->channel, CONFIG_NET_CONFIG_IEEE802154_CHANNEL);
#else
	LOG_WRN("DW3000 IEEE802154 init channel=%u", data->channel);
#endif

#if DO_NOT_RUN
  LOG_ERR("Returning early for glossy");
  return 0;
#endif

  uwb_broker_init(dev,
                  &data->rx_irq_msgq,
                  &data->tx_irq_msgq,
                  &data->tx_waiting);

	k_thread_create(&data->rx_thread,
			cfg->rx_stack,
			cfg->rx_stack_size,
			dw3000_rx_thread_fn,
			(void *)dev,
			NULL,
			NULL,
			rx_prio,
			0,
			K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "dw3000_rx");

	LOG_INF("DW3000 IEEE802154 driver initialized");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Device tree instantiation                                           */
/* ------------------------------------------------------------------ */

#define DT_DRV_COMPAT decawave_dw3000
#define DWT_PSDU_LENGTH (DW3000_MAX_PHY_PACKET_SIZE - DW3000_FCS_LEN)

#define DW3000_INIT(n)                                              \
	K_KERNEL_STACK_DEFINE(dw3000_rx_stack_##n,                        \
		CONFIG_IEEE802154_DW3000_IRQ_THREAD_STACK_SIZE);                \
	static struct dw3000_data dw3000_data_##n;                        \
	static const struct dw3000_config dw3000_config_##n = {           \
		.rx_stack      = dw3000_rx_stack_##n,                           \
		.rx_stack_size = K_KERNEL_STACK_SIZEOF(dw3000_rx_stack_##n),    \
	};                                                                \
	NET_DEVICE_DT_INST_DEFINE(n,                                      \
		dw3000_init,                                                    \
		NULL,                                                           \
		&dw3000_data_##n,                                               \
		&dw3000_config_##n,                                             \
		CONFIG_IEEE802154_DW3000_INIT_PRIO,                             \
		&dw3000_radio_api,                                              \
		IEEE802154_L2,                                                  \
		NET_L2_GET_CTX_TYPE(IEEE802154_L2),                             \
		DWT_PSDU_LENGTH)

DT_INST_FOREACH_STATUS_OKAY(DW3000_INIT)