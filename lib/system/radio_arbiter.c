/*
 * Copyright (c) 2025 SynchroFly
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/lib/system/radio_arbiter.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(radio_arbiter, LOG_LEVEL_INF);

static atomic_t radio_available = ATOMIC_INIT(0);
static const struct device *uwb_device;
static const uwb_driver_t *uwb_drv;

int radio_arbiter_init(const struct device *uwb_dev)
{
	if (!uwb_dev) {
		return -EINVAL;
	}

	uwb_device = uwb_dev;
	uwb_drv = uwb_driver_get(uwb_dev);
	if (!uwb_drv) {
		return -ENODEV;
	}

	atomic_clear(&radio_available);
	LOG_INF("Radio arbiter initialized, radio claimed");
	return 0;
}

void radio_arbiter_release(void)
{
	atomic_set(&radio_available, 1);
	LOG_INF("release");
}

void radio_arbiter_claim(void)
{
	if (atomic_cas(&radio_available, 1, 0)) {
		LOG_INF("claim (was available, now claimed)");

		/*
		 * Prevent concurrent SPI access from three sources:
		 *
		 * 1. UWB RX thread (preemptible, prio 6) holds dev_lock
		 *    during SPI transfers (read_rx_frame, etc.).
		 *    acquire_device blocks until the RX thread releases.
		 *
		 * 2. DW1000 IRQ handler on dwt_work_queue (cooperative
		 *    prio -1) holds dev_lock during SPI access.
		 *    disable_int prevents new IRQs; flush_irq drains
		 *    already-submitted work; acquire_device waits for
		 *    any handler still holding dev_lock.
		 *
		 * 3. Swarm TX thread holds dev_lock during SPI setup.
		 *    Same as (1) -- acquire_device serializes.
		 */

		/* 1. Mask hardware IRQ -- no new IRQ work can spawn. */
		if (uwb_drv && uwb_drv->disable_int) {
			uwb_drv->disable_int(uwb_device);
		}

		/* 2. Drain any already-submitted IRQ work. */
		if (uwb_drv && uwb_drv->flush_irq) {
			uwb_drv->flush_irq(uwb_device);
		}

		/* 3. Acquire dev_lock -- waits for any in-flight SPI
		 *    transfer to finish.  The UWB RX/TX threads now
		 *    hold dev_lock during SPI, so this blocks until
		 *    they release it.  The IRQ handler also holds
		 *    dev_lock, drained by steps 1-2 above.  */
		if (uwb_drv && uwb_drv->acquire_device) {
			uwb_drv->acquire_device(uwb_device);
		}

		/* 4. Stop the radio -- safe, we hold dev_lock and IRQ
		 *    is masked, no concurrent SPI possible.
		 *    Use force_trx_off (not disable_txrx) to avoid
		 *    re-enabling GPIO IRQ behind the arbiter's back. */
		if (uwb_drv && uwb_drv->force_trx_off) {
			uwb_drv->force_trx_off(uwb_device);
		}

		/* 5. Release dev_lock before cancel_wait (which may
		 *    unblock threads that need SPI access to clean up). */
		if (uwb_drv && uwb_drv->release_device) {
			uwb_drv->release_device(uwb_device);
		}

		/* 6. Cancel any thread blocked in wait_for_irq. */
		if (uwb_drv && uwb_drv->cancel_wait) {
			uwb_drv->cancel_wait(uwb_device);
		}

		/* 7. Re-enable hardware IRQ. Radio is idle (TRXOFF), so
		 *    no spurious interrupt will fire. */
		if (uwb_drv && uwb_drv->enable_int) {
			uwb_drv->enable_int(uwb_device);
		}
	}
}

bool radio_arbiter_is_available(void)
{
	return atomic_get(&radio_available) != 0;
}
