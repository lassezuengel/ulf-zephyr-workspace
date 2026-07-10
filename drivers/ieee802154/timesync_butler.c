/*
 * Copyright (c) 2024 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Butler distributed time synchronization flood primitive.
 * Based on Algorithm 1 from Mager et al. "Butler: Increasing the
 * Availability of Low-Power Wireless Communication Protocols".
 *
 * Extended with sigma's absolute timestamps for clock synchronization.
 */

#include <app/drivers/ieee802154/timesync_butler.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/drivers/ieee802154/uwb_timestamp_utils.h>
#include <app/drivers/debug/timesync_debug_gpio.h>

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/timer/nrf_rtc_timer.h>
#include <zephyr/logging/log.h>

#include <string.h>
#if IS_ENABLED(CONFIG_NODE_TABLE)
#include <app/lib/node_table/node_table.h>
#endif

LOG_MODULE_REGISTER(timesync_butler, LOG_LEVEL_INF);

/* butler_frame struct is now in timesync_butler.h */

static struct butler_resync_seed resync_seed = { .valid = false };

void butler_set_resync_seed(const struct butler_resync_seed *seed)
{
    resync_seed = *seed;
}

static struct butler_flood_config last_butler_config;
static bool last_butler_config_valid = false;

void butler_store_config(const struct butler_flood_config *conf)
{
    last_butler_config = *conf;
    last_butler_config_valid = true;
}

const struct butler_flood_config *butler_get_last_config(void)
{
    return last_butler_config_valid ? &last_butler_config : NULL;
}

/* Semaphore + callback for reading DWT system timestamp at a precise RTC instant */
static K_SEM_DEFINE(butler_read_dwt_sem, 0, 1);
static uint64_t butler_dwt_start_ts = 0;

static void butler_read_deca_ts(int32_t id, uint64_t expire_time, void *user_data)
{
    const struct device *dev = (const struct device *)user_data;
    const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
    if (uwb_driver) {
        butler_dwt_start_ts = uwb_driver->system_timestamp(dev);
    }
    k_sem_give(&butler_read_dwt_sem);
}

/**
 * Roll random TX decisions to find how many consecutive RX slots precede the
 * next TX. Returns 0..remaining (0 = TX this slot, remaining = all RX).
 */
static uint8_t roll_rx_slots_until_tx(uint8_t remaining, uint8_t p_tx_pct)
{
    uint8_t rx_slots = 0;
    while (rx_slots < remaining) {
        if ((sys_rand32_get() % 100) < p_tx_pct) {
            break;
        }
        rx_slots++;
    }
    return rx_slots;
}

int uwb_butler_flood(const struct device *dev,
                     struct butler_flood_config *conf,
                     struct butler_flood_result *result)
{
    int ret = 0;
    uwb_irq_state_e irq_state = UWB_IRQ_ERR;

    const uwb_driver_t *uwb_driver = uwb_driver_get(dev);
    if (!uwb_driver) {
        LOG_ERR("No UWB driver found");
        return -ENODEV;
    }

    uint32_t butler_entry_us = k_cyc_to_us_floor32(k_cycle_get_32());

    if (uwb_driver->acquire_device(dev) != 0) {
        LOG_ERR("Transceiver busy");
        return -EBUSY;
    }

    /* Initialize result */
    memset(result, 0, sizeof(*result));
    result->converged = false;

    uwb_driver->disable_txrx(dev);
    uwb_driver->set_frame_filter(dev, 0, 0);
    uwb_driver->align_double_buffering(dev);
    uwb_driver->setup_preamble_timeout(dev, 0);

    /* Read initial RTC + DWT timestamps for own tau proposal */
    uint32_t own_rtc = k_cycle_get_32() + 2;
    z_nrf_rtc_timer_set(1, own_rtc, butler_read_deca_ts, (void *)dev);
    if (k_sem_take(&butler_read_dwt_sem, K_MSEC(100)) != 0) {
        LOG_ERR("Failed to read initial DWT timestamp");
        ret = -EIO;
        goto cleanup;
    }
    uint64_t own_dwt = butler_dwt_start_ts;

    /* Precompute subslot duration in DWT ticks */
    uwb_ts_t subslot_dwt = uwb_driver->us_to_timestamp(dev, conf->subslot_duration_us);
    uint32_t subslot_rtc_ticks = (uint32_t)((uint64_t)conf->subslot_duration_us *
                                  CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1000000);

    /* Initialize state (Algorithm 1, lines 2-5)
     * Add a startup guard so the first delayed TX isn't already in the past
     * by the time we finish setup and reach start_tx(). */
    uwb_ts_t startup_guard_dwt = uwb_driver->us_to_timestamp(dev, conf->guard_period_us + conf->subslot_duration_us);
    uint32_t startup_guard_rtc = (uint32_t)((uint64_t)(conf->guard_period_us + conf->subslot_duration_us) *
                                  CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1000000);
    uint32_t t_grid_rtc = own_rtc + startup_guard_rtc;
    uint64_t t_grid_dwt = (own_dwt + startup_guard_dwt) & UWB_TS_MASK;
    int16_t t_remaining = conf->max_subslots;

    /* Own tau proposal: tau = t_grid + T_remaining * T_slot */
    uint32_t tau_local_rtc = t_grid_rtc + (uint32_t)t_remaining * subslot_rtc_ticks;
    uint64_t tau_local_dwt = (t_grid_dwt + (uint64_t)t_remaining * subslot_dwt) & UWB_TS_MASK;

    /* Normalized tau for wrap-safe comparison. Subtract own_dwt so all
     * values are relative to Butler entry (~0..200ms range, well within
     * the 17.2s DWT wrap). Raw tau_local_dwt is preserved for results. */
    uint64_t tau_local_dwt_norm = (tau_local_dwt - own_dwt) & UWB_TS_MASK;

    uint16_t sigma = 0;  /* 0 = unsynchronized */
    uint32_t sigma_rtc_at_tau = tau_local_rtc;
    uint64_t sigma_dwt_at_tau = tau_local_dwt;
    bool tx_next = false;

    /* For skew estimation and relay */
    uint64_t sync_sigma_tx_dwt = 0;  /* Sigma's TX timestamp from SYNC frame */
    uint8_t sync_hop_count = 0;      /* Hop count from SYNC frame */
    uint64_t sync_local_rx_dwt = 0;  /* Local DWT at SYNC RX */

    /* Apply resync seed if available (from Butler frame received during MTM).
     * Re-anchor grid to the seed's RX point and relay immediately.
     * This works because the inline call happens within ~500us of RX,
     * so seed timestamps are still fresh enough for delayed TX. */
    if (resync_seed.valid) {
        sigma = resync_seed.sigma_node_id;
        sigma_rtc_at_tau = resync_seed.sigma_rtc_at_tau;
        sigma_dwt_at_tau = resync_seed.sigma_dwt_at_tau;
        t_grid_rtc = resync_seed.local_rx_rtc;
        t_grid_dwt = resync_seed.local_rx_dwt;
        t_remaining = resync_seed.remaining_slots;
        tau_local_rtc = resync_seed.local_rx_rtc +
            (uint32_t)resync_seed.remaining_slots * subslot_rtc_ticks;
        tau_local_dwt = (resync_seed.local_rx_dwt +
            (uint64_t)resync_seed.remaining_slots * subslot_dwt) & UWB_TS_MASK;
        tau_local_dwt_norm = (tau_local_dwt - own_dwt) & UWB_TS_MASK;
        /* Advance grid by one slot and relay (same as normal SYNC) */
        t_grid_rtc += subslot_rtc_ticks;
        t_grid_dwt = (t_grid_dwt + subslot_dwt) & UWB_TS_MASK;
        t_remaining--;
        tx_next = true;
        resync_seed.valid = false;
    }

    /* LOG_WRN("Butler start: max_subslots=%u, subslot=%u us, p_tx=%u%%, own_rtc=%u", */
    /*         conf->max_subslots, conf->subslot_duration_us, conf->p_tx_pct, own_rtc); */

    /* Overhead per subslot: frame-after-preamble + TX kickoff.
     * After preamble timeout, we need margin for: IRQ processing (~50us),
     * grid advance computation (~10us), frame struct build (~10us),
     * setup_tx_frame SPI write (~100us), start_tx call (~10us).
     * Total ~200us minimum. Use 300us for safety. */
    #define BUTLER_FRAME_OVERHEAD_US 800

    /* DW1000 preamble timeout (DRX_PRETOC) counts in PAC units, not microseconds.
     * At PRF 64MHz with PAC8: 1 PAC = 8 preamble symbols * ~1.0us = ~8us per count. */
    #define BUTLER_PAC_DURATION_US 8

    /* Main loop */
    const char *last_action = "init";
    uint16_t total_iterations = 0;
    uint16_t sync_count = 0;
    uint16_t rx_nosync_count = 0;
    uint16_t timeout_count = 0;
    uint16_t tx_count = 0;
    uint8_t nosync_elapsed_min = 255, nosync_elapsed_max = 0;
    uint8_t t_remaining_max = 0;
    while (t_remaining > 0) {
        total_iterations++;

        /* Pre-roll random draws: how many consecutive RX slots before TX? */
        const char *tx_reason = "draw";
        uint8_t rx_slots = roll_rx_slots_until_tx(t_remaining, conf->p_tx_pct);
        if (tx_next) {
            rx_slots = 0;  /* SYNC forces immediate TX */
            tx_reason = "sync";
        }

        /* ---- RX phase: listen for rx_slots worth of time ---- */
        if (rx_slots > 0) {
            uint32_t rx_duration_us = (uint32_t)rx_slots * conf->subslot_duration_us;
            uint32_t timeout_us = (rx_duration_us > BUTLER_FRAME_OVERHEAD_US)
                ? (rx_duration_us - BUTLER_FRAME_OVERHEAD_US) : 1;
            uwb_driver->enable_rx(dev, timeout_us, t_grid_dwt);  /* Frame wait timeout + delayed RX */
            uwb_driver->release_device(dev);
            irq_state = uwb_driver->wait_for_irq(dev);
            uwb_driver->acquire_device(dev);

            if (irq_state == UWB_IRQ_HALF_DELAY_WARNING) {
                /* Delayed RX timestamp was in the past -- HPDWARN */
                uwb_driver->disable_txrx(dev);
                uint64_t now_dwt = uwb_driver->system_timestamp(dev);
                printk("Butler RX HPDWARN: remaining=%d/%u, last=%s, rx_slots=%u, programmed-now=%lld us\n",
                       t_remaining, conf->max_subslots,
                       last_action, rx_slots,
                       DWT_TS_TO_US((int64_t)t_grid_dwt - (int64_t)now_dwt));
                ret = -EIO;
                goto cleanup;
            }

            if (irq_state == UWB_IRQ_RX) {
                uint32_t local_rx_rtc = k_cycle_get_32();

                uint8_t buf[sizeof(struct butler_frame) + FRAME_LENGTH_ADDITIONAL];
                uwb_rx_diagnostics_t rx_diag;
                uint64_t local_rx_dwt = uwb_driver->read_rx_timestamp(dev, &rx_diag);

                uint16_t pkt_len = uwb_driver->get_rx_frame_length(dev);
                bool frame_ok = false;

                if (pkt_len <= sizeof(buf) && pkt_len >= FRAME_LENGTH_ADDITIONAL) {
                    uwb_driver->read_rx_frame(dev, buf, pkt_len, 0);
                    uwb_driver->switch_buffers(dev);

                    if (buf[0] == UWB_BUTLER_FRAME_ID &&
                        pkt_len >= sizeof(struct butler_frame) + FRAME_LENGTH_ADDITIONAL) {
                        struct butler_frame rx_frame;
                        memcpy(&rx_frame, buf, sizeof(rx_frame));

#if IS_ENABLED(CONFIG_NODE_TABLE)
                        /* Register sender in node table for schedule consistency */
                        if (rx_frame.sender_node_id != conf->node_addr) {
                            node_table_update_hop_count(rx_frame.sender_node_id, 1, 0);
                        }
#endif

                        /* Compute received tau in local time (line 17) */
                        uint32_t recv_tau_rtc = local_rx_rtc +
                            (uint32_t)rx_frame.remaining_slots * subslot_rtc_ticks;
                        uint64_t recv_tau_dwt = (local_rx_dwt +
                            (uint64_t)rx_frame.remaining_slots * subslot_dwt) & UWB_TS_MASK;
                        uint64_t recv_tau_dwt_norm = (recv_tau_dwt - own_dwt) & UWB_TS_MASK;

                        /* Already converged to the same sigma? Ignore.
                         * Reference impl line 654: avoids unnecessary resyncs
                         * and TX storms between already-converged nodes. */
                        bool do_sync = false;
                        if (rx_frame.sigma_node_id == sigma && sigma != 0) {
                            /* Same sigma -- nothing to learn, skip */
                        } else if (sigma == 0 || recv_tau_dwt_norm < tau_local_dwt_norm) {
                            do_sync = true;
                        } else if (recv_tau_dwt_norm == tau_local_dwt_norm &&
                                   rx_frame.sigma_node_id < sigma) {
                            do_sync = true;
                        }

                        if (do_sync && rx_frame.remaining_slots <= conf->max_subslots) {
                            /* SYNC (Algorithm 1, lines 24-26) */
                            t_remaining = rx_frame.remaining_slots;
                            t_grid_rtc = local_rx_rtc;
                            t_grid_dwt = local_rx_dwt;
                            tau_local_rtc = recv_tau_rtc;
                            tau_local_dwt = recv_tau_dwt;
                            tau_local_dwt_norm = recv_tau_dwt_norm;
                            sigma = rx_frame.sigma_node_id;
                            sigma_rtc_at_tau = rx_frame.sigma_rtc_at_tau;
                            sigma_dwt_at_tau = from_packed_uwb_ts(rx_frame.sigma_dwt_at_tau);
                            /* Save for relay + skew */
                            sync_sigma_tx_dwt = from_packed_uwb_ts(rx_frame.sigma_tx_dwt);
                            sync_hop_count = rx_frame.hop_count;
                            sync_local_rx_dwt = local_rx_dwt;
                            tx_next = true;

                            LOG_DBG("Butler SYNC: sigma=0x%04x, remaining=%u",
                                    sigma, t_remaining);
                            frame_ok = true;
                        }
                    }
                }
                if (!frame_ok) {
                    uwb_driver->switch_buffers(dev);
                }

                /* After RX without SYNC: advance grid to account for
                 * the wall-clock time consumed. RX was anchored to t_grid_dwt,
                 * so we can compute exactly which slot the frame landed in. */
                if (!tx_next) {
                    uint64_t dwt_delta = (local_rx_dwt - t_grid_dwt) & UWB_TS_MASK;
                    uint8_t elapsed_slots = (uint8_t)(dwt_delta / subslot_dwt) + 2;
                    if (elapsed_slots > t_remaining) {
                        elapsed_slots = t_remaining;
                    }
                    if (elapsed_slots < nosync_elapsed_min) nosync_elapsed_min = elapsed_slots;
                    if (elapsed_slots > nosync_elapsed_max) nosync_elapsed_max = elapsed_slots;
                    if (t_remaining > t_remaining_max) t_remaining_max = t_remaining;
                    t_grid_rtc += elapsed_slots * subslot_rtc_ticks;
                    t_grid_dwt = (t_grid_dwt + (uint64_t)elapsed_slots * subslot_dwt) & UWB_TS_MASK;
                    t_remaining -= elapsed_slots;
                }
                /* If SYNC happened (tx_next=true), grid was re-anchored to RX time.
                 * Advance by one slot so the TX fires one slot after reception. */
                if (tx_next) {
                    t_grid_rtc += subslot_rtc_ticks;
                    t_grid_dwt = (t_grid_dwt + subslot_dwt) & UWB_TS_MASK;
                    t_remaining--;
                }

                if (tx_next) { sync_count++; } else { rx_nosync_count++; }
                last_action = tx_next ? "rx_sync" : "rx_nosync";
                continue;  /* Got a frame -- re-enter loop (TX if synced, or re-roll) */

            } else {
                tx_reason = "rx_timeout";
                timeout_count++;
                last_action = "timeout";
                t_grid_rtc += rx_slots * subslot_rtc_ticks;
                t_grid_dwt = (t_grid_dwt + (uint64_t)rx_slots * subslot_dwt) & UWB_TS_MASK;
                t_remaining -= rx_slots;
                if (t_remaining == 0) {
                    break;
                }
            }
        }

        if (sigma == 0) {
            sigma = conf->node_addr;
        }

        /* TX fires at t_grid_dwt. Receiver gets rx_time ≈ t_grid_dwt.
         * Frame carries t_remaining so receiver computes:
         * recv_tau = rx_time + t_remaining * T_slot ≈ t_grid + t_remaining * T_slot = our tau.
         * After SYNC, receiver sets t_remaining = frame.remaining_slots, then
         * advance decrements by 1 (consuming this slot). */
        struct butler_frame frame = {
            .msg_id = UWB_BUTLER_FRAME_ID,
            .remaining_slots = t_remaining,
            .sender_node_id = conf->node_addr,
            .sigma_node_id = sigma,
            .sigma_rtc_at_tau = sigma_rtc_at_tau,
        };

        to_packed_uwb_ts(frame.sigma_dwt_at_tau, sigma_dwt_at_tau);
        if (sigma == conf->node_addr) {
            /* We are sigma: our own TX timestamp, hop 0 */
            to_packed_uwb_ts(frame.sigma_tx_dwt, t_grid_dwt);
            frame.hop_count = 0;
        } else {
            /* Propagating synced sigma: preserve original TX, hop+1 */
            to_packed_uwb_ts(frame.sigma_tx_dwt, sync_sigma_tx_dwt);
            frame.hop_count = sync_hop_count + 1;
        }

        uwb_driver->disable_txrx(dev);
        uwb_driver->setup_tx_frame(dev, (uint8_t *)&frame, sizeof(frame));
        uwb_driver->start_tx(dev, t_grid_dwt);

        uwb_driver->release_device(dev);
        irq_state = uwb_driver->wait_for_irq(dev);
        uwb_driver->acquire_device(dev);

        tx_next = false;

        if (irq_state == UWB_IRQ_HALF_DELAY_WARNING) {
            uint64_t now_dwt = uwb_driver->system_timestamp(dev);
            int64_t diff_us = DWT_TS_TO_US((int64_t)t_grid_dwt - (int64_t)now_dwt);
            printk("Butler TX HPDWARN: slot=%d/%u, reason=%s, programmed-now=%lld us\n",
                   conf->max_subslots - t_remaining, conf->max_subslots, tx_reason, diff_us);
            uwb_driver->disable_txrx(dev);
            ret = -EIO;
            goto cleanup;
        }
        if (irq_state != UWB_IRQ_TX) {
            printk("Butler TX failed: irq=%d, slot=%d/%u (remaining=%d), reason=%s\n",
                   irq_state, conf->max_subslots - t_remaining, conf->max_subslots,
                   t_remaining, tx_reason);
            uwb_driver->disable_txrx(dev);
            ret = -EIO;
            goto cleanup;
        }

        /* LOG_WRN("Butler TX OK: slot=%u/%u, reason=%s", */
        /*         conf->max_subslots - t_remaining, conf->max_subslots, tx_reason); */
        tx_count++;
        last_action = tx_reason;

        /* Advance grid past this TX slot */
        t_grid_rtc += subslot_rtc_ticks;
        t_grid_dwt = (t_grid_dwt + subslot_dwt) & UWB_TS_MASK;
        t_remaining--;
        if (t_remaining < 0) {
            LOG_ERR("Butler t_remaining went negative: %d", t_remaining);
            break;
        }
    }


    /* Compute result clock pairs (for time_sync_update) */
    if (sigma != 0) {
        /* Both sides describe the same physical instant: tau */
        result->rtc_clock_pair.local = tau_local_rtc;
        result->rtc_clock_pair.ref = sigma_rtc_at_tau;
        result->deca_clock_pair.local = tau_local_dwt;
        result->deca_clock_pair.ref = sigma_dwt_at_tau;
        result->sigma_node_id = sigma;
        result->converged = true;
    } else {
        /* No sync happened -- alone or all collisions */
        result->sigma_node_id = 0;
        result->converged = false;
    }

    LOG_DBG("Butler done: converged=%d, sigma=0x%04x", result->converged, sigma);

    /* --- Post-convergence beacon: sigma transmits one final frame --- */
    /* Non-sigma nodes listen. Both record fresh RTC+DWT timestamps.
     * This gives two time points (tau and now) for skew estimation. */
    if (sigma != 0 && sigma == conf->node_addr) {
        /* We are sigma -- transmit beacon */
        uint64_t now_dwt = uwb_driver->system_timestamp(dev);
        uint64_t tx_ts = (now_dwt + uwb_driver->us_to_timestamp(dev, 1000)) & UWB_TS_MASK;
        uint32_t beacon_rtc = k_cycle_get_32();
        struct butler_frame beacon = {
            .msg_id = UWB_BUTLER_FRAME_ID,
            .remaining_slots = 0,  /* 0 = post-convergence beacon */
            .sender_node_id = conf->node_addr,
            .sigma_node_id = sigma,
            .sigma_rtc_at_tau = beacon_rtc,
            .hop_count = 0,  /* Direct from sigma */
        };
        to_packed_uwb_ts(beacon.sigma_dwt_at_tau, tx_ts);
        to_packed_uwb_ts(beacon.sigma_tx_dwt, tx_ts);
        uwb_driver->disable_txrx(dev);
        uwb_driver->setup_tx_frame(dev, (uint8_t *)&beacon, sizeof(beacon));
        uwb_driver->start_tx(dev, tx_ts);
        uwb_driver->release_device(dev);
        irq_state = uwb_driver->wait_for_irq(dev);
        uwb_driver->acquire_device(dev);
        if (irq_state == UWB_IRQ_TX) {
            result->beacon_rtc = beacon_rtc;
            result->beacon_dwt = tx_ts;
            result->has_beacon = true;
        }
    } else if (sigma != 0) {
        /* Non-sigma: listen for beacon */
        uwb_driver->disable_txrx(dev);
        uwb_driver->enable_rx(dev, 3000, 0);  /* 3ms timeout, immediate RX */
        uwb_driver->release_device(dev);
        irq_state = uwb_driver->wait_for_irq(dev);
        uwb_driver->acquire_device(dev);
        if (irq_state == UWB_IRQ_RX) {
            uint32_t local_beacon_rtc = k_cycle_get_32();
            uint64_t local_beacon_dwt = uwb_driver->read_rx_timestamp(dev, NULL);
            uint8_t bbuf[sizeof(struct butler_frame) + FRAME_LENGTH_ADDITIONAL];
            uint16_t blen = uwb_driver->get_rx_frame_length(dev);
            if (blen >= sizeof(struct butler_frame) + FRAME_LENGTH_ADDITIONAL) {
                uwb_driver->read_rx_frame(dev, bbuf, blen, 0);
                struct butler_frame *bframe = (struct butler_frame *)bbuf;
                if (bframe->msg_id == UWB_BUTLER_FRAME_ID && bframe->remaining_slots == 0) {
                    if (bframe->sigma_node_id != sigma) {
                        LOG_ERR("Beacon sigma mismatch: expected 0x%04x, got 0x%04x",
                                sigma, bframe->sigma_node_id);
                    }
                    result->beacon_rtc = local_beacon_rtc;
                    result->beacon_dwt = local_beacon_dwt;
                    result->sigma_beacon_rtc = bframe->sigma_rtc_at_tau;
                    result->sigma_beacon_dwt = from_packed_uwb_ts(bframe->sigma_tx_dwt);
                    /* SYNC frame: sigma's original TX + local RX + hop count */
                    result->sync_local_rx_dwt = sync_local_rx_dwt;
                    result->sync_sigma_tx_dwt = sync_sigma_tx_dwt;
                    result->sync_hop_count = sync_hop_count;
                    result->has_beacon = (sync_local_rx_dwt != 0 && sync_sigma_tx_dwt != 0);
                }
            }
            uwb_driver->switch_buffers(dev);
        }
    }

cleanup:
    uwb_driver->setup_preamble_timeout(dev, 0);
    uwb_driver->release_device(dev);
    uint32_t butler_exit_us = k_cyc_to_us_floor32(k_cycle_get_32());
    LOG_DBG("Butler wall: %u us, iter=%u, sync=%u, nosync=%u, timeout=%u, tx=%u, "
            "elapsed_min=%u, elapsed_max=%u, t_rem_max=%u",
            butler_exit_us - butler_entry_us,
            total_iterations, sync_count, rx_nosync_count, timeout_count, tx_count,
            nosync_elapsed_min, nosync_elapsed_max, t_remaining_max);
    k_yield();
    return ret;
}
