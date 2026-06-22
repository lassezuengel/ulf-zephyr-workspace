#include <app/drivers/ieee802154/glossy_lf.h>

#include <errno.h>
#include <inttypes.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>

LOG_MODULE_REGISTER(glossy_lf, LOG_LEVEL_INF);

#ifndef CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US
#define CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US 2000
#endif

#ifndef CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH
#define CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH 6
#endif

#ifndef CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US
#define CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US 1250
#endif

#define GLOSSY_INTERVAL_MS 2000
#define GLOSSY_SYNC_LOST_FAILURES 5
#define GLOSSY_RECOVERY_MIN_RADIUS_MS 2
#define GLOSSY_RECOVERY_MAX_RADIUS_MS 16

static int64_t glossy_flood_latency_ms(void) {
  int64_t latency_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US +
                       (int64_t)CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH *
                           CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US;
  return (latency_us + 999) / 1000;
}

struct glossy_lf_sync_state {
  struct k_mutex mutex;
  int64_t clock_offset_ms;
  bool offset_known;
  bool sync_lost;
  int failures_in_row;
  ExternalClockSyncResultCallback result_callback;
  void *result_callback_user_data;
};

static struct glossy_lf_sync_state g_state = {
    .clock_offset_ms = 0,
    .offset_known = false,
    .sync_lost = false,
    .failures_in_row = 0,
    .result_callback = NULL,
    .result_callback_user_data = NULL,
};

static const struct device *g_dev;
static bool g_initiator;
static bool g_started;
static int g_node_id;

static int64_t glossy_recovery_radius_ms(int failures_in_row) {
  if (failures_in_row < GLOSSY_SYNC_LOST_FAILURES) {
    return 0;
  }

  int recovery_round = failures_in_row - GLOSSY_SYNC_LOST_FAILURES;
  int64_t radius_ms = GLOSSY_RECOVERY_MIN_RADIUS_MS;

  while (recovery_round-- > 0 && radius_ms < GLOSSY_RECOVERY_MAX_RADIUS_MS) {
    radius_ms *= 2;
  }

  return radius_ms > GLOSSY_RECOVERY_MAX_RADIUS_MS
             ? GLOSSY_RECOVERY_MAX_RADIUS_MS
             : radius_ms;
}

static int64_t glossy_next_boundary_ms(int64_t initiator_time_ms) {
  int64_t quotient = initiator_time_ms / GLOSSY_INTERVAL_MS;
  int64_t remainder = initiator_time_ms % GLOSSY_INTERVAL_MS;

  if (remainder < 0) {
    quotient--;
  }

  return (quotient + 1) * GLOSSY_INTERVAL_MS;
}

static int64_t glossy_next_follower_run_ms(int64_t now_ms,
                                           int64_t clock_offset_ms,
                                           int64_t lead_ms) {
  int64_t boundary_after_start =
      glossy_next_boundary_ms(now_ms - clock_offset_ms + lead_ms);

  return boundary_after_start + clock_offset_ms - lead_ms;
}

int glossy_lf_init(bool initiator, int node_id) {
  LOG_INF("Initializing Glossy LF: role=%s node_id=%d",
          initiator ? "initiator" : "follower", node_id);

  if (g_started) {
    return -EALREADY;
  }

  g_node_id = node_id;
  g_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));
  if (!device_is_ready(g_dev)) {
    LOG_ERR("UWB device not ready");
    return -ENODEV;
  }

  g_initiator = initiator;
  k_mutex_init(&g_state.mutex);

  g_started = true;

  LOG_INF("Glossy LF initialized successfully");

  return 0;
}

void glossy_lf_get_state(struct glossy_lf_state *state) {
  if (!state) {
    return;
  }

  k_mutex_lock(&g_state.mutex, K_FOREVER);
  state->clock_offset_ms = g_state.clock_offset_ms;
  state->offset_known = g_state.offset_known;
  state->sync_lost = g_state.sync_lost;
  state->failures_in_row = g_state.failures_in_row;
  k_mutex_unlock(&g_state.mutex);
}

int lf_clock_sync_init(bool grandmaster, int federate_id,
                       ExternalClockSyncResultCallback result_callback,
                       void *result_callback_user_data,
                       bool *lf_drives_sync_schedule) {
  if (!result_callback || !lf_drives_sync_schedule) {
    return -EINVAL;
  }

  int ret = glossy_lf_init(grandmaster, federate_id);
  if (ret != 0 && ret != -EALREADY) {
    return ret;
  }

  k_mutex_lock(&g_state.mutex, K_FOREVER);
  g_state.clock_offset_ms = 0;
  g_state.offset_known = false;
  g_state.sync_lost = false;
  g_state.failures_in_row = 0;
  g_state.result_callback = result_callback;
  g_state.result_callback_user_data = result_callback_user_data;
  k_mutex_unlock(&g_state.mutex);

  /* Glossy has no private worker or timer; reactor-uc schedules every round. */
  *lf_drives_sync_schedule = true;

  LOG_INF("[glossy] external clock sync initialized");

  return 0;
}

int lf_clock_sync_schedule(void) {
  if (!g_started) {
    return -EAGAIN;
  }

  ExternalClockSyncResultCallback result_callback;
  void *result_callback_user_data;

  k_mutex_lock(&g_state.mutex, K_FOREVER);
  result_callback = g_state.result_callback;
  result_callback_user_data = g_state.result_callback_user_data;
  k_mutex_unlock(&g_state.mutex);

  if (!result_callback) {
    return -EINVAL;
  }

  int failures_before_round;
  bool offset_known_before_round;

  k_mutex_lock(&g_state.mutex, K_FOREVER);
  failures_before_round = g_state.failures_in_row;
  offset_known_before_round = g_state.offset_known;
  k_mutex_unlock(&g_state.mutex);

  int64_t recovery_radius_ms =
      (!g_initiator && offset_known_before_round)
          ? glossy_recovery_radius_ms(failures_before_round)
          : 0;
  uint32_t rx_window_us = 0;
  if (!g_initiator && !offset_known_before_round) {
    rx_window_us = (GLOSSY_INTERVAL_MS + glossy_flood_latency_ms()) * 1000;
  } else if (recovery_radius_ms > 0) {
    rx_window_us =
        (2 * recovery_radius_ms + glossy_flood_latency_ms()) * 1000;
  }

  struct deca_glossy_configuration conf = {
      .node_addr = g_node_id,
      .isRoot = g_initiator,
      .measure_constant_delay = false, // We do not need constant delay measurement for LF, and it adds extra time to the flood round
      .guard_period_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US,
      .max_depth = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH,
      .rx_window_us = rx_window_us,
      .transmission_delay_us =
          CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US,
      .payload = NULL,
      .payload_size = 0,
  };

  struct deca_glossy_result result;

  LOG_DBG("[glossy] round start: role=%s id=%d depth=%d radius=%" PRId64
          " ms window=%u us",
          g_initiator ? "initiator" : "follower", g_node_id, conf.max_depth,
          recovery_radius_ms, conf.rx_window_us);

  if (g_initiator) {
    LOG_INF("[glossy] initiator starting round");
  }

  int ret = deca_glossy_time_synchronization(g_dev, &conf, &result);

  if (ret == 0) {
    int32_t rtc_offset_ticks =
        (int32_t)((uint32_t)result.rtc_clock_pair.local -
                  (uint32_t)result.rtc_clock_pair.ref);
    int64_t deca_offset_ticks = (int64_t)result.deca_clock_pair.local -
                                (int64_t)result.deca_clock_pair.ref;

    int64_t new_offset_ms = ((int64_t)rtc_offset_ticks * 1000LL) /
                            CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int64_t deca_offset_ms =
        (int64_t)DWT_TS_TO_US(
            (uint64_t)(deca_offset_ticks < 0 ? -deca_offset_ticks
                                             : deca_offset_ticks)) /
        1000LL;
    if (deca_offset_ticks < 0) {
      deca_offset_ms = -deca_offset_ms;
    }

    k_mutex_lock(&g_state.mutex, K_FOREVER);
    g_state.clock_offset_ms = new_offset_ms;
    g_state.offset_known = true;
    g_state.sync_lost = false;
    g_state.failures_in_row = 0;
    k_mutex_unlock(&g_state.mutex);

    LOG_INF("[glossy] sync ok: hops=%u rtc_offset=%" PRId64
            " ms  dwt_offset=%" PRId64 " ms",
            result.dist_to_root, new_offset_ms, deca_offset_ms);
  } else if (ret != -EBUSY) {
    int failures_in_row;
    bool sync_lost;

    k_mutex_lock(&g_state.mutex, K_FOREVER);
    g_state.failures_in_row++;
    failures_in_row = g_state.failures_in_row;
    sync_lost = g_state.sync_lost;
    if (failures_in_row >= GLOSSY_SYNC_LOST_FAILURES && !sync_lost) {
      sync_lost = true;
    }
    g_state.sync_lost = sync_lost;
    k_mutex_unlock(&g_state.mutex);

    if (failures_in_row < GLOSSY_SYNC_LOST_FAILURES ||
        failures_in_row % GLOSSY_SYNC_LOST_FAILURES == 0) {
      LOG_WRN("[glossy] round failed (ret=%d, streak=%d)",
              ret, failures_in_row);
    }
    if (sync_lost && failures_in_row == GLOSSY_SYNC_LOST_FAILURES) {
      LOG_ERR("[glossy] sync lost -- entering recovery mode");
    }
  } else {
    LOG_WRN("[glossy] round skipped: UWB radio busy");
  }

  int64_t measured_clock_offset_ms;
  bool offset_known;
  bool sync_lost;

  k_mutex_lock(&g_state.mutex, K_FOREVER);
  measured_clock_offset_ms = g_state.clock_offset_ms;
  offset_known = g_state.offset_known;
  sync_lost = g_state.sync_lost;
  int failures_in_row = g_state.failures_in_row;
  k_mutex_unlock(&g_state.mutex);

  int64_t now_ms = k_uptime_get();
  int64_t next_sync_run_ms;

  if (g_initiator) {
    next_sync_run_ms = glossy_next_boundary_ms(now_ms);
  } else if (offset_known) {
    int64_t next_radius_ms =
        sync_lost ? glossy_recovery_radius_ms(failures_in_row) : 0;

    next_sync_run_ms = glossy_next_follower_run_ms(
        now_ms, measured_clock_offset_ms, next_radius_ms);

    if (sync_lost) {
      LOG_WRN("[glossy] recovery window: next=%" PRId64
              " now=%" PRId64 " radius=%" PRId64 " ms",
              next_sync_run_ms, now_ms, next_radius_ms);
    }
  } else {
    next_sync_run_ms = now_ms + GLOSSY_INTERVAL_MS;
  }

  result_callback(result_callback_user_data, ret, next_sync_run_ms,
                  measured_clock_offset_ms);

  /* The synchronous request was accepted; ret is the synchronization result. */
  return 0;
}
