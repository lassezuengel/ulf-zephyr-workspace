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
#define CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US 750
#endif

#define GLOSSY_INTERVAL_MS 2000
#define GLOSSY_SYNC_LOST_FAILURES 10
#define GLOSSY_INITIAL_SEARCH_DEPTH 300
#define GLOSSY_RECOVERY_MIN_RADIUS_MS 20
#define GLOSSY_RECOVERY_MAX_RADIUS_MS 500

struct glossy_lf_sync_state {
  struct k_mutex mutex;
  int64_t clock_offset_ms;
  bool offset_known;
  bool sync_lost;
  int failures_in_row;
};

static struct glossy_lf_sync_state g_state = {
    .clock_offset_ms = 0,
    .offset_known = false,
    .sync_lost = false,
    .failures_in_row = 0,
};

static const struct device *g_dev;
static bool g_initiator;
static bool g_started;
static int g_node_id;
static bool g_initial_sync_tried;

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

static uint16_t glossy_depth_for_search_window_ms(int64_t search_window_ms) {
  const int64_t tx_delay_us =
      CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US;
  const int64_t guard_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US;
  const int64_t target_us = search_window_ms * 1000;

  uint16_t depth = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH;

  while ((int64_t)depth * ((int64_t)depth * tx_delay_us + guard_us) <
             target_us &&
         depth < GLOSSY_INITIAL_SEARCH_DEPTH) {
    depth++;
  }

  return depth;
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
  g_initial_sync_tried = false;
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

int lf_clock_sync_init(bool grandmaster, int federate_id) {
  int ret = glossy_lf_init(grandmaster, federate_id);
  if (ret != 0 && ret != -EALREADY) {
    return ret;
  }

  k_mutex_lock(&g_state.mutex, K_FOREVER);
  g_state.clock_offset_ms = 0;
  g_state.offset_known = false;
  g_state.sync_lost = false;
  g_state.failures_in_row = 0;
  k_mutex_unlock(&g_state.mutex);

  LOG_INF("[glossy] external clock sync initialized");

  return 0;
}

int lf_clock_sync_schedule(int64_t *next_sync_run_ms, int64_t *clock_offset_ms) {
  if (!g_started) {
    return -EAGAIN;
  }

  if (!next_sync_run_ms || !clock_offset_ms) {
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
  uint16_t max_depth = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH;

  if (!g_initial_sync_tried) {
    max_depth = GLOSSY_INITIAL_SEARCH_DEPTH;
  } else if (recovery_radius_ms > 0) {
    max_depth = glossy_depth_for_search_window_ms(2 * recovery_radius_ms);
  }

  struct deca_glossy_configuration conf = {
      .node_addr = g_node_id,
      .isRoot = g_initiator,
      .measure_constant_delay = false, // We do not need constant delay measurement for LF, and it adds extra time to the flood round
      .guard_period_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US,
      .max_depth = max_depth,
      .transmission_delay_us =
          CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US,
      .payload = NULL,
      .payload_size = 0,
  };

  g_initial_sync_tried = true;

  struct deca_glossy_result result;

  LOG_DBG("[glossy] round start: role=%s id=%d depth=%d radius=%" PRId64
          " ms",
          g_initiator ? "initiator" : "follower", g_node_id, conf.max_depth,
          recovery_radius_ms);

  if(g_initiator) {
    LOG_INF("[glossy] initiator starting round");
  }

  int ret = deca_glossy_time_synchronization(g_dev, &conf, &result);

  if (ret == 0) {
    int64_t rtc_offset_ticks = (int64_t)result.rtc_clock_pair.local -
                               (int64_t)result.rtc_clock_pair.ref;
    int64_t deca_offset_ticks = (int64_t)result.deca_clock_pair.local -
                                (int64_t)result.deca_clock_pair.ref;

    int64_t new_offset_ms = (rtc_offset_ticks * 1000LL) / 32768LL;
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
  } else {
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

  if (g_initiator) {
    *next_sync_run_ms = glossy_next_boundary_ms(now_ms);
  } else if (offset_known) {
    int64_t next_radius_ms =
        sync_lost ? glossy_recovery_radius_ms(failures_in_row) : 0;

    *next_sync_run_ms = glossy_next_follower_run_ms(
        now_ms, measured_clock_offset_ms, next_radius_ms);

    if (next_radius_ms > 0) {
      LOG_WRN("[glossy] recovery window: next=%" PRId64
              " now=%" PRId64 " radius=%" PRId64 " ms",
              *next_sync_run_ms, now_ms, next_radius_ms);
    }
  } else {
    *next_sync_run_ms = now_ms + GLOSSY_INTERVAL_MS;
  }

  *clock_offset_ms = measured_clock_offset_ms;
  return ret;
}
