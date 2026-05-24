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

#define GLOSSY_LF_STACK_SIZE 2048
#define GLOSSY_LF_PRIORITY 5

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

K_THREAD_STACK_DEFINE(glossy_lf_stack, GLOSSY_LF_STACK_SIZE);
static struct k_thread glossy_lf_thread_data;

static void glossy_lf_thread_fn(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  struct deca_glossy_configuration conf = {
      .node_addr = g_node_id,
      .isRoot = g_initiator,
      .guard_period_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US,
      .max_depth = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH,
      .transmission_delay_us =
          CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US,
      .payload = NULL,
      .payload_size = 0,
  };

  struct deca_glossy_result result;

  int64_t clock_offset_ms = 0;
  bool offset_known = false;
  int failures_in_row = 0;
  bool sync_lost = false;

  const int64_t GLOSSY_INTERVAL_MS = 2000;
  const int64_t GLOSSY_FAST_INTERVAL_MS = 20;

  int64_t next_glossy_ms = k_uptime_get();

  LOG_INF("Glossy background thread started; role=%s", g_initiator ? "initiator" : "follower");

  while (1) {
    int64_t now_ms = k_uptime_get();
    int64_t to_glossy_ms = next_glossy_ms - now_ms;

    if (to_glossy_ms > 0) {
      k_sleep(K_MSEC(to_glossy_ms));
    }

    LOG_INF("[glossy] round start: role=%s id=%d",
            g_initiator ? "initiator" : "follower", g_node_id);

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

      clock_offset_ms = new_offset_ms;

      k_mutex_lock(&g_state.mutex, K_FOREVER);
      g_state.clock_offset_ms = clock_offset_ms;
      g_state.offset_known = true;
      g_state.sync_lost = false;
      g_state.failures_in_row = 0;
      k_mutex_unlock(&g_state.mutex);

      LOG_INF("[glossy] sync ok: rtc_offset=%" PRId64
              " ms  dwt_offset=%" PRId64 " ms",
              new_offset_ms, deca_offset_ms);

      if (!offset_known) {
        offset_known = true;
        LOG_INF("[glossy] offset known for the first time");
      }
      if (sync_lost) {
        LOG_INF("[glossy] sync recovered after %d failures", failures_in_row);
        sync_lost = false;
      }
      failures_in_row = 0;
    } else {
      failures_in_row++;
      LOG_WRN("[glossy] round failed (ret=%d, streak=%d)",
              ret, failures_in_row);
      if (failures_in_row >= 3 && !sync_lost) {
        LOG_ERR("[glossy] sync lost -- entering recovery mode");
        sync_lost = true;
      }

      k_mutex_lock(&g_state.mutex, K_FOREVER);
      g_state.sync_lost = sync_lost;
      g_state.failures_in_row = failures_in_row;
      k_mutex_unlock(&g_state.mutex);
    }

    now_ms = k_uptime_get();
    int64_t interval_ms = sync_lost ? GLOSSY_FAST_INTERVAL_MS
                                    : GLOSSY_INTERVAL_MS;

    if (g_initiator) {
      if (sync_lost) {
        next_glossy_ms = now_ms + interval_ms;
      } else {
        next_glossy_ms =
            ((now_ms / GLOSSY_INTERVAL_MS) + 1) * GLOSSY_INTERVAL_MS;
      }
    } else if (offset_known && !sync_lost) {
      int64_t now_initiator = now_ms - clock_offset_ms;
      int64_t next_initiator =
          ((now_initiator / GLOSSY_INTERVAL_MS) + 1) * GLOSSY_INTERVAL_MS;
      next_glossy_ms = next_initiator + clock_offset_ms;
    } else {
      next_glossy_ms = now_ms + interval_ms;
    }
  }
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

  k_thread_create(&glossy_lf_thread_data,
                  glossy_lf_stack,
                  K_THREAD_STACK_SIZEOF(glossy_lf_stack),
                  glossy_lf_thread_fn,
                  NULL, NULL, NULL,
                  GLOSSY_LF_PRIORITY,
                  0,
                  K_NO_WAIT);
  k_thread_name_set(&glossy_lf_thread_data, "glossy_lf");

  LOG_INF("Glossy LF initialized successfully, thread started!");

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