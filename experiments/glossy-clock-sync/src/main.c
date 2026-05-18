#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/poll.h>
#include <zephyr/sys/printk.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>

LOG_MODULE_REGISTER(glossy_sync, LOG_LEVEL_INF);

/* ========================================================================== */
/* Config defaults                                                             */
/* ========================================================================== */

#ifndef CONFIG_GLOSSY_NODE_ID
#define GLOSSY_NODE_ID 1
#else
#define GLOSSY_NODE_ID CONFIG_GLOSSY_NODE_ID
#endif

#ifndef CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US
#define CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US 2000
#endif

#ifndef CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH
#define CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH 6
#endif

#ifndef CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US
#define CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US 750
#endif

#define PORT 4242
#define RECV_BUF_SIZE 128

/* UDP RX thread stack / priority */
#define UDP_RX_STACK_SIZE 2048
#define UDP_RX_PRIORITY 5

/* ========================================================================== */
/* Shared synchronisation state                                                */
/* ========================================================================== */

/*
 * Written by the main thread after every successful Glossy round.
 * Read by the UDP RX thread for every incoming packet.
 * Protected by sync_state_mutex.
 *
 * clock_offset_ms = follower_local_ms - initiator_local_ms
 *   (positive when follower clock is ahead)
 */
struct sync_state {
  struct k_mutex mutex;
  int64_t clock_offset_ms;
  bool offset_known;
};

static struct sync_state g_sync = {
    .clock_offset_ms = 0,
    .offset_known = false,
};

/* ========================================================================== */
/* UDP helpers                                                                 */
/* ========================================================================== */

static int udp_open_socket(const char *bind_addr_str) {
  struct sockaddr_in6 addr = {
      .sin6_family = AF_INET6,
      .sin6_port = htons(PORT),
  };
  inet_pton(AF_INET6, bind_addr_str, &addr.sin6_addr);

  int sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    LOG_ERR("socket() failed: %d", errno);
    return -1;
  }
  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    LOG_ERR("bind() failed: %d", errno);
    close(sock);
    return -1;
  }
  return sock;
}

static void udp_send_timestamp(int sock, const char *peer_addr_str) {
  struct sockaddr_in6 peer = {
      .sin6_family = AF_INET6,
      .sin6_port = htons(PORT),
  };
  inet_pton(AF_INET6, peer_addr_str, &peer.sin6_addr);

  int64_t send_ms = k_uptime_get();
  char buf[RECV_BUF_SIZE];
  int len = snprintf(buf, sizeof(buf), "TS %" PRId64, send_ms);

  ssize_t sent = sendto(sock, buf, len, 0,
                        (struct sockaddr *)&peer, sizeof(peer));
  if (sent < 0) {
    LOG_ERR("UDP TX failed: %d", errno);
  } else {
    LOG_INF("UDP TX: send_ms=%" PRId64, send_ms);
  }
}

/* ========================================================================== */
/* UDP RX thread                                                               */
/* ========================================================================== */

/*
 * Packet format: "TS <sender_uptime_ms>"
 *
 * One-way delay estimate (follower side):
 *
 *   clock_offset_ms = follower_local_ms - initiator_local_ms
 *                     (positive when follower clock is ahead)
 *
 *   delay_ms = arrival_local_ms - (sender_local_ms + clock_offset_ms)
 */

struct udp_rx_thread_args {
  int sock;
};

static struct udp_rx_thread_args g_udp_rx_args;

K_THREAD_STACK_DEFINE(udp_rx_stack, UDP_RX_STACK_SIZE);
static struct k_thread udp_rx_thread_data;

static void udp_rx_thread_fn(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  struct udp_rx_thread_args *args = (struct udp_rx_thread_args *)arg1;
  int sock = args->sock;

  char buf[RECV_BUF_SIZE];
  struct sockaddr_in6 from;
  socklen_t fromlen;

  LOG_INF("[udp_rx] thread started");

  while (1) {
    fromlen = sizeof(from);

    /* Blocking recvfrom — no timeout; runs until a packet arrives */
    ssize_t len = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                           (struct sockaddr *)&from, &fromlen);
    if (len < 0) {
      LOG_ERR("[udp_rx] recvfrom() failed: %d", errno);
      /* Brief back-off to avoid tight error loops */
      k_sleep(K_MSEC(10));
      continue;
    }
    buf[len] = '\0';

    int64_t arrival_ms = k_uptime_get();

    int64_t sender_ms;
    if (sscanf(buf, "TS %" SCNd64, &sender_ms) != 1) {
      LOG_WRN("[udp_rx] malformed packet: %s", buf);
      continue;
    }

    /* Snapshot shared sync state */
    int64_t clock_offset_ms;
    bool offset_known;

    k_mutex_lock(&g_sync.mutex, K_FOREVER);
    clock_offset_ms = g_sync.clock_offset_ms;
    offset_known = g_sync.offset_known;
    k_mutex_unlock(&g_sync.mutex);

    if (!offset_known) {
      LOG_INF("[udp_rx] sender_ms=%" PRId64 " arrival_ms=%" PRId64
              " (offset unknown, cannot compute delay)",
              sender_ms, arrival_ms);
      continue;
    }

    int64_t delay_ms = arrival_ms - (sender_ms + clock_offset_ms);
    LOG_INF("[udp_rx] sender_ms=%" PRId64 " arrival_ms=%" PRId64
            " offset_ms=%" PRId64 " => one_way_delay_ms=%" PRId64,
            sender_ms, arrival_ms, clock_offset_ms, delay_ms);
  }
}

/* ========================================================================== */
/* main()                                                                      */
/* ========================================================================== */

int main(void) {
  for (int i = 0; i < 3; i++) {
    printk("System alive...\n");
    k_sleep(K_MSEC(2000));
  }

  const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));
  if (!device_is_ready(dev)) {
    LOG_ERR("UWB device not ready");
    return -ENODEV;
  }

  const bool is_initiator = IS_ENABLED(CONFIG_GLOSSY_IS_INITIATOR);

  LOG_INF("Glossy+UDP started; node_id=%d role=%s",
          GLOSSY_NODE_ID, is_initiator ? "initiator" : "follower");

  /* ---------------------------------------------------------------------- */
  /* Shared state init                                                       */
  /* ---------------------------------------------------------------------- */

  k_mutex_init(&g_sync.mutex);

  /* ---------------------------------------------------------------------- */
  /* UDP socket                                                              */
  /* ---------------------------------------------------------------------- */

  int sock = udp_open_socket(CONFIG_NET_CONFIG_MY_IPV6_ADDR);
  if (sock < 0) {
    LOG_ERR("Failed to open UDP socket, aborting");
    return -EIO;
  }

  /* ---------------------------------------------------------------------- */
  /* Start continuous UDP RX thread (follower only; harmless on initiator)  */
  /* ---------------------------------------------------------------------- */

  g_udp_rx_args.sock = sock;

  k_thread_create(&udp_rx_thread_data,
                  udp_rx_stack,
                  K_THREAD_STACK_SIZEOF(udp_rx_stack),
                  udp_rx_thread_fn,
                  &g_udp_rx_args, NULL, NULL,
                  UDP_RX_PRIORITY,
                  0,
                  K_NO_WAIT);
  k_thread_name_set(&udp_rx_thread_data, "udp_rx");

  /* ---------------------------------------------------------------------- */
  /* Glossy configuration                                                    */
  /* ---------------------------------------------------------------------- */

  struct deca_glossy_configuration conf = {
      .node_addr = GLOSSY_NODE_ID,
      .isRoot = is_initiator,
      .guard_period_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US,
      .max_depth = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH,
      .transmission_delay_us =
          CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US,
      .payload = NULL,
      .payload_size = 0,
  };

  struct deca_glossy_result result;

  /* ---------------------------------------------------------------------- */
  /* Synchronisation state (main-thread copy, written under mutex)          */
  /* ---------------------------------------------------------------------- */

  int64_t clock_offset_ms = 0;
  bool offset_known = false;
  int failures_in_row = 0;
  bool sync_lost = false;

  /* ---------------------------------------------------------------------- */
  /* Scheduling                                                              */
  /* ---------------------------------------------------------------------- */

  const int64_t GLOSSY_INTERVAL_MS = 2000;
  const int64_t GLOSSY_FAST_INTERVAL_MS = 20;

  /*
   * Initiator sends a UDP timestamp halfway through each Glossy interval
   * so the follower's RX thread receives it well before the next round.
   * INT64_MAX = "not yet armed".
   */
  int64_t next_glossy_ms = k_uptime_get();
  int64_t next_udp_tx_ms = INT64_MAX; /* initiator only */

  /* ---------------------------------------------------------------------- */
  /* Event loop                                                              */
  /* ---------------------------------------------------------------------- */

  while (1) {
    int64_t now_ms = k_uptime_get();
    int64_t to_glossy_ms = next_glossy_ms - now_ms;
    int64_t to_udp_ms = next_udp_tx_ms - now_ms;

    /* Sleep until the nearest due event */
    if (to_glossy_ms > 0 && to_udp_ms > 0) {
      k_sleep(K_MSEC(MIN(to_glossy_ms, to_udp_ms)));
    }

    now_ms = k_uptime_get();
    to_glossy_ms = next_glossy_ms - now_ms;
    to_udp_ms = next_udp_tx_ms - now_ms;

    /* ------------------------------------------------------------------ */
    /* Initiator UDP TX                                                    */
    /* ------------------------------------------------------------------ */

    if (is_initiator && to_udp_ms <= 0) {
      LOG_INF("[udp_tx] sending timestamp: id=%d offset_ms=%" PRId64,
              GLOSSY_NODE_ID, clock_offset_ms);
      udp_send_timestamp(sock, CONFIG_NET_CONFIG_PEER_IPV6_ADDR);
      next_udp_tx_ms = INT64_MAX; /* disarmed; re-armed after next Glossy */
    }

    /* ------------------------------------------------------------------ */
    /* Glossy round                                                        */
    /* ------------------------------------------------------------------ */

    if (to_glossy_ms <= 0) {
      LOG_INF("[glossy] round start: role=%s id=%d",
              is_initiator ? "initiator" : "follower", GLOSSY_NODE_ID);

      /*
       * deca_glossy_time_synchronization() acquires the UWB broker
       * lease internally.  The UDP RX thread will block in recvfrom()
       * during this time and resume automatically once the lease is
       * released on return.
       */
      int ret = deca_glossy_time_synchronization(dev, &conf, &result);

      if (ret == 0) {
        int64_t rtc_offset_ticks = (int64_t)result.rtc_clock_pair.local - (int64_t)result.rtc_clock_pair.ref;
        int64_t deca_offset_ticks = (int64_t)result.deca_clock_pair.local - (int64_t)result.deca_clock_pair.ref;

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

        /* Publish updated offset to the RX thread */
        k_mutex_lock(&g_sync.mutex, K_FOREVER);
        g_sync.clock_offset_ms = clock_offset_ms;
        g_sync.offset_known = true;
        k_mutex_unlock(&g_sync.mutex);

        LOG_INF("[glossy] sync ok: rtc_offset=%" PRId64
                " ms  dwt_offset=%" PRId64 " ms",
                new_offset_ms, deca_offset_ms);

        if (!offset_known) {
          offset_known = true;
          LOG_INF("[glossy] offset known for the first time");
        }
        if (sync_lost) {
          LOG_INF("[glossy] sync recovered after %d failures",
                  failures_in_row);
          sync_lost = false;
        }
        failures_in_row = 0;
      } else {
        failures_in_row++;
        LOG_WRN("[glossy] round failed (ret=%d, streak=%d)",
                ret, failures_in_row);
        if (failures_in_row >= 3 && !sync_lost) {
          LOG_ERR("[glossy] sync lost — entering recovery mode");
          sync_lost = true;
        }
        LOG_WRN("[glossy] round done: ret=%d", ret);
      }

      /* Schedule next Glossy round */
      now_ms = k_uptime_get();
      int64_t interval_ms = sync_lost ? GLOSSY_FAST_INTERVAL_MS
                                      : GLOSSY_INTERVAL_MS;

      if (is_initiator) {
        if (sync_lost) {
          next_glossy_ms = now_ms + interval_ms;
        } else {
          /* Snap to the global 2-second grid */
          next_glossy_ms =
              ((now_ms / GLOSSY_INTERVAL_MS) + 1) * GLOSSY_INTERVAL_MS;
        }
      } else if (offset_known && !sync_lost) {
        /* Align to initiator's 2-second grid via known offset */
        int64_t now_initiator = now_ms - clock_offset_ms;
        int64_t next_initiator =
            ((now_initiator / GLOSSY_INTERVAL_MS) + 1) * GLOSSY_INTERVAL_MS;
        next_glossy_ms = next_initiator + clock_offset_ms;
      } else {
        next_glossy_ms = now_ms + interval_ms;
      }

      /*
       * Arm the initiator TX halfway through the next Glossy interval.
       * Both nodes use initiator-aligned time for scheduling so the
       * follower's RX thread sees the packet well before the next round.
       * Skip during recovery (20 ms window is too narrow).
       */
      if (is_initiator && !sync_lost) {
        next_udp_tx_ms = next_glossy_ms - (GLOSSY_INTERVAL_MS / 2);
      } else {
        next_udp_tx_ms = INT64_MAX;
      }
    }
  }

  close(sock);
  return 0;
}