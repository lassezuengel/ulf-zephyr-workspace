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

/* ========================================================================== */
/* UDP helpers                                                                 */
/* ========================================================================== */

/*
 * Packet format: "TS <sender_uptime_ms>"
 *
 * Transmission delay estimate (follower side):
 *
 *   clock_offset_ms = follower_local_ms - initiator_local_ms
 *                     (positive when follower clock is ahead)
 *
 *   delay_ms = arrival_local_ms - (sender_local_ms + clock_offset_ms)
 */

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

/*
 * Wait up to timeout_ms for one UDP packet, then return.
 * Computes and logs the delay estimate when offset is known.
 */
static void udp_recv_one(int sock, bool offset_known, int64_t clock_offset_ms,
                         int timeout_ms) {
  struct timeval tv = {
      .tv_sec = timeout_ms / 1000,
      .tv_usec = (timeout_ms % 1000) * 1000,
  };
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  char buf[RECV_BUF_SIZE];
  struct sockaddr_in6 from;
  socklen_t fromlen = sizeof(from);

  ssize_t len = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&from, &fromlen);
  if (len < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      LOG_WRN("UDP RX: timeout, no packet received");
    } else {
      LOG_ERR("recvfrom() failed: %d", errno);
    }
    return;
  }
  buf[len] = '\0';

  int64_t sender_ms;
  if (sscanf(buf, "TS %" SCNd64, &sender_ms) != 1) {
    LOG_WRN("UDP RX: malformed packet: %s", buf);
    return;
  }

  int64_t arrival_ms = k_uptime_get();

  if (!offset_known) {
    LOG_INF("UDP RX: sender_ms=%" PRId64 " arrival_ms=%" PRId64
            " (offset unknown, cannot compute delay)",
            sender_ms, arrival_ms);
    return;
  }

  int64_t delay_ms = arrival_ms - (sender_ms + clock_offset_ms);
  LOG_INF("UDP RX: sender_ms=%" PRId64 " arrival_ms=%" PRId64
          " offset_ms=%" PRId64 " => one_way_delay_ms=%" PRId64,
          sender_ms, arrival_ms, clock_offset_ms, delay_ms);
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
  /* UDP socket — both sides bind; initiator also sends                     */
  /* ---------------------------------------------------------------------- */

  int sock = udp_open_socket(CONFIG_NET_CONFIG_MY_IPV6_ADDR);
  if (sock < 0) {
    LOG_ERR("Failed to open UDP socket, aborting");
    return -EIO;
  }

  /* ---------------------------------------------------------------------- */
  /* Glossy configuration                                                    */
  /* ---------------------------------------------------------------------- */

  struct deca_glossy_configuration conf = {
      .node_addr = GLOSSY_NODE_ID,
      .isRoot = is_initiator,
      .guard_period_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US,
      .max_depth = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_MAX_DEPTH,
      .transmission_delay_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US,
      .payload = NULL,
      .payload_size = 0,
  };

  struct deca_glossy_result result;

  /* ---------------------------------------------------------------------- */
  /* Synchronization state                                                   */
  /* ---------------------------------------------------------------------- */

  /*
   * clock_offset_ms = follower_local_ms - initiator_local_ms
   * Derived from result.rtc_clock_pair each successful round.
   */
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
   * UDP fires halfway through the Glossy interval, i.e.
   * GLOSSY_INTERVAL_MS/2 after each completed Glossy round.
   * INT64_MAX means "not yet armed".
   */
  int64_t next_glossy_ms = k_uptime_get();
  int64_t next_udp_ms = INT64_MAX;

  /* ---------------------------------------------------------------------- */
  /* Event loop                                                              */
  /* ---------------------------------------------------------------------- */

  while (1) {
    int64_t now_ms = k_uptime_get();
    int64_t to_glossy_ms = next_glossy_ms - now_ms;
    int64_t to_udp_ms = next_udp_ms - now_ms;

    /* Sleep until the nearest due event */
    if (to_glossy_ms > 0 && to_udp_ms > 0) {
      k_sleep(K_MSEC(MIN(to_glossy_ms, to_udp_ms)));
    }

    now_ms = k_uptime_get();
    to_glossy_ms = next_glossy_ms - now_ms;
    to_udp_ms = next_udp_ms - now_ms;

    /* ------------------------------------------------------------------ */
    /* Glossy round                                                        */
    /* ------------------------------------------------------------------ */

    if (to_glossy_ms <= 0) {
      LOG_INF("[glossy] round start: role=%s id=%d",
              is_initiator ? "initiator" : "follower", GLOSSY_NODE_ID);

      /*
       * deca_glossy_time_synchronization() calls
       * uwb_broker_acquire_lease() internally, parking the 802.15.4
       * driver for the duration of the flood.  UDP comms resume
       * automatically once the lease is released on return.
       */
      int ret = deca_glossy_time_synchronization(dev, &conf, &result);

      if (ret == 0) {
        int64_t rtc_offset_ticks = (int64_t)result.rtc_clock_pair.local - (int64_t)result.rtc_clock_pair.ref;
        int64_t deca_offset_ticks = (int64_t)result.deca_clock_pair.local - (int64_t)result.deca_clock_pair.ref;

        int64_t new_offset_ms = (rtc_offset_ticks * 1000LL) / 32768LL;
        int64_t deca_offset_ms = (int64_t)DWT_TS_TO_US(
                                     (uint64_t)(deca_offset_ticks < 0
                                                    ? -deca_offset_ticks
                                                    : deca_offset_ticks)) /
                                 1000LL;
        if (deca_offset_ticks < 0) {
          deca_offset_ms = -deca_offset_ms;
        }

        clock_offset_ms = new_offset_ms;

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
      }

      if (ret != 0) {
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
          next_glossy_ms = ((now_ms / GLOSSY_INTERVAL_MS) + 1) * GLOSSY_INTERVAL_MS;
        }
      } else if (offset_known && !sync_lost) {
        /* Align to initiator's 2-second grid via known offset */
        int64_t now_initiator = now_ms - clock_offset_ms;
        int64_t next_initiator = ((now_initiator / GLOSSY_INTERVAL_MS) + 1) * GLOSSY_INTERVAL_MS;
        next_glossy_ms = next_initiator + clock_offset_ms;
      } else {
        next_glossy_ms = now_ms + interval_ms;
      }

      /*
       * Arm the UDP event halfway through the next Glossy interval.
       * Must be scheduled in initiator-aligned time on both nodes so
       * the follower's recvfrom fires AFTER the initiator sends.
       *
       * next_glossy_ms is already in initiator-aligned local time on
       * both nodes (see scheduling above), so offsetting from that
       * gives the same absolute moment on both clocks.
       *
       * Skip during recovery — the 20 ms fast interval leaves no
       * useful window for a UDP exchange.
       */
      if (!sync_lost) {
        next_udp_ms = next_glossy_ms - (GLOSSY_INTERVAL_MS / 2);
      } else {
        next_udp_ms = INT64_MAX;
      }
    }

    /* ------------------------------------------------------------------ */
    /* UDP timestamped exchange                                            */
    /* ------------------------------------------------------------------ */

    if (to_udp_ms <= 0) {
      LOG_INF("[udp] exchange start: role=%s id=%d offset_known=%d "
              "clock_offset_ms=%" PRId64,
              is_initiator ? "initiator" : "follower", GLOSSY_NODE_ID,
              offset_known, clock_offset_ms);
      if (is_initiator) {
        /*
         * Broker is in IEEE802154 owner state here — no Glossy round
         * is running — so the 802.15.4 driver handles this normally.
         */
        // udp_send_timestamp(sock, CONFIG_NET_CONFIG_PEER_IPV6_ADDR);
      } else {
        /*
         * Drain whatever arrived since the last check.  Typically one
         * packet sent by the initiator half an interval ago.
         */
        /* Block up to 500 ms — well within the 1-second window
         * between the UDP arm point and the next Glossy round. */
        // udp_recv_one(sock, offset_known, clock_offset_ms, 500);
      }
      next_udp_ms = INT64_MAX; /* disarmed; re-armed after next Glossy round */
    }
  }

  close(sock);
  return 0;
}