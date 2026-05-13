#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

#include <app/drivers/ieee802154/uwb_driver_api.h>

LOG_MODULE_REGISTER(glossy_sync, LOG_LEVEL_INF);

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

int main(void)
{
  for(int i = 0; i < 5; i++) {
    printk("System alive...\n");
    k_sleep(K_MSEC(2000));
  }

    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));

    if (!device_is_ready(dev)) {
        LOG_ERR("UWB device not ready");
        return -ENODEV;
    }

    LOG_INF("Glossy POC started; node id=%d role=%s", GLOSSY_NODE_ID,
            IS_ENABLED(CONFIG_GLOSSY_IS_INITIATOR) ? "initiator" : "follower");

    struct deca_glossy_configuration conf = {
        .node_addr = GLOSSY_NODE_ID,
        .isRoot = IS_ENABLED(CONFIG_GLOSSY_IS_INITIATOR),
        .guard_period_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_GUARD_US,
        .max_depth = 6,
        .payload = NULL,
        .payload_size = 0,
        .transmission_delay_us = CONFIG_SYNCHROFLY_NETWORK_DEFAULT_GLOSSY_TRANSMISSION_DELAY_US,
    };

    struct deca_glossy_result result;

    // Synchronization state
    bool is_initiator = IS_ENABLED(CONFIG_GLOSSY_IS_INITIATOR);
    int64_t rtc_offset_ms = 0;  // Offset between this node's RTC and root's RTC (local - root)
    bool offset_known = false;  // Whether follower has learned its offset
    int glossy_failures_in_row = 0;  // Count consecutive glossy sync failures
    bool sync_lost = false;  // Whether we've lost sync and are in recovery mode

    // Scheduling state
    int64_t next_glossy_ms = 0;      // Next time to run glossy round (in local time)
    int64_t next_test_msg_ms = 5000; // Next time to output test message (in local time)

    // Intervals
    int64_t glossy_interval_ms = 2000;  // between glossy rounds
    int64_t glossy_fast_interval_ms = 20;  // during recovery/resync
    int64_t test_msg_interval_ms = 5000; // between test messages

    // Initialize scheduling: glossy starts immediately
    next_glossy_ms = k_uptime_get();

    while (1) {
        int64_t current_time_ms = k_uptime_get();
        int64_t time_to_glossy_ms = next_glossy_ms - current_time_ms;
        int64_t time_to_test_msg_ms = next_test_msg_ms - current_time_ms;

        // Determine which event happens first and sleep until then
        int64_t sleep_time_ms;
        if (time_to_glossy_ms <= 0 || time_to_test_msg_ms <= 0) {
            // At least one event is due now
            sleep_time_ms = 0;
        } else {
            // Sleep until the next event
            sleep_time_ms = (time_to_glossy_ms < time_to_test_msg_ms) ? time_to_glossy_ms : time_to_test_msg_ms;
        }

        if (sleep_time_ms > 0) {
            k_sleep(K_MSEC(sleep_time_ms));
        }

        // Re-check timing after sleep
        current_time_ms = k_uptime_get();
        time_to_glossy_ms = next_glossy_ms - current_time_ms;
        time_to_test_msg_ms = next_test_msg_ms - current_time_ms;

        // Handle glossy round if due
        if (time_to_glossy_ms <= 0) {
            printk("[glossy] round start: role=%s id=%d\n",
                   is_initiator ? "init" : "follower",
                   GLOSSY_NODE_ID);

            int ret = deca_glossy_time_synchronization(dev, &conf, &result);
            if (ret == 0) {
                int64_t rtc_offset_ticks = (int64_t)result.rtc_clock_pair.local - (int64_t)result.rtc_clock_pair.ref;
                int64_t deca_offset_ticks = (int64_t)result.deca_clock_pair.local - (int64_t)result.deca_clock_pair.ref;

                int64_t new_rtc_offset_ms = (rtc_offset_ticks * 1000LL) / 32768LL;
                int64_t deca_offset_ms = (int64_t)DWT_TS_TO_US((uint64_t)(deca_offset_ticks < 0 ? -deca_offset_ticks : deca_offset_ticks)) / 1000LL;
                if (deca_offset_ticks < 0) {
                    deca_offset_ms = -deca_offset_ms;
                }

                LOG_INF("clock_offset: rtc=%" PRId64 " ms, dwt=%" PRId64 " ms",
                        new_rtc_offset_ms, deca_offset_ms);

                // Follower: update offset when first obtained
                if (!is_initiator && !offset_known) {
                    rtc_offset_ms = new_rtc_offset_ms;
                    offset_known = true;
                    LOG_INF("Follower: offset synchronized, scheduling glossy to align with root");
                } else if (!is_initiator) {
                    // Follower: update offset with latest value
                    rtc_offset_ms = new_rtc_offset_ms;
                }

                // Glossy succeeded: reset failure counter and sync recovery state
                if (glossy_failures_in_row > 0) {
                    LOG_INF("Glossy sync recovered after %d failures", glossy_failures_in_row);
                }
                if (sync_lost) {
                    LOG_INF("Sync re-established! Returning to normal glossy interval");
                    sync_lost = false;
                }
                glossy_failures_in_row = 0;
            } else {
                if(glossy_failures_in_row == 0) {
                  LOG_WRN("Glossy sync failed: %d", ret);
                }
                glossy_failures_in_row++;

                // After 3 consecutive failures, mark sync as lost and speed up retries
                if (glossy_failures_in_row >= 3 && !sync_lost) {
                    LOG_ERR("Glossy sync lost after 3 failures - root may have restarted. Entering recovery mode.");
                    sync_lost = true;
                } else if (glossy_failures_in_row >= 3 && glossy_failures_in_row % 20 == 0) {
                    LOG_WRN("Still in sync recovery mode (failures: %d)", glossy_failures_in_row);
                }
            }

            printk("[glossy] round done: ret=%d\n", ret);

            // Schedule next glossy round
            current_time_ms = k_uptime_get();

            // Use faster interval during recovery/resync, normal interval otherwise
            int64_t active_interval_ms = sync_lost ? glossy_fast_interval_ms : glossy_interval_ms;

            if (is_initiator) {
                // Initiator: glossy at normal interval (still reachable even in recovery)
                if (sync_lost) {
                    // During recovery, keep trying more frequently too
                    next_glossy_ms = current_time_ms + active_interval_ms;
                } else {
                    next_glossy_ms = ((current_time_ms / glossy_interval_ms) + 1) * glossy_interval_ms;
                }
            } else if (offset_known) {
                // Follower: glossy aligned to root's schedule
                int64_t current_abs_time_ms = current_time_ms - rtc_offset_ms;

                if (sync_lost) {
                    // During recovery, use fast interval to quickly re-establish
                    next_glossy_ms = current_time_ms + active_interval_ms;
                } else {
                    // Normal operation: align to root's 2-second schedule
                    int64_t next_abs_glossy_ms = ((current_abs_time_ms / glossy_interval_ms) + 1) * glossy_interval_ms;
                    next_glossy_ms = next_abs_glossy_ms + rtc_offset_ms;
                }
            } else {
                // Follower hasn't got offset yet, use fast interval to find root
                next_glossy_ms = current_time_ms + active_interval_ms;
            }
        }

        // Handle test message if due
        if (time_to_test_msg_ms <= 0) {
            current_time_ms = k_uptime_get();

            LOG_INF("TEST MESSAGE - synchronized output (node=%d, role=%s)",
                    GLOSSY_NODE_ID, is_initiator ? "initiator" : "follower");

            if (is_initiator) {
                // Initiator: test messages every 5 seconds in local time
                next_test_msg_ms = ((current_time_ms / test_msg_interval_ms) + 1) * test_msg_interval_ms;
            } else if (offset_known) {
                // Follower: test messages aligned to root's 5-second schedule
                int64_t current_abs_time_ms = current_time_ms - rtc_offset_ms;
                int64_t next_abs_test_msg_ms = ((current_abs_time_ms / test_msg_interval_ms) + 1) * test_msg_interval_ms;
                next_test_msg_ms = next_abs_test_msg_ms + rtc_offset_ms;
            } else {
                // Follower hasn't got offset yet, just add 5 seconds
                next_test_msg_ms = current_time_ms + test_msg_interval_ms;
            }
        }
    }

    return 0;
}