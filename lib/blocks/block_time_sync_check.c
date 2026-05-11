#include <zephyr/logging/log.h>

#include <app/lib/system/node.h>
#include <app/lib/system/hw.h>
#include <app/drivers/ieee802154/uwb_driver_api.h>
#include <app/lib/node_table/node_table.h>
#include <app/lib/timesync/time_synchronization.h>

#include <app/lib/scheduling/lower/schedule_functions.h>
#include <app/lib/scheduling/lower/schedule.h>
#include <app/lib/management/network_settings.h>

#include <app/lib/blocks/blocks.h>
#include <app/drivers/debug/timesync_debug_gpio.h>

LOG_MODULE_REGISTER(time_sync_check);

void time_sync_check_block_handler(uint64_t rtc_event_time, void *user_data)
{
    (void)user_data;

    timesync_debug_pulse();

    uint64_t local_now = k_uptime_ticks();
    uint64_t ref_now = 0;

    int err = get_current_reference_time(NULL, &ref_now);
    if (err < 0) {
        printk("{\"event\": \"sync_check\", \"node_id\": \"0x%04hx\", "
               "\"error\": \"no_ref\", \"rtc\": %lld}\n",
               get_node_addr(), rtc_event_time);
        return;
    }

    /* Compute offset: ref = local + offset */
    int64_t offset_ticks = (int64_t)ref_now - (int64_t)local_now;
    int64_t offset_us = (offset_ticks * 1000000) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

    printk("{\"event\": \"sync_check\", \"node_id\": \"0x%04hx\", "
           "\"local\": %llu, \"ref\": %llu, \"offset_us\": %lld, "
           "\"event_rtc\": %lld}\n",
           get_node_addr(),
           TICKS_TO_USEC(local_now), TICKS_TO_USEC(ref_now),
           offset_us, rtc_event_time);
}
