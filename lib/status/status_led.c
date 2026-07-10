/*
 * Copyright (c) 2025 SynchroFly Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <app/lib/status/status.h>
#include <app/lib/system/node.h>
#include <app/lib/timesync/time_synchronization.h>

LOG_MODULE_REGISTER(status_led, CONFIG_SYNCHROFLY_STATUS_LOG_LEVEL);

/* LED device tree binding */
#define STATUS_LED_NODE DT_ALIAS(synchrofly_status_led)

#if !DT_NODE_HAS_STATUS(STATUS_LED_NODE, okay)
#error "synchrofly-status-led alias not defined in device tree"
#endif

static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(STATUS_LED_NODE, gpios);

/* Identification LED - separate from status LED to avoid interference */
#define IDENTIFY_LED_NODE DT_ALIAS(identify_led)

#if DT_NODE_HAS_STATUS(IDENTIFY_LED_NODE, okay)
#define HAVE_IDENTIFY_LED 1
static const struct gpio_dt_spec identify_led = GPIO_DT_SPEC_GET(IDENTIFY_LED_NODE, gpios);
#else
#define HAVE_IDENTIFY_LED 0
#endif

/* Current status state */
static enum synchrofly_status current_status = STATUS_DISABLED;

/* Identification mode state */
static bool identify_active = false;
static int64_t identify_end_time = 0;

/* LED pattern definitions (in 250ms units) */
#define PATTERN_CYCLE_MS 250

struct led_pattern {
    const uint8_t *sequence;  /* Array of ON/OFF durations in PATTERN_CYCLE_MS units */
    uint8_t length;           /* Number of elements in sequence */
};

/* Pattern: OFF - solid off */
static const uint8_t pattern_off[] = {0};

/* Pattern: Slow blink (1Hz) - 500ms on, 500ms off */
static const uint8_t pattern_slow_blink[] = {2, 2};  /* 2*250ms on, 2*250ms off */

/* Pattern: Fast blink (4Hz) - 125ms on, 125ms off */
static const uint8_t pattern_fast_blink[] = {1, 1};  /* 1*250ms on (partial), 1*250ms off */

/* Pattern: Double blink - two quick blinks, pause */
static const uint8_t pattern_double_blink[] = {
    1,  /* 250ms on */
    1,  /* 250ms off */
    1,  /* 250ms on */
    3   /* 750ms off */
};

/* Pattern: Solid ON */
static const uint8_t pattern_solid_on[] = {1};

/* Pattern: Rapid flicker (8Hz) - very fast blink */
static const uint8_t pattern_rapid_flicker[] = {1, 1};  /* Same timing, but feels more erratic */

static const struct led_pattern patterns[] = {
    [STATUS_DISABLED] = {pattern_off, 1},
    [STATUS_WARMING_UP] = {pattern_slow_blink, 2},
    [STATUS_SYNCED_PASSIVE] = {pattern_fast_blink, 2},
    [STATUS_SYNCED_ACTIVE] = {pattern_double_blink, 4},
    [STATUS_ROOT_MODE] = {pattern_solid_on, 1},
    [STATUS_DEGRADED] = {pattern_rapid_flicker, 2},
};

/**
 * Determine the current system status by querying subsystems.
 */
static enum synchrofly_status determine_status(void)
{
    node_mode_t mode = get_node_mode();
    int32_t asn = slotted_schedule_get_asn();
    bool is_root = get_node_is_root();

    /* Root mode takes priority over all other states (except disabled) */
    if (is_root && mode != NODE_MODE_DISABLED) {
        return STATUS_ROOT_MODE;
    }

    /* Check if node is disabled */
    if (mode == NODE_MODE_DISABLED) {
        return STATUS_DISABLED;
    }

    /* Check if we're synchronized */
    if (asn < 0) {
        return STATUS_WARMING_UP;
    }

    /* We're synchronized - check node mode */
    switch (mode) {
    case NODE_MODE_PASSIVE:
        return STATUS_SYNCED_PASSIVE;

    case NODE_MODE_ACTIVE:
        return STATUS_SYNCED_ACTIVE;

    case NODE_MODE_RANGING_DISABLED:
        return STATUS_DEGRADED;

    default:
        return STATUS_DEGRADED;
    }
}

/**
 * LED control thread stack and structure.
 */
#define STATUS_LED_THREAD_STACK_SIZE 512
#define STATUS_LED_THREAD_PRIORITY K_PRIO_COOP(10)

static void status_led_thread(void *arg1, void *arg2, void *arg3);

K_THREAD_DEFINE(status_led_tid, STATUS_LED_THREAD_STACK_SIZE,
                status_led_thread, NULL, NULL, NULL,
                STATUS_LED_THREAD_PRIORITY, 0, 0);

/**
 * LED control thread main loop.
 */
static void status_led_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    uint8_t pattern_index = 0;
    bool led_state = false;
    uint8_t cycle_count = 0;
    enum synchrofly_status prev_status = STATUS_DISABLED;

    LOG_INF("SynchroFly status LED thread started");

    while (1) {
        /* Check if identification mode is active */
        if (identify_active) {
            if (k_uptime_get() >= identify_end_time) {
                identify_active = false;
#if HAVE_IDENTIFY_LED
                /* Turn off identification LED */
                gpio_pin_set_dt(&identify_led, 0);
#endif
                LOG_INF("Identification ended");
            } else {
#if HAVE_IDENTIFY_LED
                /* Fast blink identification LED: 100ms on/off */
                led_state = ((k_uptime_get() / 100) % 2) == 0;
                gpio_pin_set_dt(&identify_led, led_state ? 1 : 0);
                /* Continue with normal status LED operation */
#else
                /* Fallback: use status LED for identification */
                led_state = ((k_uptime_get() / 100) % 2) == 0;
                gpio_pin_set_dt(&status_led, led_state ? 0 : 1);
                k_msleep(50);
                continue;
#endif
            }
        }

        /* Determine current status */
        current_status = determine_status();

        /* Reset pattern state on status change */
        if (current_status != prev_status) {
            pattern_index = 0;
            cycle_count = 0;
            prev_status = current_status;
            LOG_INF("Status changed to %d (is_root=%d, mode=%d, asn=%d)",
                    current_status, get_node_is_root(), get_node_mode(),
                    slotted_schedule_get_asn());
        }

        /* Get current pattern */
        const struct led_pattern *pattern = &patterns[current_status];

        /* Update pattern index (wrap around) */
        if (pattern_index >= pattern->length) {
            pattern_index = 0;
        }

        /* Get current cycle duration from pattern */
        uint8_t duration = pattern->sequence[pattern_index];

        /* Determine LED state based on pattern position */
        /* Even indices = ON, odd indices = OFF for most patterns */
        if (current_status == STATUS_ROOT_MODE) {
            /* Solid ON: always on */
            led_state = true;
        } else if (current_status == STATUS_DISABLED) {
            /* OFF: always off */
            led_state = false;
        } else {
            /* Toggle based on pattern index (even = ON, odd = OFF) */
            led_state = (pattern_index % 2 == 0);
        }

        /* Set LED state (active-low: 0 = ON, 1 = OFF) */
        gpio_pin_set_dt(&status_led, led_state ? 0 : 1);

        /* Increment cycle counter */
        cycle_count++;

        /* Check if we've completed this pattern element */
        if (cycle_count >= duration) {
            cycle_count = 0;
            pattern_index++;
        }

        /* Sleep for one pattern cycle */
        k_msleep(PATTERN_CYCLE_MS);
    }
}

/**
 * Public API implementation
 */

enum synchrofly_status synchrofly_status_get(void)
{
    return current_status;
}

void synchrofly_status_set(enum synchrofly_status status)
{
    /* Manual override not yet implemented - status is auto-determined */
    LOG_WRN("Manual status override not implemented (requested: %d)", status);
}

void synchrofly_status_identify(uint32_t duration_ms)
{
    identify_active = true;
    identify_end_time = k_uptime_get() + duration_ms;
    LOG_INF("Identification triggered for %u ms", duration_ms);
}

/**
 * Initialize the status LED subsystem.
 */
static int status_led_init(void)
{
    int ret;

    /* Check if LED device is ready */
    if (!device_is_ready(status_led.port)) {
        LOG_ERR("Status LED GPIO device not ready");
        return -ENODEV;
    }

    /* Configure LED GPIO as output */
    ret = gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure status LED GPIO (err %d)", ret);
        return ret;
    }

    LOG_INF("SynchroFly status LED initialized on %s pin %d",
            status_led.port->name, status_led.pin);

#if HAVE_IDENTIFY_LED
    /* Configure identification LED */
    if (!device_is_ready(identify_led.port)) {
        LOG_WRN("Identification LED GPIO device not ready");
    } else {
        ret = gpio_pin_configure_dt(&identify_led, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure identification LED GPIO (err %d)", ret);
        } else {
            LOG_INF("Identification LED initialized on %s pin %d",
                    identify_led.port->name, identify_led.pin);
        }
    }
#endif

    return 0;
}

/* Initialize after node and timesync subsystems */
SYS_INIT(status_led_init, APPLICATION, 99);
