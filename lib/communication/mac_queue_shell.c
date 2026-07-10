/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/lib/communication/mac_queue.h>
#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <string.h>

static int cmd_mac_queue_stats(const struct shell *sh, size_t argc, char **argv)
{
    struct mac_queue_stats stats;
    mac_queue_get_stats(&stats);

    shell_print(sh, "MAC Queue Statistics:");
    shell_print(sh, "TX Queue:");
    shell_print(sh, "  Enqueued:      %u", stats.tx_enqueued);
    shell_print(sh, "  Dequeued:      %u", stats.tx_dequeued);
    shell_print(sh, "  Dropped:       %u", stats.tx_dropped);
    shell_print(sh, "  Current depth: %u", stats.tx_current_depth);

    shell_print(sh, "RX Queue:");
    shell_print(sh, "  Enqueued:      %u", stats.rx_enqueued);
    shell_print(sh, "  Dequeued:      %u", stats.rx_dequeued);
    shell_print(sh, "  Dropped:       %u", stats.rx_dropped);
    shell_print(sh, "  Current depth: %u", stats.rx_current_depth);

    shell_print(sh, "Errors:");
    shell_print(sh, "  Invalid frames: %u", stats.invalid_frames);
    shell_print(sh, "  Oversized:      %u", stats.oversized_frames);

    return 0;
}

static int cmd_mac_queue_reset(const struct shell *sh, size_t argc, char **argv)
{
    mac_queue_reset_stats();
    shell_print(sh, "MAC queue statistics reset");
    return 0;
}

static int cmd_mac_queue_clear_tx(const struct shell *sh, size_t argc, char **argv)
{
    size_t cleared = mac_queue_tx_clear();
    shell_print(sh, "TX queue cleared: %zu frames removed", cleared);
    return 0;
}

static int cmd_mac_queue_clear_rx(const struct shell *sh, size_t argc, char **argv)
{
    size_t cleared = mac_queue_rx_clear();
    shell_print(sh, "RX queue cleared: %zu frames removed", cleared);
    return 0;
}

static int cmd_mac_queue_send_test(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: mac_queue send_test <message>");
        return -EINVAL;
    }

    /* Create a test broadcast frame */
    struct mac_queue_frame frame;
    const char *message = argv[1];

    int ret = mac_queue_prepare_broadcast(&frame, (const uint8_t *)message, strlen(message));
    if (ret < 0) {
        shell_error(sh, "Failed to prepare frame: %d", ret);
        return ret;
    }

    ret = mac_queue_tx_push(&frame, K_NO_WAIT);
    if (ret < 0) {
        shell_error(sh, "Failed to enqueue frame: %d", ret);
        return ret;
    }

    shell_print(sh, "Test frame enqueued: \"%s\" (%zu bytes)", message, frame.length);
    return 0;
}

static int cmd_mac_queue_recv_test(const struct shell *sh, size_t argc, char **argv)
{
    struct mac_queue_frame frame;

    int ret = mac_queue_rx_pop(&frame, K_NO_WAIT);
    if (ret < 0) {
        if (ret == -EAGAIN) {
            shell_print(sh, "No frames in RX queue");
        } else {
            shell_error(sh, "Failed to dequeue frame: %d", ret);
        }
        return ret;
    }

    /* Extract and display payload */
    uint8_t payload[128];
    size_t payload_len = sizeof(payload);

    ret = mac_queue_extract_payload(&frame, payload, &payload_len);
    if (ret == 0 && payload_len > 0) {
        /* Null-terminate for safe printing */
        if (payload_len < sizeof(payload)) {
            payload[payload_len] = '\0';
        } else {
            payload[sizeof(payload) - 1] = '\0';
        }

        shell_print(sh, "Received frame:");
        shell_print(sh, "  Length: %zu bytes", frame.length);
        shell_print(sh, "  Dest: 0x%04x", frame.dest_addr);
        shell_print(sh, "  Flags: 0x%02x", frame.flags);
        shell_print(sh, "  Payload: \"%s\" (%zu bytes)", payload, payload_len);
    } else {
        shell_print(sh, "Received frame without payload (%zu bytes)", frame.length);
    }

    return 0;
}

static int cmd_mac_queue_dump_tx(const struct shell *sh, size_t argc, char **argv)
{
    size_t depth = mac_queue_tx_depth();
    shell_print(sh, "TX queue depth: %zu", depth);

    if (depth == 0) {
        shell_print(sh, "TX queue is empty");
        return 0;
    }

    shell_print(sh, "Note: Use 'mac_queue send_test' to add frames to TX queue");
    shell_print(sh, "      Use 'mac_queue clear_tx' to clear TX queue");

    return 0;
}

static int cmd_mac_queue_dump_rx(const struct shell *sh, size_t argc, char **argv)
{
    size_t depth = mac_queue_rx_depth();
    shell_print(sh, "RX queue depth: %zu", depth);

    if (depth == 0) {
        shell_print(sh, "RX queue is empty");
        return 0;
    }

    shell_print(sh, "Note: Use 'mac_queue recv_test' to pop frames from RX queue");
    shell_print(sh, "      Use 'mac_queue clear_rx' to clear RX queue");

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mac_queue,
    SHELL_CMD(stats, NULL, "Show MAC queue statistics", cmd_mac_queue_stats),
    SHELL_CMD(reset, NULL, "Reset MAC queue statistics", cmd_mac_queue_reset),
    SHELL_CMD(clear_tx, NULL, "Clear TX queue", cmd_mac_queue_clear_tx),
    SHELL_CMD(clear_rx, NULL, "Clear RX queue", cmd_mac_queue_clear_rx),
    SHELL_CMD(send_test, NULL, "Send test frame <message>", cmd_mac_queue_send_test),
    SHELL_CMD(recv_test, NULL, "Receive test frame", cmd_mac_queue_recv_test),
    SHELL_CMD(dump_tx, NULL, "Show TX queue status", cmd_mac_queue_dump_tx),
    SHELL_CMD(dump_rx, NULL, "Show RX queue status", cmd_mac_queue_dump_rx),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(mac_queue, &sub_mac_queue, "MAC queue commands", NULL);