/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Position payload implementation
 */

#include <app/lib/localization/position_payload.h>
#include <app/lib/system/node.h>
#include <app/lib/node_table/node_table.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(position_payload, LOG_LEVEL_WRN);

int position_payload_build(uint8_t *buf, size_t buf_size)
{
    if (!buf) {
        return -EINVAL;
    }

    node_position_mode_t mode = get_node_position_mode();

    if (mode == NODE_POSITION_MODE_STATIC) {
        /* Anchor: include position */
        if (buf_size < POSITION_PAYLOAD_ANCHOR_SIZE) {
            return -ENOBUFS;
        }

        struct position_anchor_payload *payload = (struct position_anchor_payload *)buf;
        payload->header.version = POSITION_PAYLOAD_VERSION;
        payload->header.flags = POSITION_PAYLOAD_FLAG_ANCHOR;

        /* Get configured position */
        struct node_config_position pos;
        int ret = get_node_config_position(&pos);
        if (ret == 0) {
            payload->pos_x = (float)pos.x;
            payload->pos_y = (float)pos.y;
            payload->pos_z = (float)pos.z;
        } else {
            /* Position not configured - use zeros */
            payload->pos_x = 0.0f;
            payload->pos_y = 0.0f;
            payload->pos_z = 0.0f;
            LOG_WRN("Anchor has no configured position");
        }

        LOG_INF("Built anchor payload pos=(%.2f, %.2f, %.2f)",
                (double)payload->pos_x, (double)payload->pos_y, (double)payload->pos_z);

        return POSITION_PAYLOAD_ANCHOR_SIZE;
    } else {
        /* Mobile node: header only */
        if (buf_size < POSITION_PAYLOAD_HEADER_SIZE) {
            return -ENOBUFS;
        }

        struct position_payload_header *header = (struct position_payload_header *)buf;
        header->version = POSITION_PAYLOAD_VERSION;
        header->flags = 0;  /* Not an anchor */

        LOG_DBG("Built mobile payload (header only)");

        return POSITION_PAYLOAD_HEADER_SIZE;
    }
}

int position_payload_process(uint16_t sender_id, const uint8_t *buf, size_t len, uint64_t rtc)
{
    if (!buf || len < POSITION_PAYLOAD_HEADER_SIZE) {
        return -EINVAL;
    }

    const struct position_payload_header *header = (const struct position_payload_header *)buf;

    /* Check version */
    if (header->version != POSITION_PAYLOAD_VERSION) {
        LOG_WRN("Unknown position payload version: %d", header->version);
        LOG_HEXDUMP_WRN(buf, len, "Bad payload:");
        return -ENOTSUP;
    }

    /* Check if sender is anchor */
    if (!(header->flags & POSITION_PAYLOAD_FLAG_ANCHOR)) {
        /* Mobile node - nothing to update in neighbor table */
        LOG_DBG("Received mobile payload from 0x%04x", sender_id);
        return 0;
    }

    /* Anchor node - extract and store position */
    if (len < POSITION_PAYLOAD_ANCHOR_SIZE) {
        LOG_WRN("Truncated anchor payload from 0x%04x: %zu bytes", sender_id, len);
        return -EINVAL;
    }

    const struct position_anchor_payload *anchor = (const struct position_anchor_payload *)buf;

    /* Skip our own position payload - node table rejects self-entries */
    if (sender_id == get_node_addr()) {
        return 0;
    }

    /* Update neighbor table with anchor position */
    int ret = node_table_update_position(sender_id,
                                         anchor->pos_x,
                                         anchor->pos_y,
                                         anchor->pos_z,
                                         rtc);
    if (ret < 0) {
        LOG_ERR("Failed to update anchor 0x%04x position: %d", sender_id, ret);
        return ret;
    }
    node_table_notify_changed();

    LOG_INF("Anchor 0x%04x pos=(%.2f, %.2f, %.2f)",
            sender_id, (double)anchor->pos_x, (double)anchor->pos_y, (double)anchor->pos_z);

    return 0;
}

bool position_payload_is_anchor(const uint8_t *buf, size_t len)
{
    if (!buf || len < POSITION_PAYLOAD_HEADER_SIZE) {
        return false;
    }

    const struct position_payload_header *header = (const struct position_payload_header *)buf;

    if (header->version != POSITION_PAYLOAD_VERSION) {
        return false;
    }

    return (header->flags & POSITION_PAYLOAD_FLAG_ANCHOR) != 0;
}
