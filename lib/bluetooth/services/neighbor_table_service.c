/**
 * @file neighbor_table_service.c
 * @brief GATT service for exposing neighbor table information
 *
 * Provides characteristics for:
 * - Neighbor count (uint8_t)
 * - Neighbor list (array of packed entries with notify support)
 */

#include <zephyr/kernel.h>
#include <app/lib/system/block_heap.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <limits.h>

#include <app/lib/node_table/node_table.h>
#include <app/lib/system/node.h>
#include "../bt_services.h"
#include "neighbor_table_service.h"

LOG_MODULE_REGISTER(neighbor_table_svc, CONFIG_LOG_DEFAULT_LEVEL);

/* Work queue for deferred notifications (to avoid mutex deadlock) */
static void neighbor_notify_work_handler(struct k_work *work);
static K_WORK_DEFINE(neighbor_notify_work, neighbor_notify_work_handler);

/* Custom UUID for neighbor table service */
/* Base UUID: 6e656967-6862-7461-626c-000000000000 (ASCII "neighbtabl") */
#define BT_UUID_NEIGHBOR_TABLE_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000000ULL)

#define BT_UUID_NEIGHBOR_TABLE_SERVICE \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_TABLE_SERVICE_VAL)

/* Neighbor count characteristic UUID: ...0001 */
#define BT_UUID_NEIGHBOR_COUNT_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000001ULL)

#define BT_UUID_NEIGHBOR_COUNT \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_COUNT_VAL)

/* Neighbor list characteristic UUID: ...0002 */
#define BT_UUID_NEIGHBOR_LIST_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000002ULL)

#define BT_UUID_NEIGHBOR_LIST \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_LIST_VAL)

/* Query address characteristic UUID: ...0003 */
#define BT_UUID_NEIGHBOR_QUERY_ADDR_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000003ULL)

#define BT_UUID_NEIGHBOR_QUERY_ADDR \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_QUERY_ADDR_VAL)

/* Query result characteristic UUID: ...0004 */
#define BT_UUID_NEIGHBOR_QUERY_RESULT_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000004ULL)

#define BT_UUID_NEIGHBOR_QUERY_RESULT \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_QUERY_RESULT_VAL)

/* Position stream characteristic UUID: ...0005 */
#define BT_UUID_POSITION_STREAM_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000005ULL)

#define BT_UUID_POSITION_STREAM \
    BT_UUID_DECLARE_128(BT_UUID_POSITION_STREAM_VAL)

/* TWR query result characteristic UUID: ...0006 */
#define BT_UUID_NEIGHBOR_TWR_QUERY_RESULT_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000006ULL)

#define BT_UUID_NEIGHBOR_TWR_QUERY_RESULT \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_TWR_QUERY_RESULT_VAL)

/* MM query result characteristic UUID: ...0007 */
#define BT_UUID_NEIGHBOR_MM_QUERY_RESULT_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000007ULL)

#define BT_UUID_NEIGHBOR_MM_QUERY_RESULT \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_MM_QUERY_RESULT_VAL)

/* MULoc round result characteristic UUID: ...0008 */
#define BT_UUID_NEIGHBOR_MULOC_RESULT_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000008ULL)

#define BT_UUID_NEIGHBOR_MULOC_RESULT \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_MULOC_RESULT_VAL)

/* CIR data characteristic UUID: ...0009 */
#define BT_UUID_NEIGHBOR_CIR_DATA_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x000000000009ULL)

#define BT_UUID_NEIGHBOR_CIR_DATA \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_CIR_DATA_VAL)

/* Eviction timeout characteristic UUID: ...000b */
#define BT_UUID_NEIGHBOR_EVICTION_TIMEOUT_VAL \
    BT_UUID_128_ENCODE(0x6e656967, 0x6862, 0x7461, 0x626c, 0x00000000000bULL)

#define BT_UUID_NEIGHBOR_EVICTION_TIMEOUT \
    BT_UUID_DECLARE_128(BT_UUID_NEIGHBOR_EVICTION_TIMEOUT_VAL)

/* Query status codes */
#define QUERY_STATUS_OK          0x00
#define QUERY_STATUS_NOT_FOUND   0x01
#define QUERY_STATUS_NO_QUERY    0x02
#define QUERY_STATUS_NO_TWR_DATA 0x03

/* Maximum neighbors to send - matches node table capacity */
#define MAX_NEIGHBORS_BLE CONFIG_NODE_TABLE_MAX_ENTRIES

/* BLE-friendly neighbor entry structure (12 bytes packed) */
struct neighbor_entry_ble {
    uint16_t node_id;
    int32_t distance_mm;
    uint16_t age_ms;
    uint8_t hop_count;
    uint8_t flags;
    int8_t rssi;
    uint8_t _pad;
} __packed;

/* Buffer for neighbor list data */
static struct neighbor_entry_ble neighbor_buffer[MAX_NEIGHBORS_BLE];
static uint8_t neighbor_count_cache = 0;

/* Track notification subscription */
static bool notifications_enabled = false;

/* Forward declaration */
static const struct bt_gatt_attr *neighbor_list_attr;

/* Query result structure (13 bytes: status + entry) */
struct query_result_ble {
    uint8_t status;
    struct neighbor_entry_ble entry;
} __packed;

/* Query state */
static uint16_t query_node_id = 0xFFFF;
static bool query_active = false;
static struct query_result_ble query_result;
static bool query_result_notifications_enabled = false;
static const struct bt_gatt_attr *query_result_attr;
static int32_t query_last_notified_distance = INT32_MIN; /* Track last distance to avoid duplicate notifications */

/* Position stream state */
struct position_ble {
    float pos_x;        /* X coordinate in meters */
    float pos_y;        /* Y coordinate in meters */
    float pos_z;        /* Z coordinate in meters */
    uint32_t timestamp; /* RTC timestamp (ms) */
} __packed;

static bool position_notifications_enabled = false;
static const struct bt_gatt_attr *position_stream_attr;
static struct position_ble current_position;

/* TWR query result structure (53 bytes packed) */
struct twr_query_result_ble {
    uint8_t  status;          /* QUERY_STATUS_* */
    uint16_t initiator_id;
    uint16_t responder_id;
    int64_t  tx_init;
    int64_t  rx_init;
    int64_t  tx_resp;
    int64_t  rx_resp;
    int64_t  tx_final;
    int64_t  rx_final;
} __packed;

/* TWR query result state */
static bool twr_query_result_notifications_enabled = false;
static const struct bt_gatt_attr *twr_query_result_attr;
static struct twr_query_result_ble twr_query_result;
static int64_t twr_query_last_tx_init = 0; /* Dedup: only notify on new ranging round */

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
/* MM query result structure (18 bytes packed) */
struct mm_query_result_ble {
    uint8_t  status;              /*  1B: QUERY_STATUS_* */
    uint16_t node_id;             /*  2B */
    int32_t  twr_distance_um;     /*  4B: TWR distance in micrometers */
    int16_t  phase_mrad;          /*  2B: fine phase in milliradians */
    int16_t  coarse_phase_mrad;   /*  2B: coarse phase in milliradians */
    uint8_t  channel;             /*  1B: UWB channel number */
    int8_t   rssi;                /*  1B */
    uint16_t age_ms;              /*  2B */
    int16_t  cfo_ppb;             /*  2B: clock frequency offset */
    uint8_t  _pad;                /*  1B: padding to even size */
} __packed;                       /* 18 bytes */

/* MM query result state */
static bool mm_query_result_notifications_enabled = false;
static const struct bt_gatt_attr *mm_query_result_attr;
static struct mm_query_result_ble mm_query_result;
static int32_t mm_query_last_notified_twr = INT32_MIN;
#endif /* CONFIG_NODE_TABLE_MM_ENABLED */

#if defined(CONFIG_SYNCHROFLY_BLOCK_MULOC)
/* MULoc round notification scratch buffer.
 * Max size: 4B header + 8*17B (8 anchors, tag mode) + 1B seq = 141 bytes */
static uint8_t muloc_notify_buf[145];
static size_t muloc_notify_len = 0;
static bool muloc_result_notifications_enabled = false;
static const struct bt_gatt_attr *muloc_result_attr;

static ssize_t read_muloc_result(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             muloc_notify_buf, muloc_notify_len);
}

static void muloc_result_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    muloc_result_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("MULoc result notifications %s",
            muloc_result_notifications_enabled ? "enabled" : "disabled");
}
#endif /* CONFIG_SYNCHROFLY_BLOCK_MULOC */

#if defined(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
#include <app/lib/blocks/cir_read.h>

/*
 * CIR chunked notification protocol:
 *   Chunk 0 (header): [chunk_idx:1][total_chunks:1][total_samples:2][fp_index:2]
 *                      [from_index:2][channel:1] = 7 bytes
 *                      + up to (MTU - 5 - 7) bytes of CIR data
 *   Chunk N (data):    [chunk_idx:1][total_chunks:1] + CIR data bytes
 */
#define CIR_CHUNK_HEADER_SIZE 7
#define CIR_CHUNK_IDX_SIZE    2   /* chunk_idx + total_chunks */
/* Conservative payload per notification (ATT_MTU=23 default minus 3 for ATT header) */
#define CIR_NOTIFY_MTU        240  /* Typical negotiated MTU payload */

static bool cir_data_notifications_enabled = false;
static const struct bt_gatt_attr *cir_data_attr;

/* CIR transfer work queue state */
static struct k_work cir_transfer_work;
static uint8_t cir_transfer_buf[CIR_NOTIFY_MTU];

/* CIR data to transfer (heap-allocated per capture, freed after last chunk) */
static uint8_t  *cir_pending_data;
static uint16_t cir_pending_byte_len;
static uint16_t cir_pending_sample_count;
static uint16_t cir_pending_fp_index;
static uint16_t cir_pending_from_index;
static uint8_t  cir_pending_channel;
static uint8_t  cir_pending_total_chunks;
static uint8_t  cir_pending_next_chunk;
static bool     cir_transfer_active;

static void cir_transfer_work_handler(struct k_work *work);

static ssize_t read_cir_data(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    /* Return empty on read -- data comes via notifications */
    uint8_t empty = 0;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &empty, 1);
}

static void cir_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    cir_data_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("CIR data notifications %s",
            cir_data_notifications_enabled ? "enabled" : "disabled");
}

static void cir_transfer_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!cir_transfer_active || !cir_data_notifications_enabled) {
        cir_transfer_active = false;
        block_free(cir_pending_data);
        cir_pending_data = NULL;
        return;
    }

    uint8_t chunk_idx = cir_pending_next_chunk;
    size_t offset = 0;

    cir_transfer_buf[offset++] = chunk_idx;
    cir_transfer_buf[offset++] = cir_pending_total_chunks;

    uint16_t data_offset_bytes;
    uint16_t max_data_this_chunk;

    if (chunk_idx == 0) {
        /* Header chunk: metadata + first data bytes */
        memcpy(&cir_transfer_buf[offset], &cir_pending_sample_count, 2); offset += 2;
        memcpy(&cir_transfer_buf[offset], &cir_pending_fp_index, 2); offset += 2;
        memcpy(&cir_transfer_buf[offset], &cir_pending_from_index, 2); offset += 2;
        cir_transfer_buf[offset++] = cir_pending_channel;

        data_offset_bytes = 0;
        max_data_this_chunk = CIR_NOTIFY_MTU - offset;
    } else {
        /* Data-only chunks */
        uint16_t first_chunk_data = CIR_NOTIFY_MTU - CIR_CHUNK_IDX_SIZE - CIR_CHUNK_HEADER_SIZE;
        data_offset_bytes = first_chunk_data + (chunk_idx - 1) * (CIR_NOTIFY_MTU - CIR_CHUNK_IDX_SIZE);
        max_data_this_chunk = CIR_NOTIFY_MTU - offset;
    }

    uint16_t remaining = cir_pending_byte_len - MIN(data_offset_bytes, cir_pending_byte_len);
    uint16_t data_len = MIN(remaining, max_data_this_chunk);

    if (data_len > 0) {
        memcpy(&cir_transfer_buf[offset], &cir_pending_data[data_offset_bytes], data_len);
        offset += data_len;
    }

    int err = bt_gatt_notify(NULL, cir_data_attr, cir_transfer_buf, offset);
    if (err) {
        LOG_WRN("CIR notify chunk %u/%u failed: %d", chunk_idx, cir_pending_total_chunks, err);
        cir_transfer_active = false;
        block_free(cir_pending_data);
        cir_pending_data = NULL;
        return;
    }

    LOG_DBG("CIR notify chunk %u/%u (%u bytes)", chunk_idx + 1, cir_pending_total_chunks, offset);

    cir_pending_next_chunk++;
    if (cir_pending_next_chunk < cir_pending_total_chunks) {
        /* Schedule next chunk */
        k_work_submit(&cir_transfer_work);
    } else {
        cir_transfer_active = false;
        block_free(cir_pending_data);
        cir_pending_data = NULL;
        LOG_INF("CIR transfer complete: %u samples in %u chunks",
                cir_pending_sample_count, cir_pending_total_chunks);
    }
}
#endif /* CONFIG_SYNCHROFLY_BLOCK_CIR_READ */

/* Eviction timeout characteristic: uint32_t (4 bytes, ms, 0 = disabled) */
static ssize_t read_eviction_timeout(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset)
{
    uint32_t val = node_table_get_eviction_timeout_ms();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t write_eviction_timeout(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len,
                                      uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(uint32_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint32_t val;
    memcpy(&val, buf, sizeof(val));

    node_table_set_eviction_timeout_ms(val);
    LOG_INF("Eviction timeout set to %u ms via BLE", val);
    return len;
}

/**
 * @brief Convert node_entry to BLE-friendly format
 *
 * Uses the node_table API for filtered distance. Must be called outside
 * the node table mutex.
 */
static void convert_entry(const struct node_entry *src, struct neighbor_entry_ble *dst,
                          int64_t current_ticks)
{
    dst->node_id = src->node_id;

#if defined(CONFIG_NODE_TABLE_DISTANCE_SERIES_ENABLED)
    int32_t dist_mm;
    if (node_table_get_filtered_distance_mm(src->node_id, &dist_mm) == 0) {
        dst->distance_mm = dist_mm;
    } else {
        dst->distance_mm = 0;
    }
#else
    dst->distance_mm = src->last_distance_mm;
#endif

    dst->hop_count = src->hop_count;
    dst->flags = src->flags;
    dst->rssi = src->rssi;
    dst->_pad = 0;

    /* Calculate age in milliseconds using kernel tick frequency */
    int64_t age_ticks = current_ticks - (int64_t)src->last_seen_rtc;
    if (age_ticks < 0) {
        age_ticks = 0;
    }
    uint64_t age_ms = k_ticks_to_ms_floor64(age_ticks);

    /* Cap at uint16 max */
    dst->age_ms = (age_ms > UINT16_MAX) ? UINT16_MAX : (uint16_t)age_ms;
}

/**
 * @brief Refresh neighbor buffer from node table
 */
static size_t refresh_neighbor_buffer(void)
{
    /* Heap-allocate to avoid blowing the BT RX stack (only 1024 bytes) --
     * node_entry is ~92 bytes, 20 entries = ~1840 bytes which won't fit on stack. */
    struct node_entry *entries = block_malloc(sizeof(struct node_entry) * MAX_NEIGHBORS_BLE);
    if (!entries) {
        LOG_WRN("Failed to allocate neighbor entries buffer");
        neighbor_count_cache = 0;
        return 0;
    }

    /* Use get_all to include root node and all known entries, not just direct neighbors */
    size_t count = node_table_get_all(entries, MAX_NEIGHBORS_BLE);

    int64_t current_ticks = k_uptime_ticks();

    for (size_t i = 0; i < count; i++) {
        convert_entry(&entries[i], &neighbor_buffer[i], current_ticks);
    }

    block_free(entries);
    neighbor_count_cache = (uint8_t)count;
    return count;
}

/* Neighbor count characteristic: uint8_t (1 byte) */
static ssize_t read_neighbor_count(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset)
{
    uint8_t count = (uint8_t)node_table_get_count();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &count, sizeof(count));
}

/* Cache the data length from the last refresh so Read Blob requests
 * (offset > 0) see a consistent size with the initial Read Request. */
static size_t neighbor_data_len_cache = 0;

/* Neighbor list characteristic: array of neighbor_entry_ble */
static ssize_t read_neighbor_list(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
    /* Only refresh on the initial read (offset 0).
     * Read Blob requests (offset > 0) reuse the cached data to avoid
     * the data length changing between ATT transactions. */
    if (offset == 0) {
        size_t count = refresh_neighbor_buffer();
        neighbor_data_len_cache = count * sizeof(struct neighbor_entry_ble);
        LOG_DBG("Reading neighbor list: %zu entries, %zu bytes", count, neighbor_data_len_cache);
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             neighbor_buffer, neighbor_data_len_cache);
}

/* CCCD changed callback for notifications */
static void neighbor_list_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Neighbor list notifications %s",
            notifications_enabled ? "enabled" : "disabled");
}

/* Write handler for query address characteristic */
static ssize_t write_query_addr(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset,
                                uint8_t flags)
{
    if (len != sizeof(uint16_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&query_node_id, buf, sizeof(uint16_t));
    query_active = true;
    query_last_notified_distance = INT32_MIN; /* Reset so first notification always fires */

    /* Update query result immediately (use copy to avoid dangling pointer) */
    struct node_entry entry_copy;
    if (node_table_get_copy(query_node_id, &entry_copy) == 0) {
        query_result.status = QUERY_STATUS_OK;
        convert_entry(&entry_copy, &query_result.entry, k_uptime_ticks());
    } else {
        query_result.status = QUERY_STATUS_NOT_FOUND;
        memset(&query_result.entry, 0, sizeof(query_result.entry));
    }

    LOG_DBG("Query address set to 0x%04x, status=%d", query_node_id, query_result.status);
    return len;
}

/* Read handler for query result characteristic */
static ssize_t read_query_result(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    if (!query_active) {
        /* No query set yet */
        struct query_result_ble no_query = {
            .status = QUERY_STATUS_NO_QUERY,
        };
        memset(&no_query.entry, 0, sizeof(no_query.entry));
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &no_query, sizeof(no_query));
    }

    /* Refresh entry data before returning (use copy to avoid dangling pointer) */
    struct node_entry entry_copy;
    if (node_table_get_copy(query_node_id, &entry_copy) == 0) {
        query_result.status = QUERY_STATUS_OK;
        convert_entry(&entry_copy, &query_result.entry, k_uptime_ticks());
    } else {
        query_result.status = QUERY_STATUS_NOT_FOUND;
        memset(&query_result.entry, 0, sizeof(query_result.entry));
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &query_result, sizeof(query_result));
}

/* CCCD changed callback for query result notifications */
static void query_result_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    query_result_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Query result notifications %s",
            query_result_notifications_enabled ? "enabled" : "disabled");
}

/* Read handler for position stream characteristic */
static ssize_t read_position(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    /* Update timestamp on read */
    current_position.timestamp = k_uptime_get_32();
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &current_position, sizeof(current_position));
}

/* CCCD changed callback for position stream notifications */
static void position_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    position_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Position notifications %s",
            position_notifications_enabled ? "enabled" : "disabled");
}

/* Read handler for TWR query result characteristic */
static ssize_t read_twr_query_result(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset)
{
    if (!query_active) {
        struct twr_query_result_ble no_query;
        memset(&no_query, 0, sizeof(no_query));
        no_query.status = QUERY_STATUS_NO_QUERY;
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &no_query, sizeof(no_query));
    }

    /* Read the TWR data from a safe copy of the node entry */
    struct node_entry entry_copy;
    if (node_table_get_copy(query_node_id, &entry_copy) != 0) {
        struct twr_query_result_ble not_found;
        memset(&not_found, 0, sizeof(not_found));
        not_found.status = QUERY_STATUS_NOT_FOUND;
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &not_found, sizeof(not_found));
    }

    const struct node_twr_timestamps *twr = &entry_copy.last_twr;
    if (twr->tx_init == 0 && twr->rx_init == 0) {
        struct twr_query_result_ble no_data;
        memset(&no_data, 0, sizeof(no_data));
        no_data.status = QUERY_STATUS_NO_TWR_DATA;
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &no_data, sizeof(no_data));
    }

    twr_query_result.status = QUERY_STATUS_OK;
    twr_query_result.initiator_id = twr->initiator_id;
    twr_query_result.responder_id = twr->responder_id;
    twr_query_result.tx_init  = twr->tx_init;
    twr_query_result.rx_init  = twr->rx_init;
    twr_query_result.tx_resp  = twr->tx_resp;
    twr_query_result.rx_resp  = twr->rx_resp;
    twr_query_result.tx_final = twr->tx_final;
    twr_query_result.rx_final = twr->rx_final;

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &twr_query_result, sizeof(twr_query_result));
}

/* CCCD changed callback for TWR query result notifications */
static void twr_query_result_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    twr_query_result_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    twr_query_last_tx_init = 0; /* Reset dedup on subscribe change */
    LOG_INF("TWR query result notifications %s",
            twr_query_result_notifications_enabled ? "enabled" : "disabled");
}

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)

/* Read handler for MM query result characteristic */
static ssize_t read_mm_query_result(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset)
{
    if (!query_active) {
        struct mm_query_result_ble no_query;
        memset(&no_query, 0, sizeof(no_query));
        no_query.status = QUERY_STATUS_NO_QUERY;
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &no_query, sizeof(no_query));
    }

    struct node_entry entry_copy;
    if (node_table_get_copy(query_node_id, &entry_copy) != 0) {
        struct mm_query_result_ble not_found;
        memset(&not_found, 0, sizeof(not_found));
        not_found.status = QUERY_STATUS_NOT_FOUND;
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &not_found, sizeof(not_found));
    }

    mm_query_result.status = QUERY_STATUS_OK;
    mm_query_result.node_id = query_node_id;
    mm_query_result.twr_distance_um = entry_copy.last_twr_distance_um;
    mm_query_result.phase_mrad = entry_copy.last_phase_mrad;
    mm_query_result.coarse_phase_mrad = entry_copy.last_coarse_phase_mrad;
    mm_query_result.channel = entry_copy.last_mm_channel;
    mm_query_result.cfo_ppb = entry_copy.last_cfo_ppb;
    mm_query_result.rssi = entry_copy.rssi;
    mm_query_result._pad = 0;

    int64_t age_ticks = k_uptime_ticks() - (int64_t)entry_copy.last_seen_rtc;
    if (age_ticks < 0) age_ticks = 0;
    uint64_t age_ms = k_ticks_to_ms_floor64(age_ticks);
    mm_query_result.age_ms = (age_ms > UINT16_MAX) ? UINT16_MAX : (uint16_t)age_ms;

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &mm_query_result, sizeof(mm_query_result));
}

/* CCCD changed callback for MM query result notifications */
static void mm_query_result_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    mm_query_result_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    mm_query_last_notified_twr = INT32_MIN;
    LOG_INF("MM query result notifications %s",
            mm_query_result_notifications_enabled ? "enabled" : "disabled");
}

#endif /* CONFIG_NODE_TABLE_MM_ENABLED */

/* Neighbor table service definition */
BT_GATT_SERVICE_DEFINE(neighbor_table_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_NEIGHBOR_TABLE_SERVICE),

    /* Neighbor count characteristic (read-only) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NEIGHBOR_COUNT,
                          BT_GATT_CHRC_READ,
                          BT_GATT_PERM_READ,
                          read_neighbor_count, NULL, NULL),
    BT_GATT_CUD("Count", BT_GATT_PERM_READ),

    /* Neighbor list characteristic (read + notify) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NEIGHBOR_LIST,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_neighbor_list, NULL, NULL),
    BT_GATT_CCC(neighbor_list_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("Neighbors", BT_GATT_PERM_READ),

    /* Query address characteristic (write only) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NEIGHBOR_QUERY_ADDR,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, write_query_addr, NULL),
    BT_GATT_CUD("Query Addr", BT_GATT_PERM_READ),

    /* Query result characteristic (read + notify) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NEIGHBOR_QUERY_RESULT,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_query_result, NULL, NULL),
    BT_GATT_CCC(query_result_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("Query Result", BT_GATT_PERM_READ),

    /* Position stream characteristic (read + notify) */
    BT_GATT_CHARACTERISTIC(BT_UUID_POSITION_STREAM,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_position, NULL, NULL),
    BT_GATT_CCC(position_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("Position", BT_GATT_PERM_READ),

    /* TWR query result characteristic (read + notify) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NEIGHBOR_TWR_QUERY_RESULT,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_twr_query_result, NULL, NULL),
    BT_GATT_CCC(twr_query_result_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("TWR Result", BT_GATT_PERM_READ),

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
    /* MM query result characteristic (read + notify) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NEIGHBOR_MM_QUERY_RESULT,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_mm_query_result, NULL, NULL),
    BT_GATT_CCC(mm_query_result_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("MM Result", BT_GATT_PERM_READ),
#endif

#if defined(CONFIG_SYNCHROFLY_BLOCK_MULOC)
    /* MULoc round data characteristic (read + notify, variable size) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NEIGHBOR_MULOC_RESULT,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_muloc_result, NULL, NULL),
    BT_GATT_CCC(muloc_result_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("MULoc", BT_GATT_PERM_READ),
#endif

#if defined(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
    /* CIR data characteristic (read + notify, chunked) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NEIGHBOR_CIR_DATA,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_cir_data, NULL, NULL),
    BT_GATT_CCC(cir_data_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("CIR Data", BT_GATT_PERM_READ),

#endif

    /* Eviction timeout characteristic (read + write, uint32_t ms) */
    BT_GATT_CHARACTERISTIC(BT_UUID_NEIGHBOR_EVICTION_TIMEOUT,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_eviction_timeout, write_eviction_timeout, NULL),
    BT_GATT_CUD("Eviction Ms", BT_GATT_PERM_READ),
);

/* Work handler for deferred full table notifications */
static void neighbor_notify_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    neighbor_table_service_notify();
}

/* Callback for node position changes - triggers BLE position notification */
static void on_node_position_changed(float x, float y, float z)
{
    neighbor_table_service_notify_position(x, y, z);
}

/* Callback for node table changes -- called via node_table_notify_changed()
 * OUTSIDE the node table mutex. Safe to call any node_table API. */
static void on_node_table_changed(uint16_t node_id, const struct node_entry *entry)
{
    /* Trigger full table notification via work queue */
    if (notifications_enabled) {
        k_work_submit(&neighbor_notify_work);
    }

    /* Query result notification - only for matching node */
    if (query_result_notifications_enabled && query_active && node_id == query_node_id) {
        if (entry) {
            query_result.status = QUERY_STATUS_OK;
            convert_entry(entry, &query_result.entry, k_uptime_ticks());

            query_last_notified_distance = query_result.entry.distance_mm;
        } else {
            /* Node was removed - always notify */
            query_result.status = QUERY_STATUS_NOT_FOUND;
            memset(&query_result.entry, 0, sizeof(query_result.entry));
            query_last_notified_distance = INT32_MIN;
        }
        int err = bt_gatt_notify(NULL, query_result_attr, &query_result, sizeof(query_result));
        if (err) {
            LOG_WRN("Failed to send query result notification: %d", err);
        } else {
            LOG_DBG("Sent query result notification for 0x%04x", node_id);
        }
    }

    /* TWR query result notification - send raw timestamps when new TWR data arrives */
    if (twr_query_result_notifications_enabled && query_active && node_id == query_node_id) {
        if (entry) {
            const struct node_twr_timestamps *twr = &entry->last_twr;
            /* Only notify if tx_init changed (indicates a new ranging round) */
            if (twr->tx_init != 0 && twr->tx_init != twr_query_last_tx_init) {
                twr_query_last_tx_init = twr->tx_init;
                twr_query_result.status = QUERY_STATUS_OK;
                twr_query_result.initiator_id = twr->initiator_id;
                twr_query_result.responder_id = twr->responder_id;
                twr_query_result.tx_init  = twr->tx_init;
                twr_query_result.rx_init  = twr->rx_init;
                twr_query_result.tx_resp  = twr->tx_resp;
                twr_query_result.rx_resp  = twr->rx_resp;
                twr_query_result.tx_final = twr->tx_final;
                twr_query_result.rx_final = twr->rx_final;
                int err = bt_gatt_notify(NULL, twr_query_result_attr,
                                         &twr_query_result, sizeof(twr_query_result));
                if (err) {
                    LOG_WRN("Failed to send TWR query result notification: %d", err);
                } else {
                    LOG_DBG("Sent TWR notification for 0x%04x (tx_init=%lld)",
                            node_id, twr->tx_init);
                }
            }
        }
    }

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
    /* MM query result notification - stream single-channel MM data */
    if (mm_query_result_notifications_enabled && query_active && node_id == query_node_id) {
        if (entry) {
            mm_query_result.status = QUERY_STATUS_OK;
            mm_query_result.node_id = node_id;
            mm_query_result.twr_distance_um = entry->last_twr_distance_um;
            mm_query_result.phase_mrad = entry->last_phase_mrad;
            mm_query_result.coarse_phase_mrad = entry->last_coarse_phase_mrad;
            mm_query_result.channel = entry->last_mm_channel;
            mm_query_result.cfo_ppb = entry->last_cfo_ppb;
            mm_query_result.rssi = entry->rssi;
            mm_query_result._pad = 0;

            int64_t age_ticks = k_uptime_ticks() - (int64_t)entry->last_seen_rtc;
            if (age_ticks < 0) age_ticks = 0;
            uint64_t age_ms = k_ticks_to_ms_floor64(age_ticks);
            mm_query_result.age_ms = (age_ms > UINT16_MAX) ? UINT16_MAX : (uint16_t)age_ms;

            mm_query_last_notified_twr = mm_query_result.twr_distance_um;
            int err = bt_gatt_notify(NULL, mm_query_result_attr,
                                     &mm_query_result, sizeof(mm_query_result));
            if (err) {
                LOG_WRN("Failed to send MM notification: %d", err);
            }
        } else {
            memset(&mm_query_result, 0, sizeof(mm_query_result));
            mm_query_result.status = QUERY_STATUS_NOT_FOUND;
            mm_query_last_notified_twr = INT32_MIN;
        }
    }
#endif
}

int neighbor_table_service_init(void)
{
    /* Find the attribute handles for notifications.
     * Service attribute layout:
     *  0: Primary Service
     *  1: Characteristic declaration (count)
     *  2: Characteristic value (count)
     *  3: CUD (count)
     *  4: Characteristic declaration (list)
     *  5: Characteristic value (list) <- neighbor_list_attr
     *  6: CCC (list)
     *  7: CUD (list)
     *  8: Characteristic declaration (query addr)
     *  9: Characteristic value (query addr)
     * 10: CUD (query addr)
     * 11: Characteristic declaration (query result)
     * 12: Characteristic value (query result) <- query_result_attr
     * 13: CCC (query result)
     * 14: CUD (query result)
     * 15: Characteristic declaration (position stream)
     * 16: Characteristic value (position stream) <- position_stream_attr
     * 17: CCC (position stream)
     * 18: CUD (position stream)
     * 19: Characteristic declaration (TWR query result)
     * 20: Characteristic value (TWR query result) <- twr_query_result_attr
     * 21: CCC (TWR query result)
     * 22: CUD (TWR query result)
     */
    neighbor_list_attr = &neighbor_table_svc.attrs[5];
    query_result_attr = &neighbor_table_svc.attrs[12];
    position_stream_attr = &neighbor_table_svc.attrs[16];
    twr_query_result_attr = &neighbor_table_svc.attrs[20];

#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
    /* MM query result attr is after TWR Result:
     * TWR CUD is at index 22, then:
     *  23: MM Characteristic declaration
     *  24: MM Characteristic value <- mm_query_result_attr
     *  25: MM CCC
     *  26: MM CUD
     */
    mm_query_result_attr = &neighbor_table_svc.attrs[24];
#endif

#if defined(CONFIG_SYNCHROFLY_BLOCK_MULOC) || defined(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
    {
        /* Dynamic attr index calculation: conditional characteristics shift indices.
         * Base: after TWR CUD at index 22.
         * +4 if CONFIG_NODE_TABLE_MM_ENABLED (MM decl+val+ccc+cud)
         * +4 if CONFIG_SYNCHROFLY_BLOCK_MULOC (MULoc decl+val+ccc+cud)
         */
        int dyn_base = 23; /* After TWR CUD (index 22) */
#if defined(CONFIG_NODE_TABLE_MM_ENABLED)
        dyn_base += 4; /* Skip MM characteristic (4 attrs) */
#endif
#if defined(CONFIG_SYNCHROFLY_BLOCK_MULOC)
        muloc_result_attr = &neighbor_table_svc.attrs[dyn_base];
        dyn_base += 4; /* Skip MULoc characteristic (decl+val+ccc+cud) */
#endif
#if defined(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
        /* CIR Data characteristic value attr (same pattern as other chars) */
        cir_data_attr = &neighbor_table_svc.attrs[dyn_base];
#endif
    }
#endif

#if defined(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
    k_work_init(&cir_transfer_work, cir_transfer_work_handler);
#endif

    /* Register callback for node table changes to push BLE notifications */
    node_table_register_callback(on_node_table_changed);

    /* Register callback for position changes to push BLE position notifications */
    node_register_position_callback(on_node_position_changed);

    LOG_INF("Neighbor table service initialized");
    return 0;
}

int neighbor_table_service_notify(void)
{
    if (!notifications_enabled) {
        return -ENOENT;
    }

    size_t count = refresh_neighbor_buffer();
    size_t data_len = count * sizeof(struct neighbor_entry_ble);

    int err = bt_gatt_notify(NULL, neighbor_list_attr, neighbor_buffer, data_len);
    if (err) {
        LOG_WRN("Failed to send neighbor notification: %d", err);
        return err;
    }

    LOG_DBG("Sent neighbor notification: %zu entries", count);
    return 0;
}

int neighbor_table_service_notify_position(float x, float y, float z)
{
    if (!position_notifications_enabled) {
        return -ENOENT;
    }

    current_position.pos_x = x;
    current_position.pos_y = y;
    current_position.pos_z = z;
    current_position.timestamp = k_uptime_get_32();

    int err = bt_gatt_notify(NULL, position_stream_attr,
                             &current_position, sizeof(current_position));
    if (err) {
        LOG_WRN("Failed to send position notification: %d", err);
        return err;
    }

    LOG_DBG("Sent position notification: (%.3f, %.3f, %.3f)",
            (double)x, (double)y, (double)z);
    return 0;
}

#if defined(CONFIG_SYNCHROFLY_BLOCK_MULOC)
#include <app/lib/blocks/muloc.h>

int neighbor_table_notify_muloc_round(const struct muloc_round_result *round,
                                       uint8_t anchor_id, uint8_t anchor_count)
{
    if (!muloc_result_notifications_enabled) {
        return 0;
    }

    /* Build notification: [anchor_id:1][anchor_count:1][round_index:1][channel:1][payload][frame_seq:1] */
    size_t offset = 0;
    muloc_notify_buf[offset++] = anchor_id;
    muloc_notify_buf[offset++] = anchor_count;
    muloc_notify_buf[offset++] = round->round_index;
    muloc_notify_buf[offset++] = round->channel;

    if (anchor_id == 0xFF) {
        /* Tag mode: pack anchor_count TO records (17 bytes each) */
        for (uint8_t i = 0; i < anchor_count && i < 8; i++) {
            if (round->to_valid_mask & BIT(i)) {
                /* Copy AO fields (13 bytes) */
                memcpy(&muloc_notify_buf[offset], &round->to_records[i].ao,
                       sizeof(struct muloc_ao_record));
                offset += sizeof(struct muloc_ao_record);
                /* Carrier integrator (4 bytes LE) */
                memcpy(&muloc_notify_buf[offset],
                       &round->to_records[i].carrier_integrator, 4);
                offset += 4;
            } else {
                memset(&muloc_notify_buf[offset], 0, 17);
                offset += 17;
            }
        }
    } else {
        /* Anchor mode: pack (anchor_count-1) AO records (13 bytes each) */
        for (uint8_t i = 0; i < anchor_count && i < 8; i++) {
            if (i == anchor_id) {
                continue; /* Skip self */
            }
            if (round->ao_valid_mask & BIT(i)) {
                memcpy(&muloc_notify_buf[offset], &round->ao_records[i],
                       sizeof(struct muloc_ao_record));
            } else {
                memset(&muloc_notify_buf[offset], 0, sizeof(struct muloc_ao_record));
            }
            offset += sizeof(struct muloc_ao_record);
        }
    }

    /* Append frame_seq */
    muloc_notify_buf[offset++] = round->frame_seq;

    muloc_notify_len = offset;
    int err = bt_gatt_notify(NULL, muloc_result_attr, muloc_notify_buf, offset);
    if (err) {
        LOG_WRN("Failed to send MULoc notification: %d", err);
    }
    return err;
}
#endif /* CONFIG_SYNCHROFLY_BLOCK_MULOC */

#if defined(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
int neighbor_table_service_notify_cir(const struct cir_read_block_result *result)
{
    if (!cir_data_notifications_enabled || !result || !result->cir_data) {
        return -ENOENT;
    }

    if (cir_transfer_active) {
        LOG_WRN("CIR transfer already in progress, dropping");
        return -EBUSY;
    }

    /* Heap-allocate and copy CIR data for async transfer */
    cir_pending_byte_len = result->sample_count * 4;
    cir_pending_data = block_malloc(cir_pending_byte_len);
    if (!cir_pending_data) {
        LOG_ERR("CIR notify: failed to allocate %u bytes", cir_pending_byte_len);
        return -ENOMEM;
    }
    memcpy(cir_pending_data, result->cir_data, cir_pending_byte_len);

    cir_pending_sample_count = result->sample_count;
    cir_pending_fp_index = result->first_path_index;
    cir_pending_from_index = result->from_index;
    cir_pending_channel = result->channel;

    /* Calculate total chunks needed */
    uint16_t first_chunk_data = CIR_NOTIFY_MTU - CIR_CHUNK_IDX_SIZE - CIR_CHUNK_HEADER_SIZE;
    uint16_t subsequent_chunk_data = CIR_NOTIFY_MTU - CIR_CHUNK_IDX_SIZE;

    if (cir_pending_byte_len <= first_chunk_data) {
        cir_pending_total_chunks = 1;
    } else {
        uint16_t remaining = cir_pending_byte_len - first_chunk_data;
        cir_pending_total_chunks = 1 + (remaining + subsequent_chunk_data - 1) / subsequent_chunk_data;
    }

    cir_pending_next_chunk = 0;
    cir_transfer_active = true;

    LOG_INF("CIR transfer starting: %u samples, %u bytes, %u chunks",
            cir_pending_sample_count, cir_pending_byte_len, cir_pending_total_chunks);

    k_work_submit(&cir_transfer_work);
    return 0;
}
#endif /* CONFIG_SYNCHROFLY_BLOCK_CIR_READ */
