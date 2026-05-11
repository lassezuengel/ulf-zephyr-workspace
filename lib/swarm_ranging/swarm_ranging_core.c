/**
 * @file swarm_ranging_core.c
 * @brief Swarm ranging core logic - state machine and distance computation
 *
 * Port of SEU-NetSI swarm ranging core to Zephyr.
 * CRITICAL: Logic preserved exactly to maintain algorithmic symmetry.
 */

#include <app/lib/swarm_ranging/swarm_ranging_core.h>
#include <app/lib/swarm_ranging/ranging_table.h>
#include <app/lib/swarm_ranging/neighbor_management.h>
#include <app/lib/swarm_ranging/velocity_mock.h>
#include <app/lib/swarm_ranging/swarm_uwb_interface.h>
#include <app/lib/swarm_ranging/platform_zephyr.h>
#include <app/lib/system/block_heap.h>
#if IS_ENABLED(CONFIG_NODE_TABLE)
#include <app/lib/node_table/node_table.h>
#endif
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(swarm_ranging_core, LOG_LEVEL_NONE);

/* ============================================================================
 * Global State
 * ============================================================================ */

static uint16_t MY_UWB_ADDRESS = 0;
static int rangingSeqNumber = 1;
static int last_sent_seq = 0;  // Store message seq before TX for callback

/* Message queues */
K_MSGQ_DEFINE(ranging_rx_msgq, sizeof(Ranging_Message_With_Timestamp_t),
              RANGING_RX_QUEUE_SIZE, 4);

/* Thread handles */
static struct k_thread tx_thread_data;
static struct k_thread rx_thread_data;

#ifndef CONFIG_SWARM_RANGING_TX_STACK_SIZE
#define CONFIG_SWARM_RANGING_TX_STACK_SIZE 1024
#endif
#ifndef CONFIG_SWARM_RANGING_RX_STACK_SIZE
#define CONFIG_SWARM_RANGING_RX_STACK_SIZE 1024
#endif

K_THREAD_STACK_DEFINE(tx_thread_stack, CONFIG_SWARM_RANGING_TX_STACK_SIZE);
K_THREAD_STACK_DEFINE(rx_thread_stack, CONFIG_SWARM_RANGING_RX_STACK_SIZE);

/* Deferred node table notification -- runs on system work queue to avoid
 * blowing the tiny RX thread stack with BLE notify + filter call chains. */
#if IS_ENABLED(CONFIG_NODE_TABLE)
static void node_table_notify_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	node_table_notify_changed();
}
static K_WORK_DEFINE(node_table_notify_work, node_table_notify_work_handler);
#endif

/* Eviction work items (must run in thread context for k_mutex_lock) */
static void neighbor_evict_work_handler(struct k_work *work);
static void ranging_table_evict_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(neighbor_evict_work, neighbor_evict_work_handler);
static K_WORK_DELAYABLE_DEFINE(ranging_table_evict_work, ranging_table_evict_work_handler);

/* Runtime-configurable base ranging period (initialized from Kconfig) */
static uint16_t ranging_period_ms = RANGING_PERIOD;

/* Runtime-configurable distance filter (enabled by default) */
static bool distance_filter_enabled = true;

/* Runtime-configurable bus boarding scheme (compile-time default from Kconfig) */
#ifdef CONFIG_SWARM_ENABLE_BUS_BOARDING
static bool bus_boarding_enabled = true;
#else
static bool bus_boarding_enabled = false;
#endif

/* ============================================================================
 * JSON Logging (MTM-compatible format)
 * ============================================================================ */

/**
 * Log distance measurement in JSON format compatible with MTM app
 * Format matches log_measurements_json() from lib/logging/log.c
 */
static void log_swarm_distance_json(uint16_t local_addr, uint16_t neighbor_addr,
                                    int16_t distance_cm, uint64_t timestamp)
{
    if (!IS_ENABLED(CONFIG_SYNCHROFLY_LOG_TWR_JSON)) {
        return;
    }
    printk("{\"event\":\"twr\",\"local_id\":\"0x%x\",\"rtc_round_ts\":%llu,\"unit\":\"cm\",\"t\":[",
           local_addr, timestamp);
    printk("{\"i\":[\"0x%x\",\"0x%x\"],\"d\":%d,\"q\":%d}",
           local_addr, neighbor_addr, distance_cm, 100);
    printk("]}\n");
}

/* ============================================================================
 * Distance Computation (UNCHANGED ALGORITHM - PRESERVE EXACTLY)
 * ============================================================================ */

int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                       Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                       Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf)
{
    bool isErrorOccurred = false;

    LOG_DBG("computeDistance: Tp.seq=%d, Rp.seq=%d, Tr.seq=%d, Rr.seq=%d, Tf.seq=%d, Rf.seq=%d",
            Tp.seqNumber, Rp.seqNumber, Tr.seqNumber, Rr.seqNumber, Tf.seqNumber, Rf.seqNumber);

    /* Validate sequence numbers */
    if (Tp.seqNumber != Rp.seqNumber || Tr.seqNumber != Rr.seqNumber ||
        Tf.seqNumber != Rf.seqNumber) {
        LOG_DBG("Ranging Error: sequence number mismatch");
        isErrorOccurred = true;
    }

    if (Tp.seqNumber >= Tf.seqNumber || Rp.seqNumber >= Rf.seqNumber) {
        LOG_DBG("Ranging Error: sequence number out of order");
        isErrorOccurred = true;
    }

    /* Two-Way Ranging calculation */
    int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, t;
    tRound1 = (Rr.timestamp - Tp.timestamp + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    tReply1 = (Tr.timestamp - Rp.timestamp + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    tRound2 = (Rf.timestamp - Tr.timestamp + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    tReply2 = (Tf.timestamp - Rr.timestamp + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    diff1 = tRound1 - tReply1;
    diff2 = tRound2 - tReply2;
    t = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) /
        (tRound1 + tRound2 + tReply1 + tReply2);

    /* CRITICAL: Preserve exact conversion constant */
    int16_t distance = (int16_t)(t * 0.4691763978616);

    if (distance_filter_enabled) {
        if (distance < 0) {
            LOG_DBG("Ranging Error: distance < 0");
            isErrorOccurred = true;
        }

        if (distance > 1000) {
            LOG_DBG("Ranging Error: distance > 1000");
            isErrorOccurred = true;
        }

        if (isErrorOccurred) {
            return -1;
        }
    }

    return distance;
}

/* ============================================================================
 * State Machine Event Handlers (UNCHANGED LOGIC - PRESERVE EXACTLY)
 * ============================================================================ */

/* S1 Event Handlers */
static void S1_Tf(Ranging_Table_t *rangingTable)
{
    /* Don't update Tf here since sending message is an async action */
    rangingTable->state = RANGING_STATE_S2;
}

static void S1_RX_NO_Rf(Ranging_Table_t *rangingTable)
{
    /* Invalid state transition, ignore */
    rangingTable->state = RANGING_STATE_S1;
}

static void S1_RX_Rf(Ranging_Table_t *rangingTable)
{
    /* Invalid state transition, ignore */
    rangingTable->state = RANGING_STATE_S1;
}

/* S2 Event Handlers */
static void S2_Tf(Ranging_Table_t *rangingTable)
{
    /* Don't update Tf here since sending message is an async action */
    rangingTable->state = RANGING_STATE_S2;
}

static void S2_RX_NO_Rf(Ranging_Table_t *rangingTable)
{
    /* Invalid state transition, ignore */
    rangingTable->state = RANGING_STATE_S2;
}

static void S2_RX_Rf(Ranging_Table_t *rangingTable)
{
    /* Find corresponding Tf in TfBuffer */
    rangingTable->Tf = findTfBySeqNumber(rangingTable->Rf.seqNumber);
    if (!rangingTable->Tf.timestamp) {
        LOG_WRN("Cannot find corresponding Tf in buffer (freq too high or buffer too small)");
    }

    /* Shift ranging table: Rp <- Rf, Tp <- Tf, Rr <- Re */
    rangingTable->Rp = rangingTable->Rf;
    rangingTable->Tp = rangingTable->Tf;
    rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;

    Timestamp_Tuple_t empty = {.timestamp = 0, .seqNumber = 0};
    rangingTable->Rf = empty;
    rangingTable->Tf = empty;
    rangingTable->Re = empty;

    rangingTable->state = RANGING_STATE_S3;
}

/* S3 Event Handlers */
static void S3_Tf(Ranging_Table_t *rangingTable)
{
    /* Don't update Tf here since sending message is an async action */
    rangingTable->state = RANGING_STATE_S4;
}

static void S3_RX_NO_Rf(Ranging_Table_t *rangingTable)
{
    /* Shift: Rr <- Re */
    rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;
    Timestamp_Tuple_t empty = {.timestamp = 0, .seqNumber = 0};
    rangingTable->Re = empty;

    rangingTable->state = RANGING_STATE_S3;
}

static void S3_RX_Rf(Ranging_Table_t *rangingTable)
{
    /* Shift: Rr <- Re */
    rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;
    Timestamp_Tuple_t empty = {.timestamp = 0, .seqNumber = 0};
    rangingTable->Re = empty;

    rangingTable->state = RANGING_STATE_S3;
}

/* S4 Event Handlers */
static void S4_Tf(Ranging_Table_t *rangingTable)
{
    /* Don't update Tf here since sending message is an async action */
    rangingTable->state = RANGING_STATE_S4;
}

static void S4_RX_NO_Rf(Ranging_Table_t *rangingTable)
{
    /* Shift: Rr <- Re */
    rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;
    Timestamp_Tuple_t empty = {.timestamp = 0, .seqNumber = 0};
    rangingTable->Re = empty;

    rangingTable->state = RANGING_STATE_S4;
}

static void S4_RX_Rf(Ranging_Table_t *rangingTable)
{
    /* Find corresponding Tf in TfBuffer */
    rangingTable->Tf = findTfBySeqNumber(rangingTable->Rf.seqNumber);

    Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate =
        rangingTableBufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Tf);

    /* Try to compute distance */
    int16_t distance = computeDistance(rangingTable->Tp, rangingTable->Rp,
                                      Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,
                                      rangingTable->Tf, rangingTable->Rf);
    if (distance > 0) {
        rangingTable->distance = distance;
        setDistance(rangingTable->neighborAddress, distance);
        LOG_DBG("Distance to %u: %d cm", rangingTable->neighborAddress, distance);

#if IS_ENABLED(CONFIG_NODE_TABLE)
        node_table_update(rangingTable->neighborAddress,
                          (int32_t)distance * 10, k_uptime_ticks());
        k_work_submit(&node_table_notify_work);
#endif

        /* Output distance in MTM-compatible JSON format */
        log_swarm_distance_json(MY_UWB_ADDRESS, rangingTable->neighborAddress,
                                distance, xTaskGetTickCount());
    }

    /* Shift ranging table: Rp <- Rf, Tp <- Tf, Rr <- Re */
    rangingTable->Rp = rangingTable->Rf;
    rangingTable->Tp = rangingTable->Tf;
    rangingTable->TrRrBuffer.candidates[rangingTable->TrRrBuffer.cur].Rr = rangingTable->Re;

    Timestamp_Tuple_t empty = {.timestamp = 0, .seqNumber = 0};
    rangingTable->Rf = empty;
    rangingTable->Tf = empty;
    rangingTable->Re = empty;

    rangingTable->state = RANGING_STATE_S3;
}

/* S5 Event Handlers (temporary state - should not be called) */
static void S5_Tf(Ranging_Table_t *rangingTable)
{
    LOG_WRN("S5_Tf: invalid handler invocation");
}

static void S5_RX_NO_Rf(Ranging_Table_t *rangingTable)
{
    LOG_WRN("S5_RX_NO_Rf: invalid handler invocation");
}

static void S5_RX_Rf(Ranging_Table_t *rangingTable)
{
    LOG_WRN("S5_RX_Rf: invalid handler invocation");
}

/* Reserved stub */
static void RESERVED_STUB(Ranging_Table_t *rangingTable)
{
    LOG_ERR("RESERVED_STUB: Error, been invoked unexpectedly");
}

/* Event handler table */
static RangingTableEventHandler EVENT_HANDLER[RANGING_TABLE_STATE_COUNT][RANGING_TABLE_EVENT_COUNT] = {
    {RESERVED_STUB, RESERVED_STUB, RESERVED_STUB},
    {S1_Tf, S1_RX_NO_Rf, S1_RX_Rf},
    {S2_Tf, S2_RX_NO_Rf, S2_RX_Rf},
    {S3_Tf, S3_RX_NO_Rf, S3_RX_Rf},
    {S4_Tf, S4_RX_NO_Rf, S4_RX_Rf},
    {S5_Tf, S5_RX_NO_Rf, S5_RX_Rf}
};

void rangingTableOnEvent(Ranging_Table_t *table, RANGING_TABLE_EVENT event)
{
    ASSERT(table->state < RANGING_TABLE_STATE_COUNT);
    ASSERT(event < RANGING_TABLE_EVENT_COUNT);
    EVENT_HANDLER[table->state][event](table);
}

/* ============================================================================
 * Message Processing (UNCHANGED LOGIC - PRESERVE EXACTLY)
 * ============================================================================ */

void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp,
                          uint16_t my_address)
{
    Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
    uint16_t neighborAddress = rangingMessage->header.srcAddress;

    Ranging_Table_Set_t *rangingTableSet = getGlobalRangingTableSet();
    int neighborIndex = -1;

    /* Search for existing table */
    for (int i = 0; i < rangingTableSet->size; i++) {
        if (rangingTableSet->tables[i].neighborAddress == neighborAddress) {
            neighborIndex = i;
            break;
        }
    }

    /* Handle new neighbor */
    if (neighborIndex == -1) {
        static Ranging_Table_t table;  /* static: called only from ranging_rx_thread */
        rangingTableInit(&table, neighborAddress);
        if (!rangingTableSetAddTable(rangingTableSet, table)) {
            LOG_WRN("Ranging table full (%d), cannot handle new neighbor %d",
                    rangingTableSet->size, neighborAddress);
            return;
        } else {
            /* Find the newly added table */
            for (int i = 0; i < rangingTableSet->size; i++) {
                if (rangingTableSet->tables[i].neighborAddress == neighborAddress) {
                    neighborIndex = i;
                    break;
                }
            }
        }
    }

    Ranging_Table_t *neighborRangingTable = &rangingTableSet->tables[neighborIndex];

    /* Update Re */
    neighborRangingTable->Re.timestamp = rangingMessageWithTimestamp->rxTime;
    neighborRangingTable->Re.seqNumber = rangingMessage->header.msgSequence;

    /* Update latest received timestamp */
    neighborRangingTable->latestReceived = neighborRangingTable->Re;

    /* Update expiration time */
    neighborRangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);

    /* Find corresponding Tr according to Rr to get valid Tr-Rr pair */
    Ranging_Table_Tr_Rr_Buffer_t *neighborTrRrBuffer = &neighborRangingTable->TrRrBuffer;
    for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++) {
        if (rangingMessage->header.lastTxTimestamps[i].timestamp &&
            neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.timestamp &&
            rangingMessage->header.lastTxTimestamps[i].seqNumber ==
                neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.seqNumber) {
            rangingTableBufferUpdate(&neighborRangingTable->TrRrBuffer,
                                   rangingMessage->header.lastTxTimestamps[i],
                                   neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr);
            break;
        }
    }

    /* Try to find corresponding Rf for MY_UWB_ADDRESS */
    Timestamp_Tuple_t neighborRf = {.timestamp = 0, .seqNumber = 0};
    if (rangingMessage->header.filter & (1 << (my_address % 16))) {
        uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) /
                               sizeof(Body_Unit_t);
        for (int i = 0; i < bodyUnitCount; i++) {
            if (rangingMessage->bodyUnits[i].address == my_address) {
                neighborRf = rangingMessage->bodyUnits[i].timestamp;
                break;
            }
        }
    }

    /* Trigger event handler according to Rf */
    if (neighborRf.timestamp) {
        neighborRangingTable->Rf = neighborRf;
        rangingTableOnEvent(neighborRangingTable, RANGING_EVENT_RX_Rf);
    } else {
        rangingTableOnEvent(neighborRangingTable, RANGING_EVENT_RX_NO_Rf);
    }

#ifdef CONFIG_SWARM_ENABLE_DYNAMIC_PERIOD
    /* Update period according to distance and velocity */
    neighborRangingTable->period = M2T(DYNAMIC_RANGING_COEFFICIENT *
        (neighborRangingTable->distance / rangingMessage->header.velocity));
    /* Bound ranging period */
    neighborRangingTable->period = MAX(neighborRangingTable->period, M2T(RANGING_PERIOD_MIN));
    neighborRangingTable->period = MIN(neighborRangingTable->period, M2T(RANGING_PERIOD_MAX));
#endif
}

Time_t generateRangingMessage(Ranging_Message_t *rangingMessage, uint16_t my_address)
{
    int8_t bodyUnitNumber = 0;
    rangingSeqNumber++;
    int curSeqNumber = rangingSeqNumber;
    rangingMessage->header.filter = 0;
    Time_t curTime = xTaskGetTickCount();
    Time_t taskDelay = M2T(ranging_period_ms);

    Ranging_Table_Set_t *rangingTableSet = getGlobalRangingTableSet();

    if (bus_boarding_enabled) {
        rangingTableSetSortByNextDelivery(rangingTableSet);
    } else {
        rangingTableSetSortByLastSend(rangingTableSet);
    }

    /* Generate message body */
    for (int index = 0; index < rangingTableSet->size; index++) {
        if (bodyUnitNumber >= RANGING_MAX_BODY_UNIT) {
            break;
        }

        Ranging_Table_t *table = &rangingTableSet->tables[index];
        if (table->latestReceived.timestamp) {
            /* Only include timestamps with expected delivery time <= current time */
            if (table->nextExpectedDeliveryTime > curTime) {
                continue;
            }

            table->nextExpectedDeliveryTime = curTime + M2T(table->period);
            table->lastSendTime = curTime;
            rangingMessage->bodyUnits[bodyUnitNumber].address = table->neighborAddress;
            rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->latestReceived;
            rangingMessage->header.filter |= 1 << (table->neighborAddress % 16);
            rangingTableOnEvent(table, RANGING_EVENT_TX_Tf);

#ifdef CONFIG_SWARM_ENABLE_DYNAMIC_PERIOD
            taskDelay = MIN(taskDelay, table->nextExpectedDeliveryTime - curTime);
            taskDelay = MAX(RANGING_PERIOD_MIN, taskDelay);
#endif
            bodyUnitNumber++;
        }
    }

    /* Generate message header */
    rangingMessage->header.srcAddress = my_address;
    rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) +
                                      sizeof(Body_Unit_t) * bodyUnitNumber;
    rangingMessage->header.msgSequence = curSeqNumber;
    getLatestNTxTimestamps(rangingMessage->header.lastTxTimestamps, RANGING_MAX_Tr_UNIT);
    rangingMessage->header.velocity = velocity_get_magnitude_cm_s();

    LOG_DBG("TX: Generated message seq=%d, bodyUnits=%d", curSeqNumber, bodyUnitNumber);

    /* Keep ranging table in order for binary search */
    rangingTableSetSortByAddress(rangingTableSet);

    return taskDelay;
}

/* ============================================================================
 * Timer Callbacks
 * ============================================================================ */

static void neighbor_evict_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    Neighbor_Set_t *neighborSet = getGlobalNeighborSet();

    k_mutex_lock(&neighborSet->mu, K_FOREVER);
    Time_t curTime = xTaskGetTickCount();
    LOG_DBG("Neighbor eviction triggered at %u", curTime);

    int evictionCount = neighborSetClearExpire(neighborSet);
    if (evictionCount > 0) {
        LOG_DBG("Evicted %d neighbors", evictionCount);
    }

    k_mutex_unlock(&neighborSet->mu);

    /* Reschedule */
    k_work_reschedule(&neighbor_evict_work,
                      K_MSEC(NEIGHBOR_SET_HOLD_TIME / 2));
}

static void ranging_table_evict_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    Ranging_Table_Set_t *rangingTableSet = getGlobalRangingTableSet();

    k_mutex_lock(&rangingTableSet->mu, K_FOREVER);
    Time_t curTime = xTaskGetTickCount();
    LOG_DBG("Ranging table eviction triggered at %u", curTime);

    int evictionCount = rangingTableSetEvictExpired(rangingTableSet);
    if (evictionCount > 0) {
        LOG_DBG("Evicted %d ranging tables", evictionCount);
    }

    k_mutex_unlock(&rangingTableSet->mu);

    /* Reschedule */
    k_work_reschedule(&ranging_table_evict_work,
                      K_MSEC(RANGING_TABLE_HOLD_TIME / 2));
}

/* ============================================================================
 * Threading
 * ============================================================================ */

static void ranging_tx_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Ranging TX thread started");

    /* Wait a bit for system to stabilize */
    k_msleep(500);

    Ranging_Message_t rangingMessage;
    Ranging_Table_Set_t *rangingTableSet = getGlobalRangingTableSet();
    Neighbor_Set_t *neighborSet = getGlobalNeighborSet();

    while (1) {
        k_mutex_lock(&rangingTableSet->mu, K_FOREVER);
        k_mutex_lock(&neighborSet->mu, K_FOREVER);

        Time_t taskDelay = generateRangingMessage(&rangingMessage, MY_UWB_ADDRESS);

        /* Store message sequence for TX callback (matches original SEU-NetSI behavior) */
        last_sent_seq = rangingMessage.header.msgSequence;

        /* Send via UWB interface */
        int ret = swarm_uwb_send_packet_blocking((uint8_t *)&rangingMessage,
                                                 rangingMessage.header.msgLength);
        if (ret < 0) {
            LOG_ERR("Failed to send ranging message: %d", ret);
        }

        k_mutex_unlock(&neighborSet->mu);
        k_mutex_unlock(&rangingTableSet->mu);

        k_msleep(taskDelay);
    }
}

static void ranging_rx_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Ranging RX thread started");

    static Ranging_Message_With_Timestamp_t rxMsg;
    Ranging_Table_Set_t *rangingTableSet = getGlobalRangingTableSet();
    Neighbor_Set_t *neighborSet = getGlobalNeighborSet();

    while (1) {
        if (k_msgq_get(&ranging_rx_msgq, &rxMsg, K_FOREVER) == 0) {
            k_mutex_lock(&rangingTableSet->mu, K_FOREVER);
            k_mutex_lock(&neighborSet->mu, K_FOREVER);

            processRangingMessage(&rxMsg, MY_UWB_ADDRESS);
            topologySensing(&rxMsg.rangingMessage, MY_UWB_ADDRESS);

            k_mutex_unlock(&neighborSet->mu);
            k_mutex_unlock(&rangingTableSet->mu);
        }
        k_msleep(1);
    }
}

/* ============================================================================
 * UWB Callbacks
 * ============================================================================ */

static void ranging_rx_callback(swarm_uwb_rx_event_t *event)
{
    static Ranging_Message_With_Timestamp_t rxMsg;  /* static: single-threaded UWB RX context */
    rxMsg.rxTime = event->rx_timestamp;
    memcpy(&rxMsg.rangingMessage, event->data,
           MIN(event->length, sizeof(Ranging_Message_t)));

    k_msgq_put(&ranging_rx_msgq, &rxMsg, K_NO_WAIT);
}

static void ranging_tx_callback(uint64_t tx_timestamp, uint16_t seq_num)
{
    /* Use message sequence (last_sent_seq) instead of UWB interface counter (seq_num).
     * This matches the original SEU-NetSI behavior where the callback read the
     * sequence from the transmitted packet buffer: rangingMessage->header.msgSequence */
    Timestamp_Tuple_t timestamp = {
        .timestamp = tx_timestamp,
        .seqNumber = last_sent_seq  // Use message seq, not tx_sequence from UWB layer
    };

    LOG_DBG("TX_CALLBACK: Storing TfBuffer seq=%d (uwb_seq=%d ignored), ts=%llx",
            last_sent_seq, seq_num, tx_timestamp);

    updateTfBuffer(timestamp);
}

/* ============================================================================
 * Runtime Configuration
 * ============================================================================ */

void swarm_ranging_set_period(uint16_t period_ms)
{
    ranging_period_ms = CLAMP(period_ms, RANGING_PERIOD_MIN, RANGING_PERIOD_MAX);
    LOG_INF("Ranging period set to %u ms", ranging_period_ms);
}

uint16_t swarm_ranging_get_period(void)
{
    return ranging_period_ms;
}

void swarm_ranging_set_distance_filter(bool enabled)
{
	distance_filter_enabled = enabled;
	LOG_INF("Distance filter %s", enabled ? "enabled" : "disabled");
}

bool swarm_ranging_get_distance_filter(void)
{
	return distance_filter_enabled;
}

void swarm_ranging_set_bus_boarding(bool enabled)
{
	bus_boarding_enabled = enabled;
	LOG_INF("Bus boarding %s", enabled ? "enabled" : "disabled");
}

bool swarm_ranging_get_bus_boarding(void)
{
	return bus_boarding_enabled;
}

/* ============================================================================
 * Reset (clear all state, keep threads running)
 * ============================================================================ */

void swarm_ranging_reset(void)
{
    LOG_INF("Resetting swarm ranging state");

    /* Purge any pending RX messages first */
    k_msgq_purge(&ranging_rx_msgq);

    /* Re-initialize data structures (handles mutex init + clearing internally) */
    Neighbor_Set_t *neighborSet = getGlobalNeighborSet();
    neighborSetInit(neighborSet);

    Ranging_Table_Set_t *rangingTableSet = getGlobalRangingTableSet();
    rangingTableSetInit(rangingTableSet);

    /* Reset sequence number */
    rangingSeqNumber = 1;
    last_sent_seq = 0;

    /* Reschedule eviction timers */
    k_work_reschedule(&neighbor_evict_work,
                      K_MSEC(NEIGHBOR_SET_HOLD_TIME / 2));
    k_work_reschedule(&ranging_table_evict_work,
                      K_MSEC(RANGING_TABLE_HOLD_TIME / 2));

    LOG_INF("Swarm ranging reset complete");
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

int ranging_init(uint16_t uwb_address)
{
    MY_UWB_ADDRESS = uwb_address;

    LOG_INF("Initializing swarm ranging (addr: 0x%04x)", MY_UWB_ADDRESS);

    /* Initialize data structures */
    Neighbor_Set_t *neighborSet = getGlobalNeighborSet();
    neighborSetInit(neighborSet);

    Ranging_Table_Set_t *rangingTableSet = getGlobalRangingTableSet();
    rangingTableSetInit(rangingTableSet);

    /* Schedule eviction work (runs on system workqueue, safe for k_mutex_lock) */
    k_work_reschedule(&neighbor_evict_work,
                      K_MSEC(NEIGHBOR_SET_HOLD_TIME / 2));
    k_work_reschedule(&ranging_table_evict_work,
                      K_MSEC(RANGING_TABLE_HOLD_TIME / 2));

    /* Register UWB callbacks */
    extern void swarm_uwb_register_rx_callback(swarm_uwb_rx_callback_t callback);
    extern void swarm_uwb_register_tx_callback(swarm_uwb_tx_callback_t callback);
    swarm_uwb_register_rx_callback(ranging_rx_callback);
    swarm_uwb_register_tx_callback(ranging_tx_callback);

    /* Start threads */
#ifndef CONFIG_SWARM_RANGING_THREAD_PRIORITY
#define CONFIG_SWARM_RANGING_THREAD_PRIORITY 5
#endif

    k_thread_create(&tx_thread_data, tx_thread_stack,
                    K_THREAD_STACK_SIZEOF(tx_thread_stack),
                    ranging_tx_thread, NULL, NULL, NULL,
                    CONFIG_SWARM_RANGING_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&tx_thread_data, "ranging_tx");

    k_thread_create(&rx_thread_data, rx_thread_stack,
                    K_THREAD_STACK_SIZEOF(rx_thread_stack),
                    ranging_rx_thread, NULL, NULL, NULL,
                    CONFIG_SWARM_RANGING_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&rx_thread_data, "ranging_rx");

    LOG_INF("Swarm ranging initialized");
    return 0;
}

void ranging_deinit(void)
{
    /* Abort threads */
    k_thread_abort(&tx_thread_data);
    k_thread_abort(&rx_thread_data);

    /* Cancel eviction work */
    k_work_cancel_delayable(&neighbor_evict_work);
    k_work_cancel_delayable(&ranging_table_evict_work);

    /* Free heap allocations and reset global pointers */
    neighborSetDeinit();
    rangingTableSetDeinit();

    LOG_INF("Swarm ranging deinitialized");
}
