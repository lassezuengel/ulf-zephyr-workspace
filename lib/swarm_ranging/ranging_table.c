/**
 * @file ranging_table.c
 * @brief Ranging table management implementation
 *
 * Port of SEU-NetSI ranging table operations to Zephyr.
 * Logic preserved exactly, only RTOS primitives adapted.
 */

#include <app/lib/swarm_ranging/ranging_table.h>
#include <app/lib/swarm_ranging/platform_zephyr.h>
#include <app/lib/system/block_heap.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>

LOG_MODULE_REGISTER(ranging_table, LOG_LEVEL_NONE);

/* ============================================================================
 * Global State
 * ============================================================================ */

/* Global ranging table set -- heap-allocated to reduce static .bss usage */
static Ranging_Table_Set_t *rangingTableSet;

/* TfBuffer (transmitted timestamp buffer) */
static int TfBufferIndex = 0;
static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static struct k_mutex TfBufferMutex;

/* Empty ranging table template */
static const Ranging_Table_t EMPTY_RANGING_TABLE = {
    .neighborAddress = UWB_DEST_EMPTY,
    .Rp.timestamp = 0,
    .Rp.seqNumber = 0,
    .Tp.timestamp = 0,
    .Tp.seqNumber = 0,
    .Rf.timestamp = 0,
    .Rf.seqNumber = 0,
    .Tf.timestamp = 0,
    .Tf.seqNumber = 0,
    .Re.timestamp = 0,
    .Re.seqNumber = 0,
    .latestReceived.timestamp = 0,
    .latestReceived.seqNumber = 0,
    .TrRrBuffer.cur = 0,
    .TrRrBuffer.latest = 0,
    .state = RANGING_STATE_S1,
    .period = RANGING_PERIOD,
    .nextExpectedDeliveryTime = M2T(RANGING_PERIOD),
    .expirationTime = M2T(RANGING_TABLE_HOLD_TIME),
    .lastSendTime = 0,
    .distance = -1
};

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static int rangingTableSetSearchTable(Ranging_Table_Set_t *set, UWB_Address_t targetAddress);

/* ============================================================================
 * Distance Tracking (REFACTORED: Use ranging table search)
 * ============================================================================ */

int16_t getDistance(UWB_Address_t neighborAddress)
{
    int idx = rangingTableSetSearchTable(rangingTableSet, neighborAddress);
    if (idx < 0) {
        return -1;  /* Not found */
    }
    return rangingTableSet->tables[idx].distance;
}

void setDistance(UWB_Address_t neighborAddress, int16_t distance)
{
    int idx = rangingTableSetSearchTable(rangingTableSet, neighborAddress);
    if (idx >= 0) {
        rangingTableSet->tables[idx].distance = distance;
    }
    /* If not found, distance is not set (no error - table may not exist yet) */
}

/* ============================================================================
 * Ranging Table Tr/Rr Buffer Operations (UNCHANGED LOGIC)
 * ============================================================================ */

void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer)
{
    rangingTableBuffer->cur = 0;
    rangingTableBuffer->latest = 0;
    Timestamp_Tuple_t empty = {.seqNumber = 0, .timestamp = 0};
    for (set_index_t i = 0; i < Tr_Rr_BUFFER_POOL_SIZE; i++) {
        rangingTableBuffer->candidates[i].Tr = empty;
        rangingTableBuffer->candidates[i].Rr = empty;
    }
}

void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                              Timestamp_Tuple_t Tr,
                              Timestamp_Tuple_t Rr)
{
    rangingTableBuffer->candidates[rangingTableBuffer->cur].Tr = Tr;
    rangingTableBuffer->candidates[rangingTableBuffer->cur].Rr = Rr;
    /* Shift */
    rangingTableBuffer->latest = rangingTableBuffer->cur;
    rangingTableBuffer->cur = (rangingTableBuffer->cur + 1) % Tr_Rr_BUFFER_POOL_SIZE;
}

Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(
    Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
    Timestamp_Tuple_t Tf)
{
    set_index_t index = rangingTableBuffer->latest;
    uint64_t rightBound = Tf.timestamp % UWB_MAX_TIMESTAMP;
    Ranging_Table_Tr_Rr_Candidate_t candidate = {.Rr.timestamp = 0, .Tr.timestamp = 0};

    for (int count = 0; count < Tr_Rr_BUFFER_POOL_SIZE; count++) {
        if (rangingTableBuffer->candidates[index].Rr.timestamp &&
            rangingTableBuffer->candidates[index].Rr.timestamp % UWB_MAX_TIMESTAMP < rightBound) {
            candidate.Tr = rangingTableBuffer->candidates[index].Tr;
            candidate.Rr = rangingTableBuffer->candidates[index].Rr;
            break;
        }
        index = (index - 1 + Tr_Rr_BUFFER_POOL_SIZE) % Tr_Rr_BUFFER_POOL_SIZE;
    }

    return candidate;
}

/* ============================================================================
 * TfBuffer Operations (UNCHANGED LOGIC)
 * ============================================================================ */

void updateTfBuffer(Timestamp_Tuple_t timestamp)
{
    k_mutex_lock(&TfBufferMutex, K_FOREVER);
    TfBufferIndex++;
    TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
    TfBuffer[TfBufferIndex] = timestamp;
    k_mutex_unlock(&TfBufferMutex);
}

void clearTfBuffer(void)
{
    k_mutex_lock(&TfBufferMutex, K_FOREVER);
    memset(TfBuffer, 0, sizeof(TfBuffer));
    TfBufferIndex = 0;
    k_mutex_unlock(&TfBufferMutex);
}

Timestamp_Tuple_t findTfBySeqNumber(uint16_t seqNumber)
{
    k_mutex_lock(&TfBufferMutex, K_FOREVER);
    Timestamp_Tuple_t Tf = {.timestamp = 0, .seqNumber = 0};
    int startIndex = TfBufferIndex;

    LOG_DBG("findTfBySeqNumber: Searching for seq=%d, TfBufferIndex=%d", seqNumber, startIndex);

    /* Backward search */
    for (int i = startIndex; i >= 0; i--) {
        if (TfBuffer[i].seqNumber == seqNumber) {
            Tf = TfBuffer[i];
            LOG_DBG("findTfBySeqNumber: FOUND seq=%d at index %d", seqNumber, i);
            break;
        }
    }

    if (!Tf.timestamp) {
        /* Forward search */
        for (int i = startIndex + 1; i < Tf_BUFFER_POOL_SIZE; i++) {
            if (TfBuffer[i].seqNumber == seqNumber) {
                Tf = TfBuffer[i];
                LOG_DBG("findTfBySeqNumber: FOUND seq=%d at index %d", seqNumber, i);
                break;
            }
        }
    }

    if (!Tf.timestamp) {
        LOG_DBG("findTfBySeqNumber: NOT FOUND seq=%d. TfBuffer contents:", seqNumber);
        for (int i = 0; i < Tf_BUFFER_POOL_SIZE; i++) {
            if (TfBuffer[i].timestamp != 0) {
                LOG_DBG("  TfBuffer[%d]: seq=%d, ts=%llx", i, TfBuffer[i].seqNumber,
                        (uint64_t)TfBuffer[i].timestamp);
            }
        }
    }

    k_mutex_unlock(&TfBufferMutex);
    return Tf;
}

Timestamp_Tuple_t getLatestTxTimestamp(void)
{
    return TfBuffer[TfBufferIndex];
}

void getLatestNTxTimestamps(Timestamp_Tuple_t *timestamps, int n)
{
    ASSERT(n <= Tf_BUFFER_POOL_SIZE);
    k_mutex_lock(&TfBufferMutex, K_FOREVER);
    int startIndex = (TfBufferIndex + 1 - n + Tf_BUFFER_POOL_SIZE) % Tf_BUFFER_POOL_SIZE;
    for (int i = n - 1; i >= 0; i--) {
        timestamps[i] = TfBuffer[startIndex];
        startIndex = (startIndex + 1) % Tf_BUFFER_POOL_SIZE;
    }
    k_mutex_unlock(&TfBufferMutex);
}

/* ============================================================================
 * Ranging Table Set Operations (UNCHANGED LOGIC)
 * ============================================================================ */

Ranging_Table_Set_t *getGlobalRangingTableSet(void)
{
    return rangingTableSet;
}

void rangingTableInit(Ranging_Table_t *table, UWB_Address_t neighborAddress)
{
    memset(table, 0, sizeof(Ranging_Table_t));
    table->state = RANGING_STATE_S1;
    table->neighborAddress = neighborAddress;
    table->period = RANGING_PERIOD;
    table->nextExpectedDeliveryTime = 0;
    table->expirationTime = 0;
    table->lastSendTime = 0;
    table->distance = -1;
    rangingTableBufferInit(&table->TrRrBuffer);
}

void rangingTableSetDeinit(void)
{
    if (rangingTableSet) {
        block_free(rangingTableSet);
        rangingTableSet = NULL;
    }
}

void rangingTableSetInit(Ranging_Table_Set_t *set)
{
    /* If called with NULL, allocate from heap (first init) */
    if (set == NULL) {
        rangingTableSet = block_malloc(sizeof(Ranging_Table_Set_t));
        __ASSERT(rangingTableSet != NULL, "Failed to allocate rangingTableSet (%zu bytes)",
                 sizeof(Ranging_Table_Set_t));
        set = rangingTableSet;
    }
    k_mutex_init(&set->mu);
    set->size = 0;
    for (int i = 0; i < RANGING_TABLE_SIZE_MAX; i++) {
        set->tables[i] = EMPTY_RANGING_TABLE;
    }
    k_mutex_init(&TfBufferMutex);
}

/* Helper: Swap two tables */
static void rangingTableSetSwapTable(Ranging_Table_Set_t *set, int first, int second)
{
    Ranging_Table_t temp = set->tables[first];
    set->tables[first] = set->tables[second];
    set->tables[second] = temp;
}

/* Helper: Binary search for table by address */
static int rangingTableSetSearchTable(Ranging_Table_Set_t *set, UWB_Address_t targetAddress)
{
    /* Binary Search */
    int left = -1, right = set->size, res = -1;
    while (left + 1 != right) {
        int mid = left + (right - left) / 2;
        if (set->tables[mid].neighborAddress == targetAddress) {
            res = mid;
            break;
        } else if (set->tables[mid].neighborAddress > targetAddress) {
            right = mid;
        } else {
            left = mid;
        }
    }
    return res;
}

/* Comparison functions for sorting */
typedef int (*rangingTableCompareFunc)(Ranging_Table_t *, Ranging_Table_t *);

static int COMPARE_BY_ADDRESS(Ranging_Table_t *first, Ranging_Table_t *second)
{
    if (first->neighborAddress == second->neighborAddress) {
        return 0;
    }
    if (first->neighborAddress > second->neighborAddress) {
        return 1;
    }
    return -1;
}

static int __attribute__((unused)) COMPARE_BY_EXPIRATION_TIME(Ranging_Table_t *first, Ranging_Table_t *second)
{
    if (first->expirationTime == second->expirationTime) {
        return 0;
    }
    if (first->expirationTime > second->expirationTime) {
        return -1;
    }
    return 1;
}

static int COMPARE_BY_NEXT_EXPECTED_DELIVERY_TIME(Ranging_Table_t *first, Ranging_Table_t *second)
{
    if (first->nextExpectedDeliveryTime == second->nextExpectedDeliveryTime) {
        return 0;
    }
    if (first->nextExpectedDeliveryTime > second->nextExpectedDeliveryTime) {
        return 1;
    }
    return -1;
}

static int COMPARE_BY_LAST_SEND_TIME(Ranging_Table_t *first, Ranging_Table_t *second)
{
    if (first->lastSendTime == second->lastSendTime) {
        return 0;
    }
    if (first->lastSendTime > second->lastSendTime) {
        return 1;
    }
    return -1;
}

/* Build the heap (heapify) */
static void rangingTableSetArrange(Ranging_Table_Set_t *set, int index, int len,
                                   rangingTableCompareFunc compare)
{
    int leftChild = 2 * index + 1;
    int rightChild = 2 * index + 2;
    int maxIndex = index;

    if (leftChild < len && compare(&set->tables[maxIndex], &set->tables[leftChild]) < 0) {
        maxIndex = leftChild;
    }
    if (rightChild < len && compare(&set->tables[maxIndex], &set->tables[rightChild]) < 0) {
        maxIndex = rightChild;
    }
    if (maxIndex != index) {
        rangingTableSetSwapTable(set, index, maxIndex);
        rangingTableSetArrange(set, maxIndex, len, compare);
    }
}

/* Sort the ranging table using heap sort */
static void rangingTableSetRearrange(Ranging_Table_Set_t *set, rangingTableCompareFunc compare)
{
    /* Build max heap */
    for (int i = set->size / 2 - 1; i >= 0; i--) {
        rangingTableSetArrange(set, i, set->size, compare);
    }
    for (int i = set->size - 1; i >= 0; i--) {
        rangingTableSetSwapTable(set, 0, i);
        rangingTableSetArrange(set, 0, i, compare);
    }
}

/* Clear expired tables */
static int rangingTableSetClearExpire(Ranging_Table_Set_t *set)
{
    Time_t curTime = xTaskGetTickCount();
    int evictionCount = 0;

    for (int i = 0; i < set->size; i++) {
        if (set->tables[i].expirationTime <= curTime) {
            LOG_DBG("Clear ranging table for neighbor %u (expired at %u)",
                    set->tables[i].neighborAddress,
                    set->tables[i].expirationTime);
            setDistance(set->tables[i].neighborAddress, -1);
            set->tables[i] = EMPTY_RANGING_TABLE;
            evictionCount++;
        }
    }

    /* Keep ranging table set in order */
    rangingTableSetRearrange(set, COMPARE_BY_ADDRESS);
    set->size -= evictionCount;

    return evictionCount;
}

bool rangingTableSetAddTable(Ranging_Table_Set_t *set, Ranging_Table_t table)
{
    int index = rangingTableSetSearchTable(set, table.neighborAddress);
    if (index != -1) {
        LOG_DBG("Table for neighbor %u already exists, updating instead",
                table.neighborAddress);
        set->tables[index] = table;
        return true;
    }

    /* If ranging table is full and there is no expired table, ignore */
    if (set->size == RANGING_TABLE_SIZE_MAX && rangingTableSetClearExpire(set) == 0) {
        LOG_WRN("Ranging table full, ignoring new neighbor %u", table.neighborAddress);
        return false;
    }

    /* Add the new entry to the last */
    uint8_t curIndex = set->size;
    set->tables[curIndex] = table;
    set->size++;

    /* Sort the ranging table, keep it in order */
    rangingTableSetRearrange(set, COMPARE_BY_ADDRESS);
    LOG_DBG("Added new neighbor %u to ranging table", table.neighborAddress);
    return true;
}

void rangingTableSetUpdateTable(Ranging_Table_Set_t *set, Ranging_Table_t table)
{
    int index = rangingTableSetSearchTable(set, table.neighborAddress);
    if (index == -1) {
        LOG_DBG("Cannot find table for neighbor %u, adding instead", table.neighborAddress);
        rangingTableSetAddTable(set, table);
    } else {
        set->tables[index] = table;
        LOG_DBG("Updated table for neighbor %u", table.neighborAddress);
    }
}

void rangingTableSetRemoveTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress)
{
    if (set->size == 0) {
        LOG_DBG("Ranging table is empty, ignore remove");
        return;
    }

    int index = rangingTableSetSearchTable(set, neighborAddress);
    if (index == -1) {
        LOG_DBG("Cannot find table for neighbor %u, ignore remove", neighborAddress);
        return;
    }

    rangingTableSetSwapTable(set, index, set->size - 1);
    set->tables[set->size - 1] = EMPTY_RANGING_TABLE;
    set->size--;
    rangingTableSetRearrange(set, COMPARE_BY_ADDRESS);
}

Ranging_Table_t rangingTableSetFindTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress)
{
    int index = rangingTableSetSearchTable(set, neighborAddress);
    Ranging_Table_t table = EMPTY_RANGING_TABLE;
    if (index == -1) {
        LOG_DBG("Cannot find table for neighbor %u", neighborAddress);
    } else {
        table = set->tables[index];
    }
    return table;
}

/* ============================================================================
 * Debug Functions
 * ============================================================================ */

void printRangingTable(Ranging_Table_t *table)
{
    LOG_INF("Rp=%u, Tr=%u, Rf=%u",
            table->Rp.seqNumber,
            table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.seqNumber,
            table->Rf.seqNumber);
    LOG_INF("Tp=%u, Rr=%u, Tf=%u, Re=%u",
            table->Tp.seqNumber,
            table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.seqNumber,
            table->Tf.seqNumber,
            table->Re.seqNumber);
}

void printRangingTableSet(Ranging_Table_Set_t *set)
{
    LOG_INF("neighbor\tdistance\tperiod\texpire");
    for (int i = 0; i < set->size; i++) {
        if (set->tables[i].neighborAddress == UWB_DEST_EMPTY) {
            continue;
        }
        LOG_INF("%u\t%d\t%u\t%u",
                set->tables[i].neighborAddress,
                set->tables[i].distance,
                set->tables[i].period,
                set->tables[i].expirationTime);
    }
    LOG_INF("---");
}

void printRangingMessage(Ranging_Message_t *rangingMessage)
{
    for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++) {
        LOG_DBG("lastTxTimestamp[%d] seq=%u, ts=%llx",
                i,
                rangingMessage->header.lastTxTimestamps[i].seqNumber,
                (unsigned long long)rangingMessage->header.lastTxTimestamps[i].timestamp);
    }

    if (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t) == 0) {
        return;
    }

    uint16_t body_unit_number = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) /
                                sizeof(Body_Unit_t);
    if (body_unit_number >= RANGING_MAX_BODY_UNIT) {
        LOG_WRN("Malformed body unit number");
        return;
    }

    for (int i = 0; i < body_unit_number; i++) {
        LOG_DBG("unitAddress=%u, Seq=%u",
                rangingMessage->bodyUnits[i].address,
                rangingMessage->bodyUnits[i].timestamp.seqNumber);
    }
}

/* ============================================================================
 * Public API for sorting (used by message generation)
 * ============================================================================ */

void rangingTableSetSortByAddress(Ranging_Table_Set_t *set)
{
    rangingTableSetRearrange(set, COMPARE_BY_ADDRESS);
}

void rangingTableSetSortByNextDelivery(Ranging_Table_Set_t *set)
{
    rangingTableSetRearrange(set, COMPARE_BY_NEXT_EXPECTED_DELIVERY_TIME);
}

void rangingTableSetSortByLastSend(Ranging_Table_Set_t *set)
{
    rangingTableSetRearrange(set, COMPARE_BY_LAST_SEND_TIME);
}

int rangingTableSetEvictExpired(Ranging_Table_Set_t *set)
{
    return rangingTableSetClearExpire(set);
}
