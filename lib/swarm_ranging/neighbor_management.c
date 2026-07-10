/**
 * @file neighbor_management.c
 * @brief Neighbor set management and topology sensing
 *
 * Port of SEU-NetSI neighbor management to Zephyr.
 * Logic preserved exactly, only RTOS primitives adapted.
 */

#include <app/lib/swarm_ranging/swarm_ranging_types.h>
#include <app/lib/swarm_ranging/platform_zephyr.h>
#include <app/lib/system/block_heap.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>

LOG_MODULE_REGISTER(neighbor_mgmt, LOG_LEVEL_NONE);

/* Global neighbor set -- heap-allocated to reduce static .bss usage */
static Neighbor_Set_t *neighborSet;

/* Forward declarations */
static void neighborBitSetInit(Neighbor_Bit_Set_t *bitSet);
static void neighborSetRemoveRelation(Neighbor_Set_t *set, UWB_Address_t neighborAddress,
                                      UWB_Address_t twoHopNeighbor);
static void neighborSetHooksInvoke(Neighbor_Set_Hooks_t *hooks, UWB_Address_t neighborAddress);
static void neighborSetUpdateExpirationTime(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
static void neighborSetRemoveNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
static bool neighborSetHasRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to);

/* ============================================================================
 * Neighbor Table Search/Lookup Helpers
 * ============================================================================ */

/**
 * Find index of neighbor by address
 * @return index [0..count-1] if found, -1 if not found
 */
static int neighborSetFindIndex(Neighbor_Set_t *set, UWB_Address_t address)
{
    for (int i = 0; i < set->count; i++) {
        if (set->neighbors[i].active && set->neighbors[i].address == address) {
            return i;
        }
    }
    return -1;
}

/**
 * Find existing neighbor or add new entry
 * @return index if found/added, -1 if table full
 */
static int neighborSetAddOrGetIndex(Neighbor_Set_t *set, UWB_Address_t address)
{
    /* Try to find existing */
    int idx = neighborSetFindIndex(set, address);
    if (idx >= 0) {
        LOG_DBG("Found existing neighbor 0x%04x at index %d", address, idx);
        return idx;
    }

    /* Not found - add new entry */
    if (set->count >= (NEIGHBOR_ADDRESS_MAX + 1)) {
        LOG_ERR("Neighbor table full (%d entries), cannot add 0x%04x", set->count, address);
        return -1;
    }

    /* Initialize new entry */
    idx = set->count;
    set->neighbors[idx].address = address;
    set->neighbors[idx].active = true;
    set->neighbors[idx].expirationTime = 0;
    neighborBitSetInit(&set->neighbors[idx].twoHopReachSet);
    set->count++;

    LOG_INF("Added NEW neighbor 0x%04x at index %d (total=%d)", address, idx, set->count);
    return idx;
}

/**
 * Get neighbor entry by index (with bounds checking)
 * @return pointer to entry or NULL if invalid
 */
/* neighborSetGetEntry removed -- was unused */

/* ============================================================================
 * Neighbor Bit Set Operations (REFACTORED: Use indices not addresses)
 * ============================================================================ */

void neighborBitSetInit(Neighbor_Bit_Set_t *bitSet)
{
    bitSet->bits = 0;
    bitSet->size = 0;
}

/**
 * Add neighbor index to bit set
 * @param index Table index (0-31), NOT address
 */
void neighborBitSetAdd(Neighbor_Bit_Set_t *bitSet, int index)
{
    ASSERT(index >= 0 && index < 64);  /* Bit set limited to 64 bits */
    uint64_t prevBits = bitSet->bits;
    bitSet->bits |= (1ULL << index);
    if (prevBits != bitSet->bits) {
        bitSet->size++;
    }
}

/**
 * Remove neighbor index from bit set
 * @param index Table index (0-31), NOT address
 */
void neighborBitSetRemove(Neighbor_Bit_Set_t *bitSet, int index)
{
    ASSERT(index >= 0 && index < 64);  /* Bit set limited to 64 bits */
    uint64_t prevBits = bitSet->bits;
    bitSet->bits &= ~(1ULL << index);
    if (prevBits != bitSet->bits) {
        bitSet->size--;
    }
}

void neighborBitSetClear(Neighbor_Bit_Set_t *bitSet)
{
    bitSet->bits = 0;
    bitSet->size = 0;
}

/**
 * Check if neighbor index is in bit set
 * @param index Table index (0-31), NOT address
 */
bool neighborBitSetHas(Neighbor_Bit_Set_t *bitSet, int index)
{
    ASSERT(index >= 0 && index < 64);  /* Bit set limited to 64 bits */
    return (bitSet->bits & (1ULL << index)) != 0;
}

/* ============================================================================
 * Neighbor Set Operations
 * ============================================================================ */

Neighbor_Set_t *getGlobalNeighborSet(void)
{
    return neighborSet;
}

void neighborSetDeinit(void)
{
    if (neighborSet) {
        block_free(neighborSet);
        neighborSet = NULL;
    }
}

void neighborSetInit(Neighbor_Set_t *set)
{
    /* If called with NULL, allocate from heap (first init) */
    if (set == NULL) {
        neighborSet = block_malloc(sizeof(Neighbor_Set_t));
        __ASSERT(neighborSet != NULL, "Failed to allocate neighborSet (%zu bytes)",
                 sizeof(Neighbor_Set_t));
        set = neighborSet;
    }
    set->count = 0;
    k_mutex_init(&set->mu);
    neighborBitSetInit(&set->oneHopIndices);
    neighborBitSetInit(&set->twoHopIndices);

    /* Initialize hooks (simplified - no linked list for now) */
    set->neighborNewHooks.hook = NULL;
    set->neighborNewHooks.next = NULL;
    set->neighborExpirationHooks.hook = NULL;
    set->neighborExpirationHooks.next = NULL;
    set->neighborTopologyChangeHooks.hook = NULL;
    set->neighborTopologyChangeHooks.next = NULL;

    /* Initialize all neighbor entries */
    for (int i = 0; i <= NEIGHBOR_ADDRESS_MAX; i++) {
        set->neighbors[i].address = 0;
        set->neighbors[i].active = false;
        set->neighbors[i].expirationTime = 0;
        neighborBitSetInit(&set->neighbors[i].twoHopReachSet);
    }
}

bool neighborSetHas(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
    int idx = neighborSetFindIndex(set, neighborAddress);
    if (idx < 0) return false;

    return neighborBitSetHas(&set->oneHopIndices, idx) ||
           neighborBitSetHas(&set->twoHopIndices, idx);
}

bool neighborSetHasOneHop(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
    int idx = neighborSetFindIndex(set, neighborAddress);
    if (idx < 0) return false;

    return neighborBitSetHas(&set->oneHopIndices, idx);
}

bool neighborSetHasTwoHop(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
    int idx = neighborSetFindIndex(set, neighborAddress);
    if (idx < 0) return false;

    return neighborBitSetHas(&set->twoHopIndices, idx);
}

void neighborSetAddOneHopNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
    bool isNewNeighbor = false;

    if (!neighborSetHas(set, neighborAddress)) {
        isNewNeighbor = true;
    }

    /* If neighbor is previous two-hop neighbor, remove it from two-hop neighbor set */
    if (neighborSetHasTwoHop(set, neighborAddress)) {
        LOG_DBG("0x%04x was two-hop, promoting to one-hop", neighborAddress);
        neighborSetRemoveNeighbor(set, neighborAddress);
    }

    /* Get or add entry */
    int idx = neighborSetAddOrGetIndex(set, neighborAddress);
    if (idx < 0) {
        LOG_ERR("Cannot add one-hop neighbor 0x%04x: table full", neighborAddress);
        return;
    }

    /* Add one-hop neighbor */
    if (!neighborBitSetHas(&set->oneHopIndices, idx)) {
        LOG_INF("Adding 0x%04x to one-hop bit set at idx=%d", neighborAddress, idx);
        neighborBitSetAdd(&set->oneHopIndices, idx);
        neighborSetUpdateExpirationTime(set, neighborAddress);
        neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, neighborAddress);
    }

    if (isNewNeighbor) {
        LOG_INF("New neighbor detected: 0x%04x", neighborAddress);
        neighborSetHooksInvoke(&set->neighborNewHooks, neighborAddress);
    }
}

void neighborSetAddTwoHopNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
    bool isNewNeighbor = false;

    if (!neighborSetHas(set, neighborAddress)) {
        isNewNeighbor = true;
    }

    /* If neighbor is previous one-hop neighbor, remove it from one-hop neighbor set */
    if (neighborSetHasOneHop(set, neighborAddress)) {
        neighborSetRemoveNeighbor(set, neighborAddress);
    }

    /* Get or add entry */
    int idx = neighborSetAddOrGetIndex(set, neighborAddress);
    if (idx < 0) {
        LOG_ERR("Cannot add two-hop neighbor 0x%04x: table full", neighborAddress);
        return;
    }

    if (!neighborBitSetHas(&set->twoHopIndices, idx)) {
        /* Add two-hop neighbor */
        neighborBitSetAdd(&set->twoHopIndices, idx);
        neighborSetUpdateExpirationTime(set, neighborAddress);
        neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, neighborAddress);
    }

    if (isNewNeighbor) {
        neighborSetHooksInvoke(&set->neighborNewHooks, neighborAddress);
    }
}

void neighborSetRemoveNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
    int idx = neighborSetFindIndex(set, neighborAddress);
    if (idx < 0) return;  /* Not found */

    if (neighborSetHasOneHop(set, neighborAddress) && neighborSetHasTwoHop(set, neighborAddress)) {
        ASSERT(0); // impossible
    }

    /* Clear expiration time */
    set->neighbors[idx].expirationTime = 0;

    if (neighborSetHasOneHop(set, neighborAddress)) {
        neighborBitSetRemove(&set->oneHopIndices, idx);
        /* Remove related paths to two-hop neighbors */
        for (int i = 0; i < set->count; i++) {
            if (!set->neighbors[i].active) continue;
            UWB_Address_t twoHopAddr = set->neighbors[i].address;
            if (neighborSetHasRelation(set, neighborAddress, twoHopAddr)) {
                neighborSetRemoveRelation(set, neighborAddress, twoHopAddr);
            }
        }
    } else if (neighborSetHasTwoHop(set, neighborAddress)) {
        neighborBitSetRemove(&set->twoHopIndices, idx);
        /* Clear related two-hop reach set */
        neighborBitSetClear(&set->neighbors[idx].twoHopReachSet);
    } else {
        ASSERT(0); // impossible
    }

    neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, neighborAddress);
}

bool neighborSetHasRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to)
{
    int fromIdx = neighborSetFindIndex(set, from);
    int toIdx = neighborSetFindIndex(set, to);

    if (toIdx < 0 || fromIdx < 0) return false;

    return neighborBitSetHas(&set->neighbors[toIdx].twoHopReachSet, fromIdx);
}

void neighborSetAddRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to)
{
    int fromIdx = neighborSetAddOrGetIndex(set, from);
    int toIdx = neighborSetAddOrGetIndex(set, to);

    if (fromIdx < 0 || toIdx < 0) {
        LOG_ERR("Cannot add relation 0x%04x -> 0x%04x: table full", from, to);
        return;
    }

    if (!neighborBitSetHas(&set->neighbors[toIdx].twoHopReachSet, fromIdx)) {
        neighborBitSetAdd(&set->neighbors[toIdx].twoHopReachSet, fromIdx);
        neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, from);
    }
}

void neighborSetRemoveRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to)
{
    int fromIdx = neighborSetFindIndex(set, from);
    int toIdx = neighborSetFindIndex(set, to);

    if (fromIdx < 0 || toIdx < 0) return;

    if (neighborBitSetHas(&set->neighbors[toIdx].twoHopReachSet, fromIdx)) {
        neighborBitSetRemove(&set->neighbors[toIdx].twoHopReachSet, fromIdx);
        neighborSetHooksInvoke(&set->neighborTopologyChangeHooks, from);
    }
}

void neighborSetRegisterNewNeighborHook(Neighbor_Set_t *set, neighborSetHook hook)
{
    ASSERT(hook);
    /* Simplified: only support single hook */
    set->neighborNewHooks.hook = hook;
}

void neighborSetRegisterExpirationHook(Neighbor_Set_t *set, neighborSetHook hook)
{
    ASSERT(hook);
    set->neighborExpirationHooks.hook = hook;
}

void neighborSetRegisterTopologyChangeHook(Neighbor_Set_t *set, neighborSetHook hook)
{
    ASSERT(hook);
    set->neighborTopologyChangeHooks.hook = hook;
}

void neighborSetHooksInvoke(Neighbor_Set_Hooks_t *hooks, UWB_Address_t neighborAddress)
{
    if (hooks->hook != NULL) {
        LOG_DBG("Invoking neighbor set hook for neighbor %u", neighborAddress);
        hooks->hook(neighborAddress);
    }
}

void neighborSetUpdateExpirationTime(Neighbor_Set_t *set, UWB_Address_t neighborAddress)
{
    int idx = neighborSetAddOrGetIndex(set, neighborAddress);
    if (idx < 0) {
        LOG_ERR("Cannot update expiration for 0x%04x: table full", neighborAddress);
        return;
    }

    Time_t newExpiration = xTaskGetTickCount() + M2T(NEIGHBOR_SET_HOLD_TIME);
    set->neighbors[idx].expirationTime = newExpiration;
    LOG_DBG("Updated expiration for 0x%04x (idx=%d) to %u", neighborAddress, idx, newExpiration);
}

int neighborSetClearExpire(Neighbor_Set_t *set)
{
    Time_t curTime = xTaskGetTickCount();
    int evictionCount = 0;

    /* Iterate through active neighbors */
    for (int i = 0; i < set->count; i++) {
        if (!set->neighbors[i].active) continue;

        UWB_Address_t addr = set->neighbors[i].address;
        if (neighborSetHas(set, addr) && set->neighbors[i].expirationTime <= curTime) {
            evictionCount++;
            neighborSetRemoveNeighbor(set, addr);
            LOG_DBG("Neighbor 0x%04x expired at %u", addr, curTime);
            neighborSetHooksInvoke(&set->neighborExpirationHooks, addr);
        }
    }

    return evictionCount;
}

/* ============================================================================
 * Topology Sensing (UNCHANGED LOGIC)
 * ============================================================================ */

void topologySensing(Ranging_Message_t *rangingMessage, uint16_t my_address)
{
    UWB_Address_t neighborAddress = rangingMessage->header.srcAddress;

    LOG_INF("TopologySensing: RX from 0x%04x (my=0x%04x)", neighborAddress, my_address);

    if (!neighborSetHasOneHop(neighborSet, neighborAddress)) {
        /* Add current neighbor to one-hop neighbor set */
        LOG_INF("Adding 0x%04x as one-hop neighbor", neighborAddress);
        neighborSetAddOneHopNeighbor(neighborSet, neighborAddress);
    }
    neighborSetUpdateExpirationTime(neighborSet, neighborAddress);

    /* Infer one-hop and two-hop neighbors from received ranging message */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) /
                            sizeof(Body_Unit_t);

    LOG_DBG("Processing %u body units from 0x%04x", bodyUnitCount, neighborAddress);

    for (int i = 0; i < bodyUnitCount; i++) {
        /* Note: OLSR MPR logic disabled for now */
        UWB_Address_t twoHopNeighbor = rangingMessage->bodyUnits[i].address;

        if (twoHopNeighbor != my_address && !neighborSetHasOneHop(neighborSet, twoHopNeighbor)) {
            /* If it is not one-hop neighbor then it is now my two-hop neighbor */
            if (!neighborSetHasTwoHop(neighborSet, twoHopNeighbor)) {
                LOG_DBG("Adding 0x%04x as two-hop neighbor (via 0x%04x)", twoHopNeighbor, neighborAddress);
                neighborSetAddTwoHopNeighbor(neighborSet, twoHopNeighbor);
            }

            if (!neighborSetHasRelation(neighborSet, neighborAddress, twoHopNeighbor)) {
                neighborSetAddRelation(neighborSet, neighborAddress, twoHopNeighbor);
            }

            neighborSetUpdateExpirationTime(neighborSet, twoHopNeighbor);
        }
    }
}

/* ============================================================================
 * Debug Functions
 * ============================================================================ */

void printNeighborBitSet(Neighbor_Bit_Set_t *bitSet, uint16_t my_address)
{
    LOG_INF("0x%04x has %u neighbors = ", my_address, bitSet->size);
    /* Print indices set in bitSet */
    for (int idx = 0; idx < 64; idx++) {
        if (neighborBitSetHas(bitSet, idx)) {
            printk("%d ", idx);
        }
    }
    printk("\n");
}

void printNeighborSet(Neighbor_Set_t *set, uint16_t my_address)
{
    LOG_INF("0x%04x has %u one-hop, %u two-hop neighbors",
            my_address, set->oneHopIndices.size, set->twoHopIndices.size);

    printk("  one-hop neighbors = ");
    for (int i = 0; i < set->count; i++) {
        if (set->neighbors[i].active && neighborBitSetHas(&set->oneHopIndices, i)) {
            printk("0x%04x ", set->neighbors[i].address);
        }
    }
    printk("\n");

    printk("  two-hop neighbors = ");
    for (int i = 0; i < set->count; i++) {
        if (set->neighbors[i].active && neighborBitSetHas(&set->twoHopIndices, i)) {
            printk("0x%04x ", set->neighbors[i].address);
        }
    }
    printk("\n");

    /* Print two-hop reach sets */
    for (int i = 0; i < set->count; i++) {
        if (!set->neighbors[i].active) continue;
        if (!neighborBitSetHas(&set->twoHopIndices, i)) continue;

        UWB_Address_t twoHopAddr = set->neighbors[i].address;
        printk("  to two-hop neighbor 0x%04x via: ", twoHopAddr);

        /* Print all one-hop neighbors that can reach this two-hop neighbor */
        for (int j = 0; j < set->count; j++) {
            if (set->neighbors[j].active && neighborBitSetHas(&set->neighbors[i].twoHopReachSet, j)) {
                printk("0x%04x ", set->neighbors[j].address);
            }
        }
        printk("\n");
    }
}
