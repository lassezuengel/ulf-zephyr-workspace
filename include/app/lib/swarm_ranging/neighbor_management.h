/**
 * @file neighbor_management.h
 * @brief Neighbor set management and topology sensing API
 */

#ifndef NEIGHBOR_MANAGEMENT_H
#define NEIGHBOR_MANAGEMENT_H

#include <app/lib/swarm_ranging/swarm_ranging_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Neighbor Bit Set Operations
 * ============================================================================ */

void neighborBitSetInit(Neighbor_Bit_Set_t *bitSet);
void neighborBitSetAdd(Neighbor_Bit_Set_t *bitSet, UWB_Address_t neighborAddress);
void neighborBitSetRemove(Neighbor_Bit_Set_t *bitSet, UWB_Address_t neighborAddress);
void neighborBitSetClear(Neighbor_Bit_Set_t *bitSet);
bool neighborBitSetHas(Neighbor_Bit_Set_t *bitSet, UWB_Address_t neighborAddress);

/* ============================================================================
 * Neighbor Set Operations
 * ============================================================================ */

Neighbor_Set_t *getGlobalNeighborSet(void);
void neighborSetInit(Neighbor_Set_t *set);
void neighborSetDeinit(void);
bool neighborSetHas(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
bool neighborSetHasOneHop(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
bool neighborSetHasTwoHop(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
void neighborSetAddOneHopNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
void neighborSetAddTwoHopNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
void neighborSetRemoveNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
bool neighborSetHasRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to);
void neighborSetAddRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to);
void neighborSetRemoveRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to);
void neighborSetRegisterNewNeighborHook(Neighbor_Set_t *set, neighborSetHook hook);
void neighborSetRegisterExpirationHook(Neighbor_Set_t *set, neighborSetHook hook);
void neighborSetRegisterTopologyChangeHook(Neighbor_Set_t *set, neighborSetHook hook);
void neighborSetHooksInvoke(Neighbor_Set_Hooks_t *hooks, UWB_Address_t neighborAddress);
void neighborSetUpdateExpirationTime(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
int neighborSetClearExpire(Neighbor_Set_t *set);

/* ============================================================================
 * Topology Sensing
 * ============================================================================ */

/**
 * Process ranging message for topology discovery
 *
 * @param rangingMessage Received ranging message
 * @param my_address Local UWB address
 */
void topologySensing(Ranging_Message_t *rangingMessage, uint16_t my_address);

/* ============================================================================
 * Debug Functions
 * ============================================================================ */

void printNeighborBitSet(Neighbor_Bit_Set_t *bitSet, uint16_t my_address);
void printNeighborSet(Neighbor_Set_t *set, uint16_t my_address);

#ifdef __cplusplus
}
#endif

#endif /* NEIGHBOR_MANAGEMENT_H */
