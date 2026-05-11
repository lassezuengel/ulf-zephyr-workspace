/**
 * @file ranging_table.h
 * @brief Ranging table management API
 */

#ifndef RANGING_TABLE_H
#define RANGING_TABLE_H

#include <app/lib/swarm_ranging/swarm_ranging_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Distance Tracking
 * ============================================================================ */

int16_t getDistance(UWB_Address_t neighborAddress);
void setDistance(UWB_Address_t neighborAddress, int16_t distance);

/* ============================================================================
 * Tr_Rr Buffer Operations
 * ============================================================================ */

void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);
void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                              Timestamp_Tuple_t Tr,
                              Timestamp_Tuple_t Rr);
Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(
    Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
    Timestamp_Tuple_t Tf);

/* ============================================================================
 * Tf Buffer Operations
 * ============================================================================ */

void updateTfBuffer(Timestamp_Tuple_t timestamp);
Timestamp_Tuple_t findTfBySeqNumber(uint16_t seqNumber);
Timestamp_Tuple_t getLatestTxTimestamp(void);
void getLatestNTxTimestamps(Timestamp_Tuple_t *timestamps, int n);

/* ============================================================================
 * Ranging Table Operations
 * ============================================================================ */

Ranging_Table_Set_t *getGlobalRangingTableSet(void);
void rangingTableInit(Ranging_Table_t *table, UWB_Address_t neighborAddress);
void rangingTableSetInit(Ranging_Table_Set_t *set);
void rangingTableSetDeinit(void);
bool rangingTableSetAddTable(Ranging_Table_Set_t *set, Ranging_Table_t table);
void rangingTableSetUpdateTable(Ranging_Table_Set_t *set, Ranging_Table_t table);
void rangingTableSetRemoveTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress);
Ranging_Table_t rangingTableSetFindTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress);

/* ============================================================================
 * Sorting and Eviction (used by message generation)
 * ============================================================================ */

void rangingTableSetSortByAddress(Ranging_Table_Set_t *set);
void rangingTableSetSortByNextDelivery(Ranging_Table_Set_t *set);
void rangingTableSetSortByLastSend(Ranging_Table_Set_t *set);
int rangingTableSetEvictExpired(Ranging_Table_Set_t *set);

/* ============================================================================
 * Debug Functions
 * ============================================================================ */

void printRangingTable(Ranging_Table_t *table);
void printRangingTableSet(Ranging_Table_Set_t *set);
void printRangingMessage(Ranging_Message_t *rangingMessage);

#ifdef __cplusplus
}
#endif

#endif /* RANGING_TABLE_H */
