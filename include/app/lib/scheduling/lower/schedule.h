#ifndef SYNCHROFLY_SCHEDULE_H
#define SYNCHROFLY_SCHEDULE_H

#include <app/drivers/ieee802154/uwb_driver_api.h>

/**
 * @brief Allocates memory for a new schedule structure with the specified number of slots
 *
 * This function allocates memory for a schedule structure and its associated slot array.
 * Typically, you should use specific schedule creation functions rather than calling this directly.
 *
 * @param slot_count Number of slots to allocate in the schedule (must be positive)
 * @param schedule Pointer to store the allocated schedule structure
 *
 * @retval 0 Success
 * @retval -EINVAL Invalid parameters (slot_count <= 0 or schedule is NULL)
 * @retval -ENOMEM Memory allocation failed
 */
int deca_schedule_alloc(int slot_count, struct deca_schedule **schedule);

/**
 * @brief Frees memory allocated for a schedule structure
 *
 * This function deallocates the memory used by a schedule structure and its slot array.
 * It should be called when the schedule is no longer needed to prevent memory leaks.
 *
 * @param schedule Pointer to the schedule structure to be freed
 *
 * @retval 0 Success
 * @retval -EINVAL Invalid parameter (schedule is NULL)
 */
int deca_schedule_free(struct deca_schedule *schedule);

#endif /* SYNCHROFLY_SCHEDULE_H */
