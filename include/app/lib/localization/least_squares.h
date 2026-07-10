#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <stdlib.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */
#include <app/lib/ranging/twr.h>
#include "location.h"

/** Least squares solver flag: constrain Z coordinate to be non-negative */
#define LS_FLAG_CONSTRAIN_Z_POSITIVE  0x01

/**
 * @brief Iterative least squares localization using Gauss-Newton method
 *
 * @param target_address Address of the node to localize
 * @param known_positions Array of anchor positions
 * @param known_positions_length Number of anchors
 * @param measurements Array of distance measurements
 * @param measurement_count Number of measurements
 * @param estimate Output position estimate
 * @param flags Solver flags (LS_FLAG_CONSTRAIN_Z_POSITIVE, etc.)
 * @return 0 on success, negative errno on failure
 */
int cholesky_linear_localization(deca_short_addr_t target_address,
    const struct node_position *known_positions, size_t known_positions_length,
    const struct measurement *measurements, size_t measurement_count,
    struct node_position *estimate, uint8_t flags);

#endif
