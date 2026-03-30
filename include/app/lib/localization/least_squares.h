#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <stdlib.h>
/* Hardware-specific headers no longer needed - using generic UWB driver API */
#include <app/lib/ranging/twr.h>
#include "location.h"

int cholesky_linear_localization(deca_short_addr_t target_address,
    const struct node_position *known_positions, size_t known_positions_length,
    const struct measurement *measurements, size_t measurement_count,
    struct node_position *estimate);

#endif
