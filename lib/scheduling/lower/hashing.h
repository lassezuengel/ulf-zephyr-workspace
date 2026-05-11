#ifndef HASHING_H
#define HASHING_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include <app/drivers/ieee802154/dw1000.h>

void seeded_hash_permute_until_index(deca_short_addr_t permutation[], size_t overall_size, size_t permutation_size, uint64_t seed);

#endif
