/**
 * @file node_config_service.h
 * @brief GATT service for configuring node parameters (position, ID, mode)
 */

#ifndef NODE_CONFIG_SERVICE_H
#define NODE_CONFIG_SERVICE_H

#include <zephyr/types.h>

/**
 * @brief Initialize and register the node configuration GATT service
 * 
 * This service provides characteristics for reading and writing:
 * - Node position (x, y, z in meters as doubles)
 * - Node ID (16-bit address)
 * - Node mode (operating mode enumeration)
 * 
 * @return 0 on success, negative errno on failure
 */
int node_config_service_init(void);

#endif /* NODE_CONFIG_SERVICE_H */
