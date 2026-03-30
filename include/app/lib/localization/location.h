#ifndef LOCATION_H
#define LOCATION_H

#include <app/drivers/ieee802154/uwb_driver_api.h>

struct vec3d_f {
    float x, y, z;
};

struct node_position {
    deca_short_addr_t addr;
    struct vec3d_f position;
};

#endif
