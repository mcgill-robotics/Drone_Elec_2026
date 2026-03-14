#ifndef UAVCAN_NODE_H
#define UAVCAN_NODE_H

#include <stdint.h>

void uavcan_init(void);
void publish_node_status(void);
void publish_airspeed_data(void);

#endif