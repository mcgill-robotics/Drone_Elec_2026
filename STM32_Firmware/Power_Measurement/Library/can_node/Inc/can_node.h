#ifndef CAN_NODE_H
#define CAN_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"


void can_node_init(FDCAN_HandleTypeDef *hfdcan);
void can_node_update(void);

#endif