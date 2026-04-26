#pragma once

#include "stm32g4xx_hal.h"

// Private file include
#include <can_node.h>
#include <dronecan_msgs.h>


int16_t send_battery_info(uint16_t voltage, uint16_t current);