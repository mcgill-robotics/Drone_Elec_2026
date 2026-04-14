#pragma once

#include "stm32g4xx_hal.h"

// Private file include
#include <can_node.h>
#include <dronecan_msgs.h>

int16_t send_pitot_info(uint16_t differential_pressure, uint16_t static_pressure, uint16_t temperature);