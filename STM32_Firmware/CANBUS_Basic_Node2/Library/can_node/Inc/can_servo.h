#pragma once

#include "stm32g4xx_hal.h"

// Private file include
#include <can_node.h>
#include <dronecan_msgs.h>


// This device controls SERVO channels [SERVO_CHANNEL_OFFSET] and [SERVO_CHANNEL_OFFSET + 1].
// Set a different value per device so they don't overlap:
// EX: if servo_count = 2
//   Device 0: SERVO_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: SERVO_CHANNEL_OFFSET 2  → channels 2, 3
//   Device 2: SERVO_CHANNEL_OFFSET 4  → channels 4, 5
#define SERVO_CHANNEL_OFFSET  2
#define SERVO_COUNT           2

void handle_SERVO_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer);

static struct {
    int16_t servo_cmd;
    int32_t last_update;
} servos[SERVO_COUNT];