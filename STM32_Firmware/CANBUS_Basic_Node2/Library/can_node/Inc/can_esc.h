#pragma once

#include "stm32g4xx_hal.h"

// Private file include
#include <can_node.h>
#include <dronecan_msgs.h>


// This device controls ESC channels [ESC_CHANNEL_OFFSET] and [ESC_CHANNEL_OFFSET + 1].
// Set a different value per device so they don't overlap:
//   Device 0: ESC_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: ESC_CHANNEL_OFFSET 2  → channels 2, 3
//   Device 2: ESC_CHANNEL_OFFSET 4  → channels 4, 5
#define ESC_CHANNEL_OFFSET  2
#define ESC_COUNT           2

// Send esc status regularly
void send_esc_status(void);

// Handle esc input in can_node when received
void handle_ESC_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer);


// Last commanded values from PX4, range [-8192, 8191]
extern int16_t esc_raw_cmd[ESC_COUNT];