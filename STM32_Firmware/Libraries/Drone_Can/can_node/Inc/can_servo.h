#pragma once


// Private file include
#include <can_node.h>



void handle_SERVO_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer);

static struct {
    int16_t servo_cmd;
    int32_t last_update;
} servos[SERVO_COUNT];