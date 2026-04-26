#pragma once


// Private file include
#include <can_node.h>



void handle_SERVO_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer);


// Servo_cmd carries the servo output value in range 1000 to 2000
// last_update is incremented each time the value is updated
typedef struct {
    uint16_t servo_cmd;
    uint32_t last_update;
} servo_state;

extern volatile servo_state servos[SERVO_COUNT];