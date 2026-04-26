#include "can_node.h"
#include <can_servo.h>


// ============================================================
//  SERVO RawCommand handler
// ============================================================

void handle_SERVO_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer)
{
    (void)ins;

    struct uavcan_equipment_actuator_ArrayCommand cmd;
    if (uavcan_equipment_actuator_ArrayCommand_decode(transfer, &cmd)) {
        return;
    }   

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        uint8_t ch = SERVO_CHANNEL_OFFSET + i;
        if (ch < cmd.commands.len) {
            switch (cmd.commands.data[i].command_type) {
                case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS:
                    servos[i].servo_cmd = cmd.commands.data[i].command_value;
                    break;
                case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM:
                    // map PWM to -1 to 1, assuming 1500 trim. If the servo has natural PWM
                    // support then we should use it directly instead
                    servos[i].servo_cmd = (cmd.commands.data[i].command_value-1500)/500.0;
                    break;
            }
            servos[i].last_update = millis32();
        } else {
            // Channel not present in this packet — safe disarm
            servos[i].servo_cmd = 0;
        }
    }
}
