#include "can_arm.h"

volatile bool arm_status = false;

void handle_arm_status(CanardInstance *ins, CanardRxTransfer *transfer){
    (void) ins;
    struct uavcan_equipment_safety_ArmingStatus msg;
    uavcan_equipment_safety_ArmingStatus_decode(transfer, &msg);

    switch (msg.status) {
    case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED:
        arm_status = true;
        break;
    default:
        arm_status = false;
        break;
    }

}