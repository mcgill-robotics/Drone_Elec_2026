#pragma once

// Private file include
#include <can_node.h>

void handle_arm_status(CanardInstance *ins, CanardRxTransfer *transfer);

extern volatile bool arm_status;