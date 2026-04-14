#include "can_node.h"
#include <can_esc.h>

// ============================================================
//  ESC RawCommand handler
//
//  PX4 broadcasts uavcan.equipment.esc.RawCommand (data type ID 1030)
//  containing up to 20 throttle values in [-8192, 8191].
//  We extract the two channels assigned to this device.
// ============================================================

void handle_ESC_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer)
{
    (void)ins;

    struct uavcan_equipment_esc_RawCommand cmd;
    uavcan_equipment_esc_RawCommand_decode(transfer, &cmd);

    for (uint8_t i = 0; i < ESC_COUNT; i++) {
        uint8_t ch = ESC_CHANNEL_OFFSET + i;
        if (ch < cmd.cmd.len) {
            esc[i].esc_cmd = cmd.cmd.data[ch];
        } else {
            // Channel not present in this packet — safe disarm
            esc[i].esc_cmd = 0;
        }
        esc[i].last_update = millis32();
    }
}

// ============================================================
//  ESC Status broadcast
//
//  PX4 requires periodic uavcan.equipment.esc.Status to mark
//  ESCs as alive. Broadcast one message per physical ESC.
// ============================================================

// Run once every 50ms
int16_t send_esc_status(void)
{
    static uint8_t transfer_id = 0;

    for (uint8_t i = 0; i < ESC_COUNT; i++) {
        struct uavcan_equipment_esc_Status status;
        memset(&status, 0, sizeof(status));

        status.esc_index    = ESC_CHANNEL_OFFSET + i;

        // Not measured so set to zero
        status.voltage      = 0.0;
        status.current      = 0.0;
        status.temperature  = 0.0;
        status.rpm          = 0;
        // Map [-8192,8191] throttle to [0,100] power percent for telemetry
        int32_t pct = ((int32_t)esc[i].esc_cmd + 8192) * 100 / 16383;
        status.power_rating_pct = (uint8_t)pct;
        status.error_count  = 0;

        uint8_t  buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
        uint16_t len = uavcan_equipment_esc_Status_encode(&status, buffer);

        can_node_broadcast(UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                        UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        len);
    }
}

