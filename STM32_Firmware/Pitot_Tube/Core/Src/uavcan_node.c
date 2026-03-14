//good luck
#include "uavcan_node.h"

#include "canard.h"
#include "stm32g4xx_hal_fdcan.h"
#include "uavcan.protocol.NodeStatus.h"

//Remove later and replace with dynamic node allocation
#define UAVCAN_NODE_ID 83

extern FDCAN_HandleTypeDef hfdcan1;

static CanardInstance canard;
static uint8_t canard_memory_pool[2048];

//Initialize canard instance
void uavcan_init(void) {
    canardInit(&canard, 
        canard_memory_pool, 
        sizeof(canard_memory_pool),
        NULL,
        NULL,
        NULL);      
    
    canard.node_id = UAVCAN_NODE_ID;
}

//Publishes heartbeat signal
static void publish_node_status(void) {
    static uint32_t uptime = 0;
    static uint8_t node_status_transfer_ID = 0;

    //Uses the DSDL compiled files to produce a default NodeStatus message
    //This message must be serialized into bytes in order to be sent
    struct uavcan_protocol_NodeStatus msg;
    msg.uptime_sec = uptime++;
    msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    uint8_t heartbeat_buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    uint16_t heartbeat_len = uavcan_protocol_NodeStatus_encode(&msg, heartbeat_buffer);

    CanardTxTransfer heartbeat_transfer;
    canardInitTxTransfer(&heartbeat_transfer);

    heartbeat_transfer.transfer_type = CanardTransferTypeBroadcast;
    heartbeat_transfer.data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
    heartbeat_transfer.data_type_id = UAVCAN_PROTOCOL_NODESTATUS_ID;
    heartbeat_transfer.inout_transfer_id = &node_status_transfer_ID;
    heartbeat_transfer.priority = CANARD_TRANSFER_PRIORITY_MEDIUM;
    heartbeat_transfer.payload = &heartbeat_buffer;
    heartbeat_transfer.payload_len = heartbeat_len;
    
    canardBroadcastObj(&canard, &heartbeat_transfer);
}

void publish_airspeed_data(void) {
    
}