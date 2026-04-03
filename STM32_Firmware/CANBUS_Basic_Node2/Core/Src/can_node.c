#include "can_node.h"
#include "stm32g4xx_hal.h"
#include "uavcan.protocol.dynamic_node_id.Allocation.h"
#include <canard.h>
#include <dronecan_msgs.h>
#include <string.h>

// ============================================================
//  Configuration
// ============================================================

#define PREFERRED_NODE_ID 73

// ============================================================
//  Libcanard
// ============================================================

static CanardInstance canard;
static uint8_t        memory_pool[1024];

// ============================================================
//  HAL handle
// ============================================================

static FDCAN_HandleTypeDef *_hfdcan;

// ============================================================
//  Node state
// ============================================================

static struct uavcan_protocol_NodeStatus node_status;
static uint64_t next_1hz_at_us = 0;

// ============================================================
//  DWT microsecond timer
// ============================================================

static void timing_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint64_t micros64(void)
{
    static uint32_t last_cycles = 0;
    static uint64_t upper       = 0;

    uint32_t now = DWT->CYCCNT;
    if (now < last_cycles) {
        upper += (uint64_t)1 << 32;
    }
    last_cycles = now;

    return (upper | (uint64_t)now) / (SystemCoreClock / 1000000ULL);
}

static uint32_t millis32(void) {
    return micros64() / 1000ULL;
}

// ============================================================
//  Unique ID — reads STM32 factory 96-bit UID
// ============================================================

static void get_unique_id(uint8_t id[16])
{
    memset(id, 0, 16);
    uint32_t uid[3];
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();
    memcpy(id, uid, 12);
}
// ============================================================
//  RX — polled
// ============================================================

static void process_rx(void)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t               rx_data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(_hfdcan, FDCAN_RX_FIFO0,
                                   &rx_header, rx_data) != HAL_OK) {
            break;
        }

        if (rx_header.IdType != FDCAN_EXTENDED_ID) {
            continue;
        }

        CanardCANFrame frame;
        frame.id       = rx_header.Identifier | CANARD_CAN_FRAME_EFF;

        // Directly map datalength to data_len, does not need any conversion
        frame.data_len = rx_header.DataLength;
        memcpy(frame.data, rx_data, frame.data_len);

        canardHandleRxFrame(&canard, &frame, micros64());
    }
}

// ============================================================
//  TX — polled
// ============================================================

static void process_tx(void)
{
    static const uint32_t dlc_table[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };

    for (const CanardCANFrame *txf = NULL;
         (txf = canardPeekTxQueue(&canard)) != NULL; )
    {
        FDCAN_TxHeaderTypeDef tx_header = {
            .Identifier          = txf->id & CANARD_CAN_EXT_ID_MASK,
            .IdType              = FDCAN_EXTENDED_ID,
            .TxFrameType         = FDCAN_DATA_FRAME,
            .DataLength          = dlc_table[txf->data_len],
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch       = FDCAN_BRS_OFF,
            .FDFormat            = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
            .MessageMarker       = 0,
        };

        if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &tx_header,
                                          (uint8_t *)txf->data) == HAL_OK) {
            canardPopTxQueue(&canard);
        } else {
            break;
        }
    }
}

// ============================================================
//  DroneCAN service handlers
// ============================================================

static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = micros64() / 1000000ULL;
    pkt.status             = node_status;

    pkt.software_version.major = 1;
    pkt.software_version.minor = 0;
    pkt.hardware_version.major = 1;
    pkt.hardware_version.minor = 0;
    get_unique_id(pkt.hardware_version.unique_id);

    const char *name = "com.example.stm32_node";
    strncpy((char *)pkt.name.data, name, sizeof(pkt.name.data));
    pkt.name.len = strlen(name);

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           total_size);
}


// ============================================================
//  Dynamic ID allocation
// ============================================================

// Dynamic node id struct
static struct {
    uint32_t send_next_node_id_allocation_request_at_ms;
    uint32_t node_id_allocation_unique_id_offset;
} DNA;

// Node allocation
static void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        // already allocated
        return;
    }

    // Rule C - updating the randomized time interval
    DNA.send_next_node_id_allocation_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        ( HAL_GetTick() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        return;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation msg;

    uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    get_unique_id(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        // No match, return
        return;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to
        // the next stage and updating the timeout.
        DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
        DNA.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

    } else {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(ins, msg.node_id);
    }
}

/*
  ask for a dynamic node allocation
 */
static void request_DNA()
{
    const uint32_t now = millis32();
    static uint8_t node_id_allocation_transfer_id = 0;
    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (HAL_GetTick() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(PREFERRED_NODE_ID << 1U);

    if (DNA.node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;     // First part of unique ID
    }

    uint8_t my_unique_id[16];
    get_unique_id(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);
    
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                    &node_id_allocation_transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &allocation_request[0],
                    (uint16_t) (uid_size + 1));

}

// ============================================================
//  Libcanard callbacks
// ============================================================

static void on_transfer_received(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
            handle_GetNodeInfo(ins, transfer);
            break;
        default:
            break;
        }
    }

    if (transfer->transfer_type == CanardTransferTypeBroadcast){
        switch(transfer->data_type_id){
            case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
                handle_DNA_Allocation(ins, transfer);
            }
        }
    }
}

static bool should_accept_transfer(const CanardInstance *ins,
                                   uint64_t *out_data_type_signature,
                                   uint16_t data_type_id,
                                   CanardTransferType transfer_type,
                                   uint8_t source_node_id)
{
    (void)ins; (void)source_node_id;

    if (transfer_type == CanardTransferTypeRequest) {
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        default:
            break;
        }
    }

    if (transfer_type == CanardTransferTypeBroadcast) {
        // see if we want to handle a specific broadcast packet
        switch (data_type_id) {
            case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
                *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
                return true;
            }
        }
    }
    return false;
}

// ============================================================
//  1 Hz tasks
// ============================================================

static void send_node_status(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];

    node_status.uptime_sec                  = micros64() / 1000000ULL;
    node_status.health                      = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode                        = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode                    = 0;
    node_status.vendor_specific_status_code = 0;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    static uint8_t transfer_id;
    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

static void process_1hz_tasks(uint64_t ts_usec)
{
    canardCleanupStaleTransfers(&canard, ts_usec);
    send_node_status();
}

// ============================================================
//  Public API
// ============================================================

void can_node_init(FDCAN_HandleTypeDef *hfdcan)
{

    timing_init();

    _hfdcan = hfdcan;

    DNA.send_next_node_id_allocation_request_at_ms = millis32();

    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               on_transfer_received,
               should_accept_transfer,
               NULL);


    HAL_FDCAN_Start(_hfdcan);
    // No notification needed for polling — FIFO fill level is checked directly
}

void can_node_update(void)
{
    process_rx(); 
    process_tx();

    uint64_t ts = micros64();
    if (ts >= next_1hz_at_us) {
        next_1hz_at_us = ts + 1000000ULL;
        process_1hz_tasks(ts);
    }


    if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
        // waiting for DNA
    }

    // see if we are still doing DNA
    if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
        // we're still waiting for a DNA allocation of our node ID
        if (millis32() > DNA.send_next_node_id_allocation_request_at_ms) {
            request_DNA();
        }
        return;
    }
}