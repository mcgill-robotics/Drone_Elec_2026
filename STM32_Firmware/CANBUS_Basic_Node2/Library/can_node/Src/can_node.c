#include "can_node.h"


// ============================================================
//  Libcanard
// ============================================================

// Set to 0 for dynamic node ID allocation (required for multi-node bus)
#define MY_NODE_ID        0
#define PREFERRED_NODE_ID 73

static CanardInstance canard;
static uint8_t        memory_pool[1024];
static FDCAN_HandleTypeDef *_hfdcan;
static struct uavcan_protocol_NodeStatus node_status;
 


// ============================================================
//  Internal ring buffer queue
// ============================================================

typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  data_len;
} CanRawFrame;

#define RX_QUEUE_DEPTH 32

static volatile struct {
    CanRawFrame buf[RX_QUEUE_DEPTH];
    volatile uint16_t head;   // written by ISR
    volatile uint16_t tail;   // read  by task
} s_rx_ring;

static inline bool ring_push(const CanRawFrame *f)
{
    uint16_t next = (s_rx_ring.head + 1) % RX_QUEUE_DEPTH;
    if (next == s_rx_ring.tail) return false;  // full — drop
    s_rx_ring.buf[s_rx_ring.head] = *f;
    s_rx_ring.head = next;
    return true;
}

static inline bool ring_pop(CanRawFrame *f)
{
    if (s_rx_ring.head == s_rx_ring.tail) return false;  // empty
    *f = s_rx_ring.buf[s_rx_ring.tail];
    s_rx_ring.tail = (s_rx_ring.tail + 1) % RX_QUEUE_DEPTH;
    return true;
}


// ============================================================
//  Callback hooks (set by main.c)
// ============================================================

static can_isr_notify_fn s_rx_notify_fn = NULL;
static can_tx_ready_fn   s_tx_ready_fn  = NULL;

void can_node_set_rx_notify(can_isr_notify_fn fn) { s_rx_notify_fn = fn; }
void can_node_set_tx_ready(can_tx_ready_fn fn)    { s_tx_ready_fn  = fn; }


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
//  ISR entry point — called from HAL_FDCAN_RxFifo0Callback
// ============================================================

BaseType_t can_node_rx_isr(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef hdr;
    CanRawFrame frame;
    BaseType_t woken = pdFALSE;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &hdr, frame.data) != HAL_OK) break;
        if (hdr.IdType != FDCAN_EXTENDED_ID) continue;

        frame.id       = hdr.Identifier | CANARD_CAN_FRAME_EFF;
        frame.data_len = hdr.DataLength;
        ring_push(&frame);  // lock-free single-producer single-consumer
    }

    // Fire the registered notify hook (wired to vTaskNotifyGiveFromISR in main.c)
    if (s_rx_notify_fn) s_rx_notify_fn(&woken);
    return woken;
}

// ============================================================
//  Public polling functions (called from tasks in main.c)
// ============================================================

// Dequeue one frame and feed it to libcanard.
// Returns true if a frame was processed; call in a loop until false.
// Caller must hold the canard mutex.
bool can_node_dequeue_and_process(void)
{
    CanRawFrame raw;
    if (!ring_pop(&raw)) return false;

    CanardCANFrame frame;
    frame.id       = raw.id;
    frame.data_len = raw.data_len;
    memcpy(frame.data, raw.data, raw.data_len);
    canardHandleRxFrame(&canard, &frame, micros64());
    return true;
}

// Drain libcanard's TX queue into the FDCAN hardware FIFO.
// Caller must hold the canard mutex.
void can_node_flush_tx(void)
{
    static const uint32_t dlc_table[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };

    for (const CanardCANFrame *txf;
         (txf = canardPeekTxQueue(&canard)) != NULL; )
    {
        FDCAN_TxHeaderTypeDef hdr = {
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
        if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &hdr,
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

    // Assign name if desired 
    char name[50];
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

static struct {
    uint32_t send_next_node_id_allocation_request_at_ms;
    uint32_t node_id_allocation_unique_id_offset;
} DNA;

static void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        return;
    }

    DNA.send_next_node_id_allocation_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (HAL_GetTick() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        return;
    }

    struct uavcan_protocol_dynamic_node_id_Allocation msg;
    uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    get_unique_id(my_unique_id);

    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        return;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
        DNA.send_next_node_id_allocation_request_at_ms -=
            UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
    } else {
        canardSetLocalNodeID(ins, msg.node_id);
    }
}

static void request_DNA(void)
{
    const uint32_t now = millis32();
    static uint8_t node_id_allocation_transfer_id = 0;

    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (HAL_GetTick() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(PREFERRED_NODE_ID << 1U);

    if (DNA.node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;
    }

    uint8_t my_unique_id[16];
    get_unique_id(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1],
            &my_unique_id[DNA.node_id_allocation_unique_id_offset],
            uid_size);

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                    &node_id_allocation_transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &allocation_request[0],
                    (uint16_t)(uid_size + 1));
}

// DNA allocation poll — call from RX task after draining frames.
// Caller must hold the canard mutex. Returns 1 once ID is assigned.
int8_t can_node_poll_dna(void)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) return 1;
    if (millis32() > DNA.send_next_node_id_allocation_request_at_ms) {
        request_DNA();
        if (s_tx_ready_fn) s_tx_ready_fn();
    }
    return 0;
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

    if (transfer->transfer_type == CanardTransferTypeBroadcast) {
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
            handle_DNA_Allocation(ins, transfer);
            break;
        
        #ifdef USE_ESC
            case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
                // Only handle ESC commands once we have a node ID
                if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
                    handle_ESC_RawCommand(ins, transfer);
                }
                break;
        #endif
        
        #ifdef USE_SERVO
            prinf();
        #endif

        default:
            break;
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
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;

        #ifdef USE_ESC 
        case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
            *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
            return true;
        #endif

        #ifdef USE_SERVO
            prinf();
        #endif

        default:
            break;
        }
    }

    return false;
}

// ============================================================
//  1 Hz tasks
// ============================================================

void can_node_1hz_tasks(void)
{
    canardCleanupStaleTransfers(&canard, micros64());

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
                    buffer, len);

    // Signal TX — tx_ready_fn is safe to call from task context too
    if (s_tx_ready_fn) s_tx_ready_fn();
}

// ============================================================
//  Init
// ============================================================

void can_node_init(FDCAN_HandleTypeDef *hfdcan)
{
    timing_init();
    _hfdcan = hfdcan;

    s_rx_ring.head = s_rx_ring.tail = 0;

    DNA.send_next_node_id_allocation_request_at_ms = millis32();
    DNA.node_id_allocation_unique_id_offset        = 0;

    canardInit(&canard, memory_pool, sizeof(memory_pool),
               on_transfer_received, should_accept_transfer, NULL);

#if MY_NODE_ID > 0
    canardSetLocalNodeID(&canard, MY_NODE_ID);
#endif

    HAL_FDCAN_ActivateNotification(_hfdcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(_hfdcan);
}