#include "can_node.h"
#include <string.h>

// ============================================================
//  Internal queue — plain ring buffer, no RTOS types
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
//  Libcanard + HAL state
// ============================================================

static CanardInstance        canard;
static uint8_t               memory_pool[1024];
static FDCAN_HandleTypeDef  *_hfdcan;
static struct uavcan_protocol_NodeStatus node_status;

// ============================================================
//  DWT timer (unchanged)
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
    if (now < last_cycles) upper += (uint64_t)1 << 32;
    last_cycles = now;
    return (upper | (uint64_t)now) / (SystemCoreClock / 1000000ULL);
}

static uint32_t millis32(void) { return micros64() / 1000ULL; }

// ============================================================
//  Unique ID
// ============================================================

static void get_unique_id(uint8_t id[16])
{
    memset(id, 0, 16);
    uint32_t uid[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };
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
//  1 Hz tasks (unchanged internals)
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