#include "can_node.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "uavcan.protocol.dynamic_node_id.Allocation.h"
#include <canard.h>
#include <dronecan_msgs.h>
#include <string.h>

// ============================================================
//  Configuration
// ============================================================

// Set to 0 for dynamic node ID allocation
#define MY_NODE_ID        0
#define PREFERRED_NODE_ID 73

// How often to send BatteryInfo (milliseconds)
// PX4 expects at least 1 Hz; 2 Hz is a reasonable default
#define BATTERY_STATUS_PERIOD_MS  500

// Number of battery packs (pairs of cells in series)
// Each pack has: 2 voltage sensors summed, 1 current sensor
#define BATTERY_COUNT  3

// ============================================================
//  Sensor input — implement these to read your ADC values
// ============================================================

// Return voltage in Volts for sensor index [0..5]
// Sensors 0,1 → pack 0 (series pair)
// Sensors 2,3 → pack 1
// Sensors 4,5 → pack 2
extern float battery_get_cell_voltage(uint8_t sensor_index);

// Return current in Amps for sensor index [0..2]
// One current sensor per pack
extern float battery_get_current(uint8_t sensor_index);

// Return temperature in Kelvin (or NaN if not available)
// Return UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_TEMP_HOT etc. flags if needed
extern float battery_get_temperature(uint8_t pack_index);

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
static uint64_t next_1hz_at_us      = 0;
static uint32_t next_battery_status_ms = 0;

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
//  Battery status broadcast
//
//  Sends uavcan.equipment.power.BatteryInfo for each of the
//  3 battery packs. Each pack is two cells in series:
//    voltage = cell[2*i] + cell[2*i + 1]
//    current = current_sensor[i]
//
//  PX4 differentiates packs by battery_id (0, 1, 2).
// ============================================================

static void send_battery_status(void)
{
    static uint8_t transfer_id = 0;

    for (uint8_t i = 0; i < BATTERY_COUNT; i++) {
        struct uavcan_equipment_power_BatteryInfo pkt;
        memset(&pkt, 0, sizeof(pkt));

        // Voltage: sum of the two series cells for this pack
        // float v0 = battery_get_cell_voltage(i * 2);
        // float v1 = battery_get_cell_voltage(i * 2 + 1);
        pkt.voltage  = battery_get_cell_voltage(i * 2);

        // Current from this pack's dedicated sensor
        pkt.current          = battery_get_current(i);

        // Temperature (Kelvin); set to 0 if not available
        pkt.temperature      = battery_get_temperature(i);

        // Power consumed — integrate elsewhere and supply here if available,
        // otherwise leave as 0 (unknown) and PX4 will estimate
        pkt.full_charge_capacity_wh = 0.0f;  // Set if known (Wh)
        pkt.remaining_capacity_wh   = 0.0f;  // Set if tracked

        // State of charge: NaN / 0 tells PX4 to estimate from voltage
        // Set to 0..1 if you have a fuel gauge
        pkt.state_of_charge_pct              = 0;
        pkt.state_of_charge_pct_stdev        = 127;  // 127 = unknown

        // // Cell voltages: report the two individual cells
        // pkt.cell_voltages.len         = 2;
        // pkt.cell_voltages.data[0]     = v0;
        // pkt.cell_voltages.data[1]     = v1;

        // Pack identity — PX4 uses this to distinguish multiple batteries
        pkt.battery_id = i;

        // Status flags
        pkt.status_flags =
            UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_IN_USE;

        uint8_t  buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];
        uint16_t len = uavcan_equipment_power_BatteryInfo_encode(&pkt, buffer);

        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        len);
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

    const char *name = "battery_monitor";
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
        default:
            break;
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
    DNA.node_id_allocation_unique_id_offset        = 0;

    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               on_transfer_received,
               should_accept_transfer,
               NULL);

#if MY_NODE_ID > 0
    canardSetLocalNodeID(&canard, MY_NODE_ID);
#endif

    HAL_FDCAN_Start(_hfdcan);
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

    // DNA phase — do not send battery status until we have a node ID
    if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
        if (millis32() > DNA.send_next_node_id_allocation_request_at_ms) {
            request_DNA();
        }
        return;
    }

    // Periodic battery status to PX4
    uint32_t now_ms = millis32();
    if (now_ms >= next_battery_status_ms) {
        next_battery_status_ms = now_ms + BATTERY_STATUS_PERIOD_MS;
        send_battery_status();
    }
}