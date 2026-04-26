#include "can_battery.h"
#include "uavcan.equipment.power.BatteryInfo.h"
#include <complex.h>

int16_t send_battery_info(uint16_t voltage, uint16_t current)
{
    static uint8_t transfer_id = 0;

    struct uavcan_equipment_power_BatteryInfo pkt;
    memset(&pkt, 0, sizeof(pkt));
    uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];
    pkt.voltage = voltage;
    pkt.current = current;

    uint32_t len = uavcan_equipment_power_BatteryInfo_encode(&pkt, buffer);


    return can_node_broadcast(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        len);
}
