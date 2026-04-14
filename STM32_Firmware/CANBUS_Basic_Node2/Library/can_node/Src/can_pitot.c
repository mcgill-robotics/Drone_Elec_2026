#include "can_pitot.h"


int16_t send_pitot_info(uint16_t differential_pressure, uint16_t static_pressure, uint16_t temperature)
{
    static uint8_t transfer_id = 0;

    struct uavcan_equipment_air_data_RawAirData pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.differential_pressure = differential_pressure;
    pkt.static_pressure = static_pressure;  
    pkt.static_pressure_sensor_temperature = temperature;

    uint32_t len = uavcan_equipment_air_data_RawAirData_encode(pkt, buff);

    return can_node_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        len);
    }
}