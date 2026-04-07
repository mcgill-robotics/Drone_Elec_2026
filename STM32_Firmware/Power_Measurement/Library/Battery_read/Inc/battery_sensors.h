#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Initialise both ADS131M03 devices over SPI1.
 *         Call once after CubeMX-generated MX_SPI1_Init().
 * @return true if both devices responded with a valid ID.
 */
bool battery_sensors_init(void);

/**
 * @brief  Poll both ADS131M03 DRDY pins and read new samples when
 *         available. Poll TMCS1127 current sensors via ADC.
 *         Call this from your main loop or a periodic task — the
 *         ADS131M03 at OSR=256 produces new data at ~4 kSPS so
 *         calling every 1–10 ms is plenty.
 */
void battery_sensors_update(void);

// ---- These three are called by can_node.c -------------------

/** Return cell voltage in Volts for sensor_index 0..5. */
float battery_get_cell_voltage(uint8_t sensor_index);

/** Return pack current in Amps for pack_index 0..2. */
float battery_get_current(uint8_t sensor_index);

/** Return pack temperature in Kelvin, or 0 if unknown. */
float battery_get_temperature(uint8_t pack_index);