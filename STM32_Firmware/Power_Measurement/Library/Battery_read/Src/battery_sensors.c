#include "battery_sensors.h"
#include "ads131m03.h"
#include "stm32g4xx_hal.h"
#include <string.h>

// ============================================================
//  Wiring summary
//  ---------------------------------------------------------------
//  ADS131M03 #1 (CS1)  — voltage sensors for packs 0 and 1
//    CH0 → cell voltage 0  (pack 0, low cell)
//    CH1 → cell voltage 1  (pack 0, high cell)
//    CH2 → cell voltage 2  (pack 1, low cell)
//
//  ADS131M03 #2 (CS2)  — voltage sensors for pack 2 + spare
//    CH0 → cell voltage 3  (pack 1, high cell)
//    CH1 → cell voltage 4  (pack 2, low cell)
//    CH2 → cell voltage 5  (pack 2, high cell)
//
//  TMCS1127C1A (×3, read via STM32 ADC)
//    ADC CH0 → pack 0 current
//    ADC CH1 → pack 1 current
//    ADC CH2 → pack 2 current
//
//    TMCS1127C1A:  VREF = 0.33 V,  sensitivity = 25 mV/A
//    Transfer:  I = (VOUT - VREF) / S  →  I = (VOUT - 0.33) / 0.025
//
//  STM32G4 ADC:
//    12-bit, VDDA = 3.3 V  →  LSB = 3.3 / 4096 V
// ============================================================

// ---- Configuration — adjust to your hardware ----------------

// SPI peripheral shared between both ADS131M03 devices
extern SPI_HandleTypeDef hspi1;

// CS GPIO for each ADS131M03
#define ADS_CS1_PORT   GPIOB
#define ADS_CS1_PIN    GPIO_PIN_11

#define ADS_CS2_PORT   GPIOB
#define ADS_CS2_PIN    GPIO_PIN_12

// DRDY GPIO for each ADS131M03 (can tie together if you don't care
// about simultaneous sampling — just poll both)
#define ADS_DRDY1_PORT  GPIOB
#define ADS_DRDY1_PIN   GPIO_PIN_13

#define ADS_DRDY2_PORT  GPIOB
#define ADS_DRDY2_PIN   GPIO_PIN_13

// STM32 ADC handle and channels for TMCS1127 current sensors
extern ADC_HandleTypeDef hadc1;

// ADC channel rank that each current sensor occupies.
// These correspond to the order in which your ADC is configured
// in CubeMX (regular sequence rank 1, 2, 3).
#define CURRENT_ADC_CHANNELS  3

// TMCS1127C1A parameters
#define TMCS1127_SENSITIVITY_V_PER_A   0.025f   // 25 mV/A
#define TMCS1127_VREF_V                0.33f    // zero-current output

// STM32G4 ADC reference — adjust if using external VREF
#define ADC_VREF_V   3.3f
#define ADC_BITS     12
#define ADC_COUNTS   ((float)(1 << ADC_BITS))  // 4096

// ---- Low-pass filter for current readings -------------------
// Simple single-pole IIR:  y = alpha*x + (1-alpha)*y_prev
// alpha=1.0 disables filtering
#define CURRENT_FILTER_ALPHA  0.2f

// ---- State --------------------------------------------------

static ADS131M03_t  adc1, adc2;
static float        cell_voltage_v[6];   // v[0..5], raw from ADS131M03
static float        current_a[3];        // filtered current per pack

static bool sensors_ready = false;

// ============================================================
//  Internal: read all ADC channels for TMCS1127
//  Triggers a regular conversion sequence and reads all 3 ranks.
// ============================================================

static void read_current_sensors(void)
{
    uint32_t raw[CURRENT_ADC_CHANNELS];

    // Start injected or regular conversion depending on your CubeMX setup.
    // Using regular scan mode with DMA-less polling here.
    for (uint8_t i = 0; i < CURRENT_ADC_CHANNELS; i++) {
        // HAL_ADC_Start triggers the next rank in the scan sequence
        if (HAL_ADC_Start(&hadc1) != HAL_OK) {
            raw[i] = 0;
            continue;
        }
        if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
            raw[i] = HAL_ADC_GetValue(&hadc1);
        } else {
            raw[i] = 0;
        }
        HAL_ADC_Stop(&hadc1);
    }

    // Convert ADC counts → current in Amps using TMCS1127 transfer function:
    //   VOUT = VREF + S * I  →  I = (VOUT - VREF) / S
    for (uint8_t i = 0; i < CURRENT_ADC_CHANNELS; i++) {
        float vout = ((float)raw[i] / ADC_COUNTS) * ADC_VREF_V;
        float i_new = (vout - TMCS1127_VREF_V) / TMCS1127_SENSITIVITY_V_PER_A;

        // Apply low-pass filter
        current_a[i] = CURRENT_FILTER_ALPHA * i_new +
                       (1.0f - CURRENT_FILTER_ALPHA) * current_a[i];
    }
}

// ============================================================
//  Internal: read all voltage channels from both ADS131M03
// ============================================================

static void read_voltage_sensors(void)
{
    // Device 1: CH0=cell0, CH1=cell1, CH2=cell2
    if (ads131m03_read(&adc1, ADS_DRDY1_PORT, ADS_DRDY1_PIN)) {
        cell_voltage_v[0] = ads131m03_get_voltage(&adc1, 0);
        cell_voltage_v[1] = ads131m03_get_voltage(&adc1, 1);
        cell_voltage_v[2] = ads131m03_get_voltage(&adc1, 2);
    }

    // Device 2: CH0=cell3, CH1=cell4, CH2=cell5
    if (ads131m03_read(&adc2, ADS_DRDY2_PORT, ADS_DRDY2_PIN)) {
        cell_voltage_v[3] = ads131m03_get_voltage(&adc2, 0);
        cell_voltage_v[4] = ads131m03_get_voltage(&adc2, 1);
        cell_voltage_v[5] = ads131m03_get_voltage(&adc2, 2);
    }
}

// ============================================================
//  Public API
// ============================================================

bool battery_sensors_init(void)
{
    memset(cell_voltage_v, 0, sizeof(cell_voltage_v));
    memset(current_a,      0, sizeof(current_a));

    bool ok1 = ads131m03_init(&adc1, &hspi1, ADS_CS1_PORT, ADS_CS1_PIN);
    bool ok2 = ads131m03_init(&adc2, &hspi1, ADS_CS2_PORT, ADS_CS2_PIN);

    sensors_ready = ok1 && ok2;
    return sensors_ready;
}

void battery_sensors_update(void)
{
    // if (!sensors_ready) return;
    read_voltage_sensors();
    read_current_sensors();
}

// ============================================================
//  Extern functions called by can_node.c
// ============================================================

/**
 * @brief  Return individual cell voltage in Volts.
 *         sensor_index 0..5 maps to the 6 resistor-divider inputs.
 *
 *  IMPORTANT: The ADS131M03 measures a differential voltage at its
 *  input pins.  If your resistor divider presents the cell voltage
 *  directly as a differential signal, this value is already correct.
 *  If you use a voltage divider (e.g. to scale 4.2V into the 1.2V
 *  reference range), multiply by your divider ratio here:
 *
 *    Example: top resistor 100k, bottom resistor 33k
 *    ratio = (100+33)/33 = 4.03
 *    return cell_voltage_v[sensor_index] * 4.03f;
 */
float battery_get_cell_voltage(uint8_t sensor_index)
{
    if (sensor_index >= 6) return 0.0f;

    // *** Adjust VOLTAGE_DIVIDER_RATIO to match your resistor network ***
    // Set to 1.0f if the ADS131M03 input is already at battery voltage
    // (unlikely for cells above 1.2 V — you almost certainly need a divider)
    const float VOLTAGE_DIVIDER_RATIO = 10.0f / 604.0f;

    return cell_voltage_v[sensor_index] * VOLTAGE_DIVIDER_RATIO;
}

/**
 * @brief  Return pack current in Amps (positive = discharge).
 *         sensor_index 0..2, one per pack.
 */
float battery_get_current(uint8_t sensor_index)
{
    if (sensor_index >= 3) return 0.0f;
    return current_a[sensor_index];
}

/**
 * @brief  Return pack temperature in Kelvin, or 0 if not available.
 *         Stub — wire up a thermistor or NTC here if you have one.
 */
float battery_get_temperature(uint8_t pack_index)
{
    (void)pack_index;
    // Return 0 to signal "unknown" to the BatteryInfo encoder
    return 0.0f;
}