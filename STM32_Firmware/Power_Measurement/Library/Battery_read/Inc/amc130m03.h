#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * AMC130M03 — 3-channel 24-bit delta-sigma ADC driver
 *
 * SPI: CPOL=0, CPHA=1 (Mode 1), MSB first
 * One frame = 6 words × 3 bytes = 18 bytes
 *
 * TX frame: [CMD   ][0x000000][0x000000][0x000000][0x000000][0x000000]
 * RX frame: [STATUS][CH0     ][CH1     ][CH2     ][CRC     ][padding ]
 *
 * All words are 24-bit (3 bytes), MSB first.
 * Commands are 16-bit values sent in the upper 2 bytes of a 24-bit word
 * (i.e. passed as a uint32_t = cmd << 8, so byte[0]=cmd_hi, byte[1]=cmd_lo, byte[2]=0x00).
 */

/* ---------- Commands (16-bit) -------------------------------------------- */
#define AMC130_CMD_NULL         0x0000u
#define AMC130_CMD_RESET        0x0011u
#define AMC130_CMD_STANDBY      0x0022u
#define AMC130_CMD_WAKEUP       0x0033u
#define AMC130_CMD_LOCK         0x0555u
#define AMC130_CMD_UNLOCK       0x0655u

/*
 * Register read/write command encoding (AMC130M03 datasheet, Section 8.5.2):
 *   Bits [15:12] = 1010 (RREG) or 0110 (WREG)
 *   Bits [11:8]  = 0000 (reserved)
 *   Bits [7:2]   = 6-bit register address
 *   Bits [1:0]   = (count - 1), max 3
 */
#define AMC130_CMD_RREG(addr, n)  (0xA000u | (((uint16_t)(addr) & 0x3Fu) << 2) | (((n) - 1u) & 0x03u))
#define AMC130_CMD_WREG(addr, n)  (0x6000u | (((uint16_t)(addr) & 0x3Fu) << 2) | (((n) - 1u) & 0x03u))

/* ---------- Register addresses ------------------------------------------- */
#define AMC130_REG_ID            0x00u
#define AMC130_REG_STATUS        0x01u
#define AMC130_REG_MODE          0x02u
#define AMC130_REG_CLOCK         0x03u
#define AMC130_REG_GAIN          0x04u
#define AMC130_REG_CFG           0x06u
#define AMC130_REG_CH0_CFG       0x09u
#define AMC130_REG_CH1_CFG       0x0Eu
#define AMC130_REG_CH2_CFG       0x13u

/* ---------- CLOCK register (0x03) ---------------------------------------- */
/*
 * Bits [10:8] = channel enables (1 = on)
 * Bits [4:2]  = OSR: 000=4096, 001=2048, 010=1024, 011=512,
 *                    100=256,  101=128,  110=64
 * Bit  [0]    = PWR: 0=very-low-power, 1=high-resolution (keep 1)
 */
#define AMC130_CLOCK_CH0_EN      (1u << 8)
#define AMC130_CLOCK_CH1_EN      (1u << 9)
#define AMC130_CLOCK_CH2_EN      (1u << 10)
#define AMC130_CLOCK_ALL_CH      (AMC130_CLOCK_CH0_EN | AMC130_CLOCK_CH1_EN | AMC130_CLOCK_CH2_EN)

#define AMC130_CLOCK_OSR_4096    (0u << 2)
#define AMC130_CLOCK_OSR_2048    (1u << 2)
#define AMC130_CLOCK_OSR_1024    (2u << 2)
#define AMC130_CLOCK_OSR_512     (3u << 2)
#define AMC130_CLOCK_OSR_256     (4u << 2)   /* 4 kSPS @ 1.024 MHz CLKIN */
#define AMC130_CLOCK_OSR_128     (5u << 2)
#define AMC130_CLOCK_OSR_64      (6u << 2)

#define AMC130_CLOCK_HIGH_RES    (1u << 0)   /* Set for best noise performance */

/* ---------- GAIN register (0x04) ----------------------------------------- */
/*
 * Each channel has a 3-bit field:
 *   CH0: bits [2:0], CH1: bits [6:4], CH2: bits [10:8]
 *   000=1, 001=2, 010=4, 011=8, 100=16, 101=32, 110=64, 111=128
 */
#define AMC130_GAIN_1            0u
#define AMC130_GAIN_2            1u
#define AMC130_GAIN_4            2u
#define AMC130_GAIN_8            3u
#define AMC130_GAIN_16           4u
#define AMC130_GAIN_32           5u
#define AMC130_GAIN_64           6u
#define AMC130_GAIN_128          7u

#define AMC130_GAIN_REG(g0, g1, g2) \
    ((uint16_t)(((g2) & 0x07u) << 8) | \
     (uint16_t)(((g1) & 0x07u) << 4) | \
     (uint16_t)(((g0) & 0x07u)     ))

/* ---------- Physical constants ------------------------------------------- */
#define AMC130_VREF_V            1.25f    /* Internal reference voltage (V) */
#define AMC130_FULL_SCALE        (1u << 23)

/* ---------- Expected ID register value ------------------------------------ */
#define AMC130_ID_EXPECTED       0x2200u  /* Upper 16 bits of the ID word */

/* ---------- Device handle ------------------------------------------------ */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    int32_t  raw[3];           /* Last raw 24-bit two's-complement readings   */
    float    lsb_uv[3];        /* LSB size in microvolts per channel          */
} AMC130M03_t;

/* ---------- API ---------------------------------------------------------- */

/**
 * @brief  Initialise the AMC130M03.
 *         Sends RESET, verifies ID, configures clock and gain.
 *         Call after SPI peripheral is initialised.
 * @return true  on success (device ID matched)
 *         false if the device did not respond correctly
 */
bool amc130m03_init(AMC130M03_t *dev,
                    SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin);

/**
 * @brief  Read all 3 channels if DRDY is asserted (active-low).
 *         Stores results in dev->raw[0..2].
 * @return true  if DRDY was low and new data was read
 *         false if no new data was ready
 */
bool amc130m03_read(AMC130M03_t *dev,
                    GPIO_TypeDef *drdy_port,
                    uint16_t drdy_pin);

/**
 * @brief  Convert the last raw reading for a channel to millivolts.
 * @param  channel  0, 1 or 2
 * @return Voltage in mV, or 0.0f for an invalid channel index
 */
float amc130m03_get_mv(const AMC130M03_t *dev, uint8_t channel);