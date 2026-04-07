#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================
//  ADS131M03 — 3-channel 24-bit delta-sigma ADC driver
//  SPI: CPOL=0, CPHA=1 (Mode 1), MSB first, word size 24-bit
//  transferred as 3 bytes per word.
//
//  One SPI frame for a data read = 6 words × 3 bytes = 18 bytes:
//    TX: [CMD_NULL][0x0000][0x0000][0x0000][0x0000][0x0000]
//    RX: [STATUS  ][CH0   ][CH1   ][CH2   ][CRC   ][-----]
//  All words are 24 bits (3 bytes), MSB first.
// ============================================================

// ---- Commands -----------------------------------------------
#define ADS131M03_CMD_NULL       0x0000
#define ADS131M03_CMD_RESET      0x0011
#define ADS131M03_CMD_STANDBY    0x0022
#define ADS131M03_CMD_WAKEUP     0x0033
#define ADS131M03_CMD_LOCK       0x0555
#define ADS131M03_CMD_UNLOCK     0x0655
// RREG: 0b 1010 aaaa aaaa cccc  (a=addr 8-bit, c=count-1 4-bit)
#define ADS131M03_CMD_RREG(addr, n)  (0xA000 | (((addr) & 0x3F) << 7) | (((n)-1) & 0x0F))
// WREG: 0b 0110 aaaa aaaa cccc
#define ADS131M03_CMD_WREG(addr, n)  (0x6000 | (((addr) & 0x3F) << 7) | (((n)-1) & 0x0F))

// ---- Register addresses -------------------------------------
#define ADS131M03_REG_ID         0x00
#define ADS131M03_REG_STATUS     0x01
#define ADS131M03_REG_MODE       0x02
#define ADS131M03_REG_CLOCK      0x03
#define ADS131M03_REG_GAIN       0x04
#define ADS131M03_REG_CFG        0x06
#define ADS131M03_REG_THRSHLD_MSB 0x07
#define ADS131M03_REG_THRSHLD_LSB 0x08
#define ADS131M03_REG_CH0_CFG    0x09
#define ADS131M03_REG_CH0_OCAL_MSB 0x0A
#define ADS131M03_REG_CH0_OCAL_LSB 0x0B
#define ADS131M03_REG_CH0_GCAL_MSB 0x0C
#define ADS131M03_REG_CH0_GCAL_LSB 0x0D
#define ADS131M03_REG_CH1_CFG    0x0E
#define ADS131M03_REG_CH1_OCAL_MSB 0x0F
#define ADS131M03_REG_CH1_OCAL_LSB 0x10
#define ADS131M03_REG_CH1_GCAL_MSB 0x11
#define ADS131M03_REG_CH1_GCAL_LSB 0x12
#define ADS131M03_REG_CH2_CFG    0x13
#define ADS131M03_REG_CH2_OCAL_MSB 0x14
#define ADS131M03_REG_CH2_OCAL_LSB 0x15
#define ADS131M03_REG_CH2_GCAL_MSB 0x16
#define ADS131M03_REG_CH2_GCAL_LSB 0x17
#define ADS131M03_REG_REGMAP_CRC  0x3E

// ---- CLOCK register bits ------------------------------------
// OSR[2:0] field in CLOCK register bits [4:2]
// 000=4096, 001=2048, 010=1024, 011=512, 100=256, 101=128, 110=64
#define ADS131M03_CLOCK_OSR_256  (0x04 << 2)  // 4kSPS @ 8.192MHz CLKIN
#define ADS131M03_CLOCK_CH0_EN   (1 << 8)
#define ADS131M03_CLOCK_CH1_EN   (1 << 9)
#define ADS131M03_CLOCK_CH2_EN   (1 << 10)
#define ADS131M03_CLOCK_ALL_CH   (ADS131M03_CLOCK_CH0_EN | \
                                   ADS131M03_CLOCK_CH1_EN | \
                                   ADS131M03_CLOCK_CH2_EN)

// ---- GAIN register ------------------------------------------
// PGA_GAIN_x[2:0] for each channel: 000=1,001=2,010=4,011=8,...
#define ADS131M03_GAIN_CH0(g)  ((g) & 0x07)
#define ADS131M03_GAIN_CH1(g)  (((g) & 0x07) << 4)
#define ADS131M03_GAIN_CH2(g)  (((g) & 0x07) << 8)
#define ADS131M03_GAIN_1       0
#define ADS131M03_GAIN_2       1
#define ADS131M03_GAIN_4       2
#define ADS131M03_GAIN_8       3

// ---- Internal reference voltage -----------------------------
#define ADS131M03_VREF          1.2f   // V

// ---- Device instance ----------------------------------------
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    // Cached last conversion results, raw 24-bit two's complement
    int32_t  raw[3];
    // Conversion factor: (Vref / gain) / 2^23  [V/LSB]
    float    lsb_to_volts[3];
} ADS131M03_t;

// ---- API ----------------------------------------------------

/**
 * @brief  Initialise device. Sends RESET, configures all 3 channels
 *         at the requested OSR and gain=1. Call after SPI1 is started.
 * @return true on success (device responded with expected ID)
 */
bool ads131m03_init(ADS131M03_t *dev,
                    SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin);

/**
 * @brief  Poll for new data (checks DRDY pin level) and read all 3
 *         channels in one SPI frame. Stores results in dev->raw[].
 * @param  drdy_port  GPIO port of the DRDY pin
 * @param  drdy_pin   GPIO pin number of the DRDY pin
 * @return true if new data was available and read
 */
bool ads131m03_read(ADS131M03_t *dev,
                    GPIO_TypeDef *drdy_port,
                    uint16_t drdy_pin);

/**
 * @brief  Return channel voltage in Volts (uses stored raw value).
 *         Assumes gain=1 and VREF=1.2V.
 */
float ads131m03_get_voltage(const ADS131M03_t *dev, uint8_t channel);