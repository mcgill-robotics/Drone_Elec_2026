#include "ads131m03.h"
#include <string.h>

// ============================================================
//  Internal helpers
// ============================================================

static void cs_low(ADS131M03_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void cs_high(ADS131M03_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// Send one 24-bit word (3 bytes) and receive one 24-bit word.
// CS must already be low. Returns received 24-bit value.
static uint32_t spi_xfer_word(ADS131M03_t *dev, uint32_t tx_word)
{
    uint8_t tx[3] = {
        (tx_word >> 16) & 0xFF,
        (tx_word >>  8) & 0xFF,
        (tx_word      ) & 0xFF
    };
    uint8_t rx[3] = {0, 0, 0};

    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 3, 10);

    return ((uint32_t)rx[0] << 16) |
           ((uint32_t)rx[1] <<  8) |
           ((uint32_t)rx[2]      );
}

// Write a 16-bit register (padded to 24-bit word as per ADS131M03
// framing: command word + data word, each 3 bytes, CS held low).
static void write_reg(ADS131M03_t *dev, uint8_t addr, uint16_t value)
{
    cs_low(dev);
    spi_xfer_word(dev, (uint32_t)ADS131M03_CMD_WREG(addr, 1) << 8);
    spi_xfer_word(dev, (uint32_t)value << 8);
    // Remaining 4 dummy words to complete 6-word frame
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    cs_high(dev);
    HAL_Delay(1);
}

// Convert raw 24-bit two's complement to signed int32
static int32_t raw24_to_int32(uint32_t raw)
{
    // Sign-extend from 24 bits
    if (raw & 0x800000) {
        return (int32_t)(raw | 0xFF000000);
    }
    return (int32_t)raw;
}

// ============================================================
//  Public API
// ============================================================

bool ads131m03_init(ADS131M03_t *dev,
                    SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin)
{
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    memset(dev->raw, 0, sizeof(dev->raw));

    // Gain=1 on all channels: LSB = VREF / 2^23
    float lsb = ADS131M03_VREF / (float)(1 << 23);
    dev->lsb_to_volts[0] = lsb;
    dev->lsb_to_volts[1] = lsb;
    dev->lsb_to_volts[2] = lsb;

    // Ensure CS is deasserted, wait for POR (250 µs max per datasheet)
    cs_high(dev);
    HAL_Delay(2);

    // Send RESET command in a 6-word frame
    cs_low(dev);
    spi_xfer_word(dev, (uint32_t)ADS131M03_CMD_RESET << 8);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    cs_high(dev);
    HAL_Delay(2);  // Wait for reset to complete (tREGACQ = 5µs, round up)

    // Read ID register to verify comms. ID[7:0] should be 0x22 for ADS131M03.
    cs_low(dev);
    spi_xfer_word(dev, (uint32_t)ADS131M03_CMD_RREG(ADS131M03_REG_ID, 1) << 8);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    cs_high(dev);
    HAL_Delay(1);

    // Read response (next frame's STATUS word contains the register value)
    cs_low(dev);
    uint32_t status_word = spi_xfer_word(dev, 0);  // STATUS
    uint32_t id_word     = spi_xfer_word(dev, 0);  // reg data
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);
    cs_high(dev);
    (void)status_word;

    uint8_t id = (id_word >> 8) & 0xFF;
    if (id != 0x22) {
        // Communication error — ADS131M03 ID[7:0] must be 0x22
        return false;
    }

    // Configure CLOCK register: enable all 3 channels, OSR=256 (4kSPS)
    uint16_t clock_val = ADS131M03_CLOCK_OSR_256 | ADS131M03_CLOCK_ALL_CH;
    write_reg(dev, ADS131M03_REG_CLOCK, clock_val);

    // Configure GAIN: gain=1 on all channels (default, write explicitly)
    uint16_t gain_val = ADS131M03_GAIN_CH0(ADS131M03_GAIN_1) |
                        ADS131M03_GAIN_CH1(ADS131M03_GAIN_1) |
                        ADS131M03_GAIN_CH2(ADS131M03_GAIN_1);
    write_reg(dev, ADS131M03_REG_GAIN, gain_val);

    return true;
}

bool ads131m03_read(ADS131M03_t *dev,
                    GPIO_TypeDef *drdy_port,
                    uint16_t drdy_pin)
{
    // DRDY is active-low; return false if no new data
    if (HAL_GPIO_ReadPin(drdy_port, drdy_pin) != GPIO_PIN_RESET) {
        return false;
    }

    // Send NULL command to clock out the conversion frame.
    // Frame layout (6 words × 3 bytes each):
    //   TX: NULL, 0, 0, 0, 0, 0
    //   RX: STATUS, CH0[23:0], CH1[23:0], CH2[23:0], CRC, (padding)
    cs_low(dev);

    spi_xfer_word(dev, (uint32_t)ADS131M03_CMD_NULL << 8);  // STATUS returned
    uint32_t ch0 = spi_xfer_word(dev, 0);
    uint32_t ch1 = spi_xfer_word(dev, 0);
    uint32_t ch2 = spi_xfer_word(dev, 0);
    spi_xfer_word(dev, 0);  // CRC (ignored)
    spi_xfer_word(dev, 0);  // padding

    cs_high(dev);

    // Each channel word is 24 bits in the upper 3 bytes of the 32-bit transfer
    dev->raw[0] = raw24_to_int32(ch0);
    dev->raw[1] = raw24_to_int32(ch1);
    dev->raw[2] = raw24_to_int32(ch2);

    return true;
}

float ads131m03_get_voltage(const ADS131M03_t *dev, uint8_t channel)
{
    if (channel > 2) return 0.0f;
    return (float)dev->raw[channel] * dev->lsb_to_volts[channel];
}