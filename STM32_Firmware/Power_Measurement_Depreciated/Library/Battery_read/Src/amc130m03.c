#include "amc130m03.h"
#include <string.h>

/* ============================================================
 *  Internal helpers
 * ============================================================ */

static inline void cs_low(AMC130M03_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(AMC130M03_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/*
 * Transfer one 24-bit word (3 bytes) over SPI.
 * CS must already be asserted before calling.
 *
 * @param  tx  24-bit value to transmit (upper byte first)
 * @return     24-bit value received
 */
static uint32_t spi_word(AMC130M03_t *dev, uint32_t tx)
{
    uint8_t txb[3] = { (tx >> 16) & 0xFF, (tx >> 8) & 0xFF, tx & 0xFF };
    uint8_t rxb[3] = { 0, 0, 0 };

    HAL_SPI_TransmitReceive(dev->hspi, txb, rxb, 3, 10);

    return ((uint32_t)rxb[0] << 16) |
           ((uint32_t)rxb[1] <<  8) |
           ((uint32_t)rxb[2]      );
}

/*
 * Send a 16-bit command as the first word of a 6-word frame.
 * The command occupies the upper 2 bytes; the third byte is 0x00.
 * Five dummy words complete the frame.
 *
 * @param  cmd  16-bit command word (e.g. AMC130_CMD_RESET)
 */
static void send_command(AMC130M03_t *dev, uint16_t cmd)
{
    cs_low(dev);
    spi_word(dev, (uint32_t)cmd << 8);   /* cmd_hi, cmd_lo, 0x00 */
    spi_word(dev, 0);
    spi_word(dev, 0);
    spi_word(dev, 0);
    spi_word(dev, 0);
    spi_word(dev, 0);
    cs_high(dev);
}

/*
 * Write one 16-bit register.
 * Frame layout (6 words):
 *   TX: [WREG cmd][reg value][dummy x4]
 *   RX: ignored
 */
static void write_reg(AMC130M03_t *dev, uint8_t addr, uint16_t value)
{
    cs_low(dev);
    spi_word(dev, (uint32_t)AMC130_CMD_WREG(addr, 1) << 8);
    spi_word(dev, (uint32_t)value << 8);
    spi_word(dev, 0);
    spi_word(dev, 0);
    spi_word(dev, 0);
    spi_word(dev, 0);
    cs_high(dev);
    HAL_Delay(1);
}

/*
 * Read one 16-bit register.
 *
 * The AMC130M03 SPI interface is pipelined: the response to a command
 * arrives in the *following* frame, not the current one.
 *
 * Frame 1 TX: [RREG cmd][dummy x5]   — issues the read request
 * Frame 2 TX: [NULL     ][dummy x5]   — clocks out the response
 * Frame 2 RX: [cmd echo ][reg data][...]
 *
 * The register value is in word[1] of frame 2 (upper 16 bits of the 24-bit word).
 */
static uint16_t read_reg(AMC130M03_t *dev, uint8_t addr)
{
    /* Frame 1: send RREG */
    cs_low(dev);
    spi_word(dev, (uint32_t)AMC130_CMD_RREG(addr, 1) << 8);
    spi_word(dev, 0);
    spi_word(dev, 0);
    spi_word(dev, 0);
    spi_word(dev, 0);
    spi_word(dev, 0);
    cs_high(dev);

    /* Frame 2: send NULL, read response */
    uint32_t w[6];
    cs_low(dev);
    for (int i = 0; i < 6; i++) {
        w[i] = spi_word(dev, 0);
    }
    cs_high(dev);

    /* Register data is in word[1], upper 16 bits of the 24-bit word */
    return (uint16_t)(w[1] >> 8);
}

/*
 * Sign-extend a 24-bit two's-complement value to int32_t.
 */
static int32_t sign_extend_24(uint32_t raw)
{
    if (raw & 0x800000u) {
        return (int32_t)(raw | 0xFF000000u);
    }
    return (int32_t)raw;
}


/* ============================================================
 *  Public API
 * ============================================================ */

bool amc130m03_init(AMC130M03_t *dev,
                    SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin)
{
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    memset(dev->raw, 0, sizeof(dev->raw));

    /* LSB size in µV for gain=1: (VREF / 2^23) * 1e6 */
    float lsb_uv = (AMC130_VREF_V / (float)AMC130_FULL_SCALE) * 1e6f;
    dev->lsb_uv[0] = lsb_uv;
    dev->lsb_uv[1] = lsb_uv;
    dev->lsb_uv[2] = lsb_uv;

    /* Deassert CS and wait for power-on reset (250 µs max) */
    cs_high(dev);
    HAL_Delay(2);

    /* Send RESET and wait for it to complete */
    send_command(dev, AMC130_CMD_RESET);
    HAL_Delay(2);

    /* Flush the SPI pipeline with one dummy frame */
    send_command(dev, AMC130_CMD_NULL);

    /* Verify the device ID register (expected: 0x2200) */
    uint16_t id = read_reg(dev, AMC130_REG_ID);
    if (id != AMC130_ID_EXPECTED) {
        return false;
    }

    /* Configure CLOCK: all 3 channels on, OSR=256, high-resolution mode */
    write_reg(dev, AMC130_REG_CLOCK,
              AMC130_CLOCK_ALL_CH | AMC130_CLOCK_OSR_256 | AMC130_CLOCK_HIGH_RES);

    /* Configure GAIN: gain=1 on all channels */
    write_reg(dev, AMC130_REG_GAIN,
              AMC130_GAIN_REG(AMC130_GAIN_1, AMC130_GAIN_1, AMC130_GAIN_1));

    return true;
}

bool amc130m03_read(AMC130M03_t *dev,
                    GPIO_TypeDef *drdy_port,
                    uint16_t drdy_pin)
{
    /* DRDY is active-low; bail out if no new data */
    if (HAL_GPIO_ReadPin(drdy_port, drdy_pin) != GPIO_PIN_RESET) {
        return false;
    }

    /*
     * Send a NULL command frame and capture the conversion results.
     *
     * RX word layout:
     *   w[0] = STATUS word
     *   w[1] = CH0 (24-bit two's complement)
     *   w[2] = CH1
     *   w[3] = CH2
     *   w[4] = CRC  (ignored)
     *   w[5] = padding
     */
    uint32_t w[6];
    cs_low(dev);
    w[0] = spi_word(dev, (uint32_t)AMC130_CMD_NULL << 8);
    w[1] = spi_word(dev, 0);
    w[2] = spi_word(dev, 0);
    w[3] = spi_word(dev, 0);
    w[4] = spi_word(dev, 0);
    w[5] = spi_word(dev, 0);
    cs_high(dev);

    dev->raw[0] = sign_extend_24(w[1]);
    dev->raw[1] = sign_extend_24(w[2]);
    dev->raw[2] = sign_extend_24(w[3]);

    return true;
}

float amc130m03_get_mv(const AMC130M03_t *dev, uint8_t channel)
{
    if (channel > 2) return 0.0f;
    return (float)dev->raw[channel] * dev->lsb_uv[channel] / 1000.0f;
}