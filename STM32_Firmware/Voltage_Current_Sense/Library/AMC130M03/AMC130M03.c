#include "AMC130M03.h"

/**
 * @file    amc130m03.c
 * @brief   AMC130M03 3-channel ADC driver for STM32G4
 *
 * Implementation notes
 * --------------------
 * Frame structure (24-bit word mode, reset default):
 *
 *   Byte index  |  0   1   2  |  3   4   5  |  6   7   8  |  9  10  11  | 12  13  14 |
 *   Word        |   RESPONSE  |    CH0 data  |    CH1 data  |    CH2 data  |    CRC     |
 *
 * Each 24-bit word is [DATA_HI : DATA_LO : 0x00].
 * The RESPONSE word echoes back the previous command's status/address.
 *
 * Register read sequence (two frames):
 *   Frame 1 TX: [READ_CMD | addr_hi][addr_lo_shifted][0x00] + 4 × [0,0,0]
 *   Frame 2 TX: 5 × [0,0,0]   →  RX word 0 bytes [0:1] = register value
 *
 * Register write sequence (one frame):
 *   Frame 1 TX: [WRITE_CMD | addr_hi][addr_lo_shifted][0x00][value_hi][value_lo][0x00] + ...
 *
 * Data read sequence (one frame, issued in DRDY ISR):
 *   Frame TX: 5 × [0,0,0]   →  RX[3:4] = CH0, RX[6:7] = CH1, RX[9:10] = CH2
 */

#include "amc130m03.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* -----------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------- */

#define CS_LOW(h)   HAL_GPIO_WritePin((h)->cs_port, (h)->cs_pin,  GPIO_PIN_RESET)
#define CS_HIGH(h)  HAL_GPIO_WritePin((h)->cs_port, (h)->cs_pin,  GPIO_PIN_SET)
#define RST_LOW(h)  HAL_GPIO_WritePin((h)->rst_port, (h)->rst_pin, GPIO_PIN_RESET)
#define RST_HIGH(h) HAL_GPIO_WritePin((h)->rst_port, (h)->rst_pin, GPIO_PIN_SET)

/* SPI command bytes
 *   Read:  1010 0000 | (addr >> 1)   then  bit7 of next byte = addr & 1
 *   Write: 0110 0000 | (addr >> 1)   then  bit7 of next byte = addr & 1
 */
#define READ_CMD_HI(addr)  (0xA0U | ((addr) >> 1))
#define WRITE_CMD_HI(addr) (0x60U | ((addr) >> 1))
#define ADDR_LO_BYTE(addr) (((addr) & 0x01U) << 7)

static HAL_StatusTypeDef spi_txrx(AMC130M03_Handle_t *hdev,
                                   const uint8_t *tx, uint8_t *rx,
                                   uint16_t len)
{
    HAL_StatusTypeDef st;
    CS_LOW(hdev);
    st = HAL_SPI_TransmitReceive(hdev->hspi,
                                 (uint8_t *)tx, rx, len,
                                 100 /* ms timeout */);
    CS_HIGH(hdev);
    return st;
}

/* Parse three channel values out of a raw data frame */
static void parse_frame(const uint8_t *rx, AMC130M03_Data_t *data)
{
    /* CH0: bytes 3–4 (big-endian, 16-bit signed) */
    data->ch[0] = (int16_t)(((uint16_t)rx[AMC130M03_CH0_BYTE_HI] << 8) |
                             (uint16_t)rx[AMC130M03_CH0_BYTE_HI + 1]);

    /* CH1: bytes 6–7 */
    data->ch[1] = (int16_t)(((uint16_t)rx[AMC130M03_CH1_BYTE_HI] << 8) |
                             (uint16_t)rx[AMC130M03_CH1_BYTE_HI + 1]);

    /* CH2: bytes 9–10 */
    data->ch[2] = (int16_t)(((uint16_t)rx[AMC130M03_CH2_BYTE_HI] << 8) |
                             (uint16_t)rx[AMC130M03_CH2_BYTE_HI + 1]);

    data->valid        = true;
    data->timestamp_ms = HAL_GetTick();
}

/* -----------------------------------------------------------------------
 * Public API – register access (blocking / polled)
 * --------------------------------------------------------------------- */

AMC130M03_Status_t AMC130M03_ReadReg(AMC130M03_Handle_t *hdev,
                                     uint8_t             reg_addr,
                                     uint16_t           *out)
{
    if (reg_addr > 63 || out == NULL) return AMC130M03_ERR_ARG;

    /* ---- Frame 1: issue READ command --------------------------------- */
    uint8_t tx1[AMC130M03_FRAME_BYTES] = {0};
    uint8_t rx1[AMC130M03_FRAME_BYTES] = {0};

    tx1[0] = READ_CMD_HI(reg_addr);
    tx1[1] = ADDR_LO_BYTE(reg_addr);
    /* remaining bytes are 0 – padding words */

    if (spi_txrx(hdev, tx1, rx1, AMC130M03_FRAME_BYTES) != HAL_OK)
        return AMC130M03_ERR_SPI;

    /* ---- Frame 2: clock out zeros, device echoes register value ------ */
    uint8_t tx2[AMC130M03_FRAME_BYTES] = {0};
    uint8_t rx2[AMC130M03_FRAME_BYTES] = {0};

    if (spi_txrx(hdev, tx2, rx2, AMC130M03_FRAME_BYTES) != HAL_OK)
        return AMC130M03_ERR_SPI;

    /* Register value arrives in the first word of frame 2, bytes 0–1   */
    *out = ((uint16_t)rx2[0] << 8) | (uint16_t)rx2[1];
    return AMC130M03_OK;
}

AMC130M03_Status_t AMC130M03_WriteReg(AMC130M03_Handle_t *hdev,
                                      uint8_t             reg_addr,
                                      uint16_t            reg_value)
{
    if (reg_addr > 63) return AMC130M03_ERR_ARG;

    uint8_t tx[AMC130M03_FRAME_BYTES] = {0};
    uint8_t rx[AMC130M03_FRAME_BYTES] = {0};

    /* Word 0: WRITE command */
    tx[0] = WRITE_CMD_HI(reg_addr);
    tx[1] = ADDR_LO_BYTE(reg_addr);
    tx[2] = 0x00;

    /* Word 1: data to write */
    tx[3] = (uint8_t)(reg_value >> 8);
    tx[4] = (uint8_t)(reg_value & 0xFF);
    tx[5] = 0x00;

    /* Words 2–4: zeros (no-op) */

    if (spi_txrx(hdev, tx, rx, AMC130M03_FRAME_BYTES) != HAL_OK)
        return AMC130M03_ERR_SPI;

    return AMC130M03_OK;
}

AMC130M03_Status_t AMC130M03_ReadChannels(AMC130M03_Handle_t *hdev,
                                          AMC130M03_Data_t   *data)
{
    if (data == NULL) return AMC130M03_ERR_ARG;

    uint8_t tx[AMC130M03_FRAME_BYTES] = {0};
    uint8_t rx[AMC130M03_FRAME_BYTES] = {0};

    if (spi_txrx(hdev, tx, rx, AMC130M03_FRAME_BYTES) != HAL_OK)
        return AMC130M03_ERR_SPI;

    parse_frame(rx, data);
    return AMC130M03_OK;
}

float AMC130M03_CountsToVolts(int16_t raw_count)
{
    return (float)raw_count / (float)AMC130M03_FULL_SCALE * AMC130M03_VREF_V;
}

/* -----------------------------------------------------------------------
 * Initialisation
 * --------------------------------------------------------------------- */

AMC130M03_Status_t AMC130M03_Init(AMC130M03_Handle_t *hdev)
{
    AMC130M03_Status_t ret;
    uint16_t           reg_val;

    /* ---- 1. Hardware reset ------------------------------------------ */
    CS_HIGH(hdev);          /* de-assert CS before reset                */
    RST_HIGH(hdev);
    HAL_Delay(1);
    RST_LOW(hdev);          /* assert reset (active low)                */
    HAL_Delay(1);
    RST_HIGH(hdev);         /* release reset                            */
    HAL_Delay(2);           /* device needs ~1 ms after reset           */

    /* ---- 2. Verify device ID ---------------------------------------- */
    ret = AMC130M03_ReadReg(hdev, AMC130M03_REG_ID, &reg_val);
    if (ret != AMC130M03_OK) return ret;
    /* ID is informational only – different silicon revisions may differ */

    /* ---- 3. Check STATUS register ------------------------------------ */
    ret = AMC130M03_ReadReg(hdev, AMC130M03_REG_STATUS, &reg_val);
    if (ret != AMC130M03_OK) return ret;

    /* ---- 4. Enable internal DCDC converter --------------------------- */
    ret = AMC130M03_WriteReg(hdev, AMC130M03_REG_DCDC_CTRL,
                              AMC130M03_DCDC_CTRL_EN);
    if (ret != AMC130M03_OK) return ret;

    HAL_Delay(2); /* wait for DCDC to stabilise                        */

    /* ---- 5. Verify DCDC running (STATUS bit 6 should clear) ---------- */
    ret = AMC130M03_ReadReg(hdev, AMC130M03_REG_STATUS, &reg_val);
    if (ret != AMC130M03_OK) return ret;
    /*
     * Bit 6 = DCDC_ERR; if still set, the external CLKIN may not be
     * running.  Log / handle as needed for your application.
     */

    /* ---- 6. Zero out internal state ---------------------------------- */
    memset((void *)&hdev->latest, 0, sizeof(hdev->latest));
    hdev->irq_count = 0;
    hdev->busy      = false;
    memset(hdev->tx_buf, 0, sizeof(hdev->tx_buf));

    /*
     * The DRDY EXTI line should already be configured for falling-edge
     * in CubeMX.  Enable it now that the device is running.
     * The EXTI line number must match the GPIO pin of your DRDY signal.
     * Uncomment and adjust the line below if not enabling EXTI elsewhere:
     *
     *   HAL_NVIC_EnableIRQ(EXTIx_IRQn);
     */

    return AMC130M03_OK;
}

/* -----------------------------------------------------------------------
 * Interrupt-driven callbacks
 * --------------------------------------------------------------------- */

/**
 * Called from HAL_GPIO_EXTI_Callback() on the DRDY falling edge.
 *
 * Starts a non-blocking SPI DMA transfer.  The driver sets the CS low
 * here and restores it in AMC130M03_SPI_CpltCallback().
 *
 * If DMA is not configured, fall back to the polled path by calling
 * AMC130M03_ReadChannels() directly instead (but this blocks the ISR).
 */
void AMC130M03_DRDY_Callback(AMC130M03_Handle_t *hdev)
{
    hdev->irq_count++;

    /* Don't start a new transfer while one is still in progress */
    if (hdev->busy) return;

    /* tx_buf is pre-zeroed in Init – send all zeros for a data read */
    hdev->busy = true;
    CS_LOW(hdev);

    HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive_DMA(hdev->hspi,
                                    hdev->tx_buf,
                                    hdev->rx_buf,
                                    AMC130M03_FRAME_BYTES);

    if (st != HAL_OK)
    {
        /* DMA failed – release the bus and let the next DRDY retry */
        CS_HIGH(hdev);
        hdev->busy = false;
    }
}

/**
 * Called from HAL_SPI_TxRxCpltCallback() when the DMA transfer finishes.
 * Runs in interrupt context – keep it short.
 */
void AMC130M03_SPI_CpltCallback(AMC130M03_Handle_t *hdev)
{
    CS_HIGH(hdev);
    parse_frame(hdev->rx_buf, (AMC130M03_Data_t *)&hdev->latest);
    hdev->busy = false;
}