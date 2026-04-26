/**
 * @file    amc130m03.h
 * @brief   AMC130M03 3-channel ADC driver for STM32G4
 *
 * Interrupt-driven driver using DRDY (Data Ready) pin to trigger
 * SPI reads via EXTI. Ported from MicroPython/RP2040 reference code.
 *
 * Hardware connections (adjust pin defines to match your board):
 *   MISO  -> SPI MISO
 *   MOSI  -> SPI MOSI
 *   SCK   -> SPI SCK
 *   CS    -> GPIO (active low)
 *   DRDY  -> EXTI (falling edge)
 *   RESET -> GPIO (active low)
 *   CLKIN -> TIM PWM output, 8 MHz
 */

#ifndef AMC130M03_H
#define AMC130M03_H

#include <stdbool.h>
#include "stm32g4xx_hal.h"

/* -----------------------------------------------------------------------
 * Register map
 * --------------------------------------------------------------------- */
#define AMC130M03_REG_ID          0x00
#define AMC130M03_REG_STATUS      0x01
#define AMC130M03_REG_MODE        0x02
#define AMC130M03_REG_CLOCK       0x03
#define AMC130M03_REG_GAIN        0x04
#define AMC130M03_REG_CFG         0x06
#define AMC130M03_REG_DCDC_CTRL   0x31

/* STATUS register bits */
#define AMC130M03_STATUS_DRDY_ALL  (0x07 << 0)   /* CH0-2 data ready  */
#define AMC130M03_STATUS_DCDC_ERR  (1 << 6)       /* DCDC error flag   */

/* CLOCK register – OSR field default 0x0 = 128, CLKIN required for DCDC */
#define AMC130M03_CLOCK_CLKIN_EN   (1 << 3)

/* DCDC_CTRL register */
#define AMC130M03_DCDC_CTRL_EN     (1 << 0)

/* -----------------------------------------------------------------------
 * Frame format
 *
 * The device uses 24-bit SPI words by default:
 *   [23:8]  = 16-bit register / ADC data
 *   [7:0]   = 0 padding
 *
 * One frame = 5 words × 3 bytes = 15 bytes (STATUS + CH0 + CH1 + CH2 + CRC)
 * --------------------------------------------------------------------- */
#define AMC130M03_FRAME_BYTES     15   /* 5 × 24-bit words            */

/* Byte offsets of 16-bit ADC results inside a read frame               */
#define AMC130M03_CH0_BYTE_HI     3
#define AMC130M03_CH1_BYTE_HI     6
#define AMC130M03_CH2_BYTE_HI     9

/* -----------------------------------------------------------------------
 * Voltage reference
 * --------------------------------------------------------------------- */
#define AMC130M03_VREF_V          1.2f   /* Internal reference, volts  */
#define AMC130M03_FULL_SCALE      32768  /* 2^15 for signed 16-bit     */

/* -----------------------------------------------------------------------
 * Public data structure
 * --------------------------------------------------------------------- */

/** Raw 16-bit ADC counts (signed, two's complement) */
typedef struct {
    int16_t ch[3];          /**< Channel 0 / 1 / 2 raw counts          */
    bool    valid;          /**< Set true after first successful read   */
    uint32_t timestamp_ms;  /**< HAL_GetTick() at time of capture       */
} AMC130M03_Data_t;

/** Driver handle – fill in before calling AMC130M03_Init() */
typedef struct {
    /* Peripherals (provided by application) */
    SPI_HandleTypeDef *hspi;    /**< SPI handle, CPOL=0 CPHA=1, 8-bit  */
    GPIO_TypeDef      *cs_port; /**< Chip-select GPIO port              */
    uint16_t           cs_pin;  /**< Chip-select GPIO pin               */
    GPIO_TypeDef      *rst_port;/**< RESET GPIO port  (active low)      */
    uint16_t           rst_pin; /**< RESET GPIO pin                     */

    /* Latest converted data – written by ISR, read by application      */
    volatile AMC130M03_Data_t latest;

    /* Internal state */
    volatile uint32_t  irq_count;   /**< Increments on every DRDY edge */
    volatile bool      busy;        /**< SPI transaction in progress    */
    uint8_t            tx_buf[AMC130M03_FRAME_BYTES];
    uint8_t            rx_buf[AMC130M03_FRAME_BYTES];
} AMC130M03_Handle_t;

/* -----------------------------------------------------------------------
 * Return codes
 * --------------------------------------------------------------------- */
typedef enum {
    AMC130M03_OK       =  0,
    AMC130M03_ERR_SPI  = -1,
    AMC130M03_ERR_ARG  = -2,
    AMC130M03_ERR_BUSY = -3,
} AMC130M03_Status_t;

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

/**
 * @brief  Initialise the driver, reset the device and enable the DCDC.
 *
 * Call once after configuring SPI, GPIO and EXTI in CubeMX / HAL.
 * The DRDY EXTI line must be configured for falling-edge detection but
 * NOT yet enabled – this function enables it after device startup.
 *
 * @param  hdev   Pointer to an AMC130M03_Handle_t filled by the caller.
 * @return AMC130M03_OK on success, negative on error.
 */
AMC130M03_Status_t AMC130M03_Init(AMC130M03_Handle_t *hdev);

/**
 * @brief  Read any 16-bit configuration register.
 *
 * Blocking, polled – do not call from the DRDY ISR.
 *
 * @param  hdev      Driver handle.
 * @param  reg_addr  Register address (0 – 63).
 * @param  out       Receives the 16-bit register value.
 * @return AMC130M03_OK, AMC130M03_ERR_ARG, or AMC130M03_ERR_SPI.
 */
AMC130M03_Status_t AMC130M03_ReadReg(AMC130M03_Handle_t *hdev,
                                     uint8_t             reg_addr,
                                     uint16_t           *out);

/**
 * @brief  Write any 16-bit configuration register.
 *
 * Blocking, polled – do not call from the DRDY ISR.
 *
 * @param  hdev       Driver handle.
 * @param  reg_addr   Register address (0 – 63).
 * @param  reg_value  16-bit value to write.
 * @return AMC130M03_OK, AMC130M03_ERR_ARG, or AMC130M03_ERR_SPI.
 */
AMC130M03_Status_t AMC130M03_WriteReg(AMC130M03_Handle_t *hdev,
                                      uint8_t             reg_addr,
                                      uint16_t            reg_value);

/**
 * @brief  Read all three ADC channels (blocking, polled).
 *
 * Useful for one-shot reads without interrupt-driven operation.
 *
 * @param  hdev   Driver handle.
 * @param  data   Receives the three channel values.
 * @return AMC130M03_OK or AMC130M03_ERR_SPI.
 */
AMC130M03_Status_t AMC130M03_ReadChannels(AMC130M03_Handle_t *hdev,
                                          AMC130M03_Data_t   *data);

/**
 * @brief  Convert a raw 16-bit count to volts.
 *
 * @param  raw_count  Signed 16-bit value from AMC130M03_Data_t.ch[].
 * @return Voltage in volts (±VREF range).
 */
float AMC130M03_CountsToVolts(int16_t raw_count);

/**
 * @brief  DRDY interrupt service callback.
 *
 * Call this from your HAL_GPIO_EXTI_Callback() for the DRDY pin.
 * It starts a non-blocking SPI DMA transfer; results are available in
 * hdev->latest after the SPI transfer-complete callback fires.
 *
 * Example in stm32g4xx_it.c / main.c:
 * @code
 *   void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 *   {
 *       if (GPIO_Pin == DRDY_PIN)
 *           AMC130M03_DRDY_Callback(&g_adc);
 *   }
 * @endcode
 *
 * @param  hdev  Driver handle.
 */
void AMC130M03_DRDY_Callback(AMC130M03_Handle_t *hdev);

/**
 * @brief  SPI TX/RX DMA complete callback.
 *
 * Call from HAL_SPI_TxRxCpltCallback() when the SPI handle matches.
 *
 * Example:
 * @code
 *   void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
 *   {
 *       if (hspi == g_adc.hspi)
 *           AMC130M03_SPI_CpltCallback(&g_adc);
 *   }
 * @endcode
 *
 * @param  hdev  Driver handle.
 */
void AMC130M03_SPI_CpltCallback(AMC130M03_Handle_t *hdev);

#ifdef __cplusplus
}
#endif
#endif /* AMC130M03_H */