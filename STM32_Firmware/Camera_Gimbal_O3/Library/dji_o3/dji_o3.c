#include "dji_o3.h"

// MSP constants
#define MSP_HEADER_1 '$'
#define MSP_HEADER_2 'M'
#define MSP_HEADER_3 '<'

#define MSP_API_VERSION 1
#define MSP_STATUS      101
#define MSP_SET_ARMING  214

#define DJI_O3_SEND_INTERVAL_MS 500
#define DJI_O3_BOOT_DELAY_MS    3000

static void msp_send(UART_HandleTypeDef *huart, uint8_t cmd, uint8_t *payload, uint8_t size)
{
    uint8_t buf[32];
    uint8_t i = 0;
    uint8_t checksum = 0;

    buf[i++] = MSP_HEADER_1;
    buf[i++] = MSP_HEADER_2;
    buf[i++] = MSP_HEADER_3;

    buf[i++] = size;
    checksum ^= size;

    buf[i++] = cmd;
    checksum ^= cmd;

    for (uint8_t j = 0; j < size; j++) {
        buf[i++] = payload[j];
        checksum ^= payload[j];
    }

    buf[i++] = checksum;

    HAL_UART_Transmit(huart, buf, i, 10);
}

void DJI_O3_Init(DJI_O3_Handle_t *dev, UART_HandleTypeDef *huart)
{
    dev->huart = huart;
    dev->last_send = HAL_GetTick();
    dev->initialized = 0;
}

void DJI_O3_Update(DJI_O3_Handle_t *dev, bool armed)
{
    uint32_t now = HAL_GetTick();

    // Wait for O3 boot
    if (!dev->initialized) {
        if (now < DJI_O3_BOOT_DELAY_MS)
            return;

        // Send initial handshake once
        msp_send(dev->huart, MSP_API_VERSION, NULL, 0);
        osDelay(50);
        msp_send(dev->huart, MSP_STATUS, NULL, 0);

        dev->initialized = 1;
        dev->last_send = now;
        return;
    }

    // Send periodically (~0.5s)
    if ((now - dev->last_send) < DJI_O3_SEND_INTERVAL_MS)
        return;

    dev->last_send = now;

    // Arm/disarm packet
    uint8_t payload[1];
    payload[0] = armed ? 1 : 0;

    msp_send(dev->huart, MSP_SET_ARMING, payload, 1);
}