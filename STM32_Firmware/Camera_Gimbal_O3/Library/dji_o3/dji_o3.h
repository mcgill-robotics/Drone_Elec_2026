#pragma once

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include "cmsis_os.h"

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t last_send;
    uint8_t initialized;
} DJI_O3_Handle_t;

void DJI_O3_Init(DJI_O3_Handle_t *dev, UART_HandleTypeDef *huart);
void DJI_O3_Update(DJI_O3_Handle_t *dev, bool armed);