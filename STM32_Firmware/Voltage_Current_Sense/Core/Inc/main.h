/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drone_can.h"
#include "AMC130M03.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS1_Pin GPIO_PIN_11
#define CS1_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_12
#define CS2_GPIO_Port GPIOB
#define DRDY_Pin GPIO_PIN_13
#define DRDY_GPIO_Port GPIOB
#define DRDY_EXTI_IRQn EXTI15_10_IRQn
#define RST_Pin GPIO_PIN_14
#define RST_GPIO_Port GPIOB
#define GRN_LED_Pin GPIO_PIN_3
#define GRN_LED_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_4
#define RED_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Voltage divider resistor values
#define vdiv_r1 604
#define vdiv_r2 10

// Number of voltage/current reads till send over px4
#define voltage_read 5

// Mapping adc values to current
#define adc_max 4096.0f
#define current_sense_min -9.2f
#define current_sense_max 115.0f

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
