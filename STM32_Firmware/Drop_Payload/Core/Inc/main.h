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
#include <drone_can.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Alowed number of PWM write function without recieving pwm command
// till it enters a zero state
#define ALLOWED_SERVO_FAILS 5

#define SERVO_CENTER 4904 

// Servo Min input value to spin clockwise
#define SERVO_UPPER_LIMIT 1650

// Servo clockwise pulse (2ms pwm)
#define SERVO_SPIN_CLKWISE 6538

// Servo Min input value to spin clockwise
#define SERVO_LOWER_LIMIT 1350

// Servo clockwise pulse (1ms pwm)
#define SERVO_SPIN_CNTCLKWISE 3269
// Struct for tracking if esc/servo has updated recently
typedef struct{
  int32_t update_failed_count;
  uint32_t last_update;
  int32_t update_without_fault;
} update_tracking;


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
#define RED_LED_Pin GPIO_PIN_11
#define RED_LED_GPIO_Port GPIOB
#define GRN_LED_Pin GPIO_PIN_12
#define GRN_LED_GPIO_Port GPIOB
#define SERVO3_Pin GPIO_PIN_15
#define SERVO3_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_3
#define SERVO2_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_4
#define SERVO1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
