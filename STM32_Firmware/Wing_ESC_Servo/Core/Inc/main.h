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
#define ALLOWED_ESC_FAILS 5

// Servo output pulse max and min for pwm timer map function
#define SERVO_PULSE_MAX 6538
#define SERVO_PULSE_MIN 3269


// Servo input from canbus min and max for map function
#define SERVO_CAN_MAX 2000
#define SERVO_CAN_MIN 1000

// ESC output pulse max and min for pwm timer map function
//#define ESC_PULSE_MAX 48570
//#define ESC_PULSE_MIN 24285
#define ESC_PULSE_MAX 6538   // 2000µs → full throttle
#define ESC_PULSE_MIN 3269   // 1000µs → disarmed/min

// ESC input from canbus min and max for map function
#define ESC_CAN_MAX 8191
#define ESC_CAN_MIN 0

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
#define ESC1_Pin GPIO_PIN_6
#define ESC1_GPIO_Port GPIOB
#define ESC2_Pin GPIO_PIN_7
#define ESC2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
