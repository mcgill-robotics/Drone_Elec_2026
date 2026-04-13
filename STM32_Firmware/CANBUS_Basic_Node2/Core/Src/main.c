/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_esc.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "projdefs.h"
#include "stm32_hal_legacy.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim4;

/* Definitions for CanStatus */
osThreadId_t CanStatusHandle;
const osThreadAttr_t CanStatus_attributes = {
  .name = "CanStatus",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for CanRx */
osThreadId_t CanRxHandle;
const osThreadAttr_t CanRx_attributes = {
  .name = "CanRx",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for CanTx */
osThreadId_t CanTxHandle;
const osThreadAttr_t CanTx_attributes = {
  .name = "CanTx",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};
/* Definitions for EscStatus */
osThreadId_t EscStatusHandle;
const osThreadAttr_t EscStatus_attributes = {
  .name = "EscStatus",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for LedBlink */
osThreadId_t LedBlinkHandle;
const osThreadAttr_t LedBlink_attributes = {
  .name = "LedBlink",
  .priority = (osPriority_t) osPriorityLow7,
  .stack_size = 128 * 4
};
/* Definitions for CanardlibMutex */
osMutexId_t CanardlibMutexHandle;
const osMutexAttr_t CanardlibMutex_attributes = {
  .name = "CanardlibMutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM4_Init(void);
void StartCanStatus(void *argument);
void StartCanRx(void *argument);
void StartCanTx(void *argument);
void StartEscStatus(void *argument);
void StartLedBlink(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Convert can esc output to pwm
uint16_t esc_raw_to_pwm(int16_t raw)
{
    // raw is [-8192 .. 8191], map to [ESC_PWM_MIN_US .. ESC_PWM_MAX_US]
    // Clamp to [0, 8191] for unidirectional ESCs (most drone ESCs)
    if (raw < 0) raw = -raw;
    uint32_t pwm = ESC_PWM_MIN_US + ((uint32_t)raw * (ESC_PWM_MAX_US - ESC_PWM_MIN_US)) / 8191;
    return (uint16_t)pwm;
}

// Called whenever new ESC commands are received.
void esc_set_output(uint8_t esc_index, int16_t raw_value)
{
    uint16_t ccr = esc_raw_to_pwm(raw_value) * 60714 / 2500;

    if (esc_index == 0){
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
    }

    if (esc_index == 1){
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr);
    }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Setup canbus interupt
  HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // Initialize dronecan
  can_node_init(&hfdcan1);

  // Start Canbus
  HAL_FDCAN_Start(&hfdcan1);


  // Commented out for testing on a specific pcb
  // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of CanardlibMutex */
  CanardlibMutexHandle = osMutexNew(&CanardlibMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CanStatus */
  CanStatusHandle = osThreadNew(StartCanStatus, NULL, &CanStatus_attributes);

  /* creation of CanRx */
  CanRxHandle = osThreadNew(StartCanRx, NULL, &CanRx_attributes);

  /* creation of CanTx */
  CanTxHandle = osThreadNew(StartCanTx, NULL, &CanTx_attributes);

  /* creation of EscStatus */
  EscStatusHandle = osThreadNew(StartEscStatus, NULL, &EscStatus_attributes);

  /* creation of LedBlink */
  LedBlinkHandle = osThreadNew(StartLedBlink, NULL, &LedBlink_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Initialize can
  // can_node_init(&hfdcan1);

  // Initialize motor pwm
  
  while (1)
  {
    // 10hz task 
    // send_esc_status();
  
    // 1hz task
    // can_node_1hz_tasks();

    // 1hz task blink led
    // To implement

    // Can handling
    // can_node_update();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 17;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 6;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60713;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12|TMP_GRN_LED_Pin|TMP_RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB11 PB12 TMP_GRN_LED_Pin TMP_RED_LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|TMP_GRN_LED_Pin|TMP_RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Canbus hardware interupt
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags){
  (void) flags;

  // Run library function to clear can buffer to ring buffer
  can_node_rx_isr(hfdcan);

  // Notify rx task to run
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(CanRxHandle, &woken);
  portYIELD_FROM_ISR(woken);
}

// Now handle FreeRTOS functions (which get called by the FDCAN_interupt)

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCanStatus */
/**
  * @brief  Function implementing the CanStatus thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCanStatus */
void StartCanStatus(void *argument)
{
  /* USER CODE BEGIN 5 */
  (void) argument;
  uint32_t ticks = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  { 
    // Delay must be at start
    ticks += 1000U;
    osDelayUntil(ticks);

    // Call mutex
    if (osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK)
    {
      can_node_1hz_tasks();

      osMutexRelease(CanardlibMutexHandle);
    }

      // Start can tx task
      xTaskNotifyGive(CanTxHandle); 
 
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCanRx */
/**
* @brief Function implementing the CanRx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanRx */
void StartCanRx(void *argument)
{
  /* USER CODE BEGIN StartCanRx */
  (void) argument;

  // Variable for tracking if node id aquired
  uint8_t can_id_status = 0;
  // Make LED RED
  HAL_GPIO_WritePin(TMP_RED_LED_GPIO_Port, TMP_RED_LED_Pin, GPIO_PIN_SET);
  while(can_id_status != 1) {

    // Thread safe delay, precision not required
    osDelay(20); 

    if(osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK) 
    {
      // Process incoming messages 
      while(can_node_dequeue_and_process()) {} 

      // Try to get dynamic node, return 2 to send, 1 if recieved and 0 if not
      can_id_status = can_node_poll_dna();
    
      if (can_id_status == 2)
      {
        xTaskNotifyGive(CanTxHandle);
      }

      osMutexRelease(CanardlibMutexHandle); 

    }
    
  }
  
  // Turn off red led - Node aquired
  HAL_GPIO_WritePin(TMP_RED_LED_GPIO_Port, TMP_RED_LED_Pin, GPIO_PIN_RESET);


  /* Infinite loop */
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK)
    {
      // Process canbus rx frames
      while(can_node_dequeue_and_process()) {}
      osMutexRelease(CanardlibMutexHandle);
    }

      // Start can tx task
      xTaskNotifyGive(CanTxHandle);

  }
  /* USER CODE END StartCanRx */
}

/* USER CODE BEGIN Header_StartCanTx */
/**
* @brief Function implementing the CanTx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTx */
void StartCanTx(void *argument)
{
  /* USER CODE BEGIN StartCanTx */
  /* Infinite loop */
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK)
    {
      can_node_flush_tx();
      osMutexRelease(CanardlibMutexHandle);
    }
  }
  /* USER CODE END StartCanTx */
}

/* USER CODE BEGIN Header_StartEscStatus */
/**
* @brief Function implementing the EscStatus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEscStatus */
void StartEscStatus(void *argument)
{
  /* USER CODE BEGIN StartEscStatus */
  (void) argument;
  uint32_t ticks = osKernelGetTickCount();
  
  /* Infinite loop */
  for(;;)
  { 
    // Must be at start to avoid mutexacquire delay
    ticks += 100U;
    osDelayUntil(ticks);

    // Call mutex
    if (osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK)
    {
      send_esc_status();   

      osMutexRelease(CanardlibMutexHandle);
    }

      // Start can tx task
      xTaskNotifyGive(CanTxHandle); 
    
  }
  /* USER CODE END StartEscStatus */
}

/* USER CODE BEGIN Header_StartLedBlink */
/**
* @brief Function implementing the LedBlink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedBlink */
void StartLedBlink(void *argument)
{
  /* USER CODE BEGIN StartLedBlink */
  /* Infinite loop */
  for(;;)
  {
    // Blink heartbeat led, using osDelay as precision not needed
    HAL_GPIO_WritePin(TMP_GRN_LED_GPIO_Port, TMP_GRN_LED_Pin, GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(TMP_GRN_LED_GPIO_Port, TMP_GRN_LED_Pin, GPIO_PIN_RESET);
    osDelay(800);
  }
  /* USER CODE END StartLedBlink */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
