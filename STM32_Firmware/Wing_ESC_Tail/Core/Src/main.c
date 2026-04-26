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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint32_t servo_error = 0;
uint32_t esc_error = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
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
  .priority = (osPriority_t) osPriorityHigh3,
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
  .priority = (osPriority_t) osPriorityLow6,
  .stack_size = 128 * 4
};
/* Definitions for EscUpdate */
osThreadId_t EscUpdateHandle;
const osThreadAttr_t EscUpdate_attributes = {
  .name = "EscUpdate",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for ServoUpdate */
osThreadId_t ServoUpdateHandle;
const osThreadAttr_t ServoUpdate_attributes = {
  .name = "ServoUpdate",
  .priority = (osPriority_t) osPriorityAboveNormal4,
  .stack_size = 256 * 4
};
/* Definitions for ErrorLED */
osThreadId_t ErrorLEDHandle;
const osThreadAttr_t ErrorLED_attributes = {
  .name = "ErrorLED",
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
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void StartCanStatus(void *argument);
void StartCanRx(void *argument);
void StartCanTx(void *argument);
void StartEscStatus(void *argument);
void StartLedBlink(void *argument);
void StartEscUpdate(void *argument);
void StartServoUpdate(void *argument);
void StartErrorLED(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t servo_channel(int i){
  switch (i){
    case 0:
      return TIM_CHANNEL_1;
      break;
    case 1:
      return TIM_CHANNEL_2;
      break;
    case 2:
      return TIM_CHANNEL_1;
    default:
      esc_error |= (1u << 30);
      return TIM_CHANNEL_1;
  }
}
TIM_HandleTypeDef* servo_timer(int i){
  switch (i){
    case 0:
      return &htim3;
      break;
    case 1:
      return &htim2;
      break;
    case 2:
      return &htim2;
      break;
    default:
      esc_error |= (1u << 29);
      return &htim2;

  }
}
// Sets which PWM channel drives which esc
uint32_t esc_channel(int i){
  switch (i){
    case 0:
      return TIM_CHANNEL_1;
      break;
    case 1:
      return TIM_CHANNEL_2;
      break;
    default:
      esc_error |= (1u << 28);
      return TIM_CHANNEL_1;
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
 
  // Setup canbus interupt
  HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // Initialize dronecan
  can_node_init(&hfdcan1);

  // Start Canbus
  HAL_FDCAN_Start(&hfdcan1);

  // Start PWM channels
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

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

  /* creation of EscUpdate */
  EscUpdateHandle = osThreadNew(StartEscUpdate, NULL, &EscUpdate_attributes);

  /* creation of ServoUpdate */
  ServoUpdateHandle = osThreadNew(StartServoUpdate, NULL, &ServoUpdate_attributes);

  /* creation of ErrorLED */
  ErrorLEDHandle = osThreadNew(StartErrorLED, NULL, &ErrorLED_attributes);

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
  while (1)
  {
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 51;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65383;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOB, RED_LED_Pin|GRN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_LED_Pin GRN_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|GRN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Map function for canbus servo/esc command to PWM pulse
static inline int32_t map_int(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Canbus interupt
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags){
  (void) flags;

  // Run library function to clear can buffer to ring buffer
  can_node_rx_isr(hfdcan);

  // Notify rx task to run
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(CanRxHandle, &woken);
  portYIELD_FROM_ISR(woken);
}

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
  
  for(;;)
  { 
    // Wait 1 second
    ticks += 1000U;
    osDelayUntil(ticks);

    // Aquire canbus mutex
    if (osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK)
    {
      // Send update
      can_node_1hz_tasks();

      osMutexRelease(CanardlibMutexHandle);
    }
      // Notify canbus transmit task to send data
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

  // Getting node allocated
  uint8_t can_id_status = 0;

  // Turn red led on to indicate wait in progress
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);

  // Wait untill node allocated
  while(can_id_status != 1) {

    // Thread safe delay, precision not required
    osDelay(20); 

    // Get canbus mutex
    if(osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK) 
    {
      // Process incoming messages 
      while(can_node_dequeue_and_process()) {} 

      // Try to get dynamic node, returns 2 to signal data transmission required, 1 if recieved and 0 if else
      can_id_status = can_node_poll_dna();
      
      // Transmit data if required
      if (can_id_status == 2)
      {
        xTaskNotifyGive(CanTxHandle);
      }

      osMutexRelease(CanardlibMutexHandle); 
    } 
  }
  
  // Turn off red led - Node aquired
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /* Infinite loop */
  for(;;)
  {

    // Wait forever untill triggered by canbus interupt
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Aquire mutex
    if (osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK)
    {
      // Process canbus rx frames
      while(can_node_dequeue_and_process()) {}
      osMutexRelease(CanardlibMutexHandle);
    }
      // Notify canbus tx task to send data 
      // Done as processing data can require responding
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
  for(;;)
  {
    // Wait forever untill function is called
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Aquire canbus mutex
    if(osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK)
    {
      // Call drone can send data function
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
    // Send status at 10hz
    ticks += 100U;
    osDelayUntil(ticks);

    // Aquire mutex
    if (osMutexAcquire(CanardlibMutexHandle, osWaitForever) == osOK){
      // Send status
      send_esc_status();
      osMutexRelease(CanardlibMutexHandle);
    }
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
  (void) argument;
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);
    osDelay(900);
  }
  /* USER CODE END StartLedBlink */
}

/* USER CODE BEGIN Header_StartEscUpdate */
/**
* @brief Function implementing the EscUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEscUpdate */
void StartEscUpdate(void *argument)
{
  /* USER CODE BEGIN StartEscUpdate */
  (void) argument;
  uint32_t ticks = osKernelGetTickCount();
  update_tracking esc_status[ESC_COUNT] = {0};
  /* Infinite loop */
  for(;;)
  {
    // Update at 200hz
    ticks += 5;
    osDelayUntil(ticks);
    for(int i = 0; i < ESC_COUNT; i++){
      if (esc[i].last_update !=esc_status[i].last_update){
        esc_status[i].last_update = esc[i].last_update;
        esc_status[i].update_failed_count = 0;
        esc_status[i].update_without_fault ++;
        if (esc_status[i].update_without_fault > 200) esc_error &= ~(1u << i);
        if (esc[i].esc_cmd < 0) esc[i].esc_cmd = -esc[i].esc_cmd;
        uint32_t pulse = map_int(esc[i].esc_cmd, ESC_CAN_MIN, ESC_CAN_MAX, ESC_PULSE_MIN, ESC_PULSE_MAX);

        __HAL_TIM_SET_COMPARE(&htim4, esc_channel(i) , pulse);
        // Write esc command
      }
      else {
        esc_status[i].update_failed_count ++;
        esc_status[i].update_without_fault = 0;
        if (esc_status[i].update_failed_count >= ALLOWED_ESC_FAILS) {
          esc_error |= (1u << i);
          __HAL_TIM_SET_COMPARE(&htim4, esc_channel(i) , 0);
        }
      }
    }
  
  }
  /* USER CODE END StartEscUpdate */
}

/* USER CODE BEGIN Header_StartServoUpdate */
/**
* @brief Function implementing the ServoUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoUpdate */
void StartServoUpdate(void *argument)
{
  /* USER CODE BEGIN StartServoUpdate */
  (void) argument;
  uint32_t ticks = osKernelGetTickCount();
  update_tracking servo_status[SERVO_COUNT] = {0};
  /* Infinite loop */
  for(;;)
  {
    // Update at 50hz
    ticks += 20;
    osDelayUntil(ticks);
    for(int i = 0; i < SERVO_COUNT; i++){
      if (servos[i].last_update != servo_status[i].last_update){
        servo_status[i].last_update = servos[i].last_update;
        servo_status[i].update_failed_count = 0;
        servo_status[i].update_without_fault ++;
        if (servo_status[i].update_without_fault > 50) servo_error &= ~(1u << i);
        uint32_t pulse = map_int(servos[i].servo_cmd, SERVO_CAN_MIN, SERVO_CAN_MAX, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        __HAL_TIM_SET_COMPARE(servo_timer(i), servo_channel(i) , pulse);
      }
      else {
        servo_status[i].update_failed_count ++;
        servo_status[i].update_without_fault = 0;
        if (servo_status[i].update_failed_count >= ALLOWED_SERVO_FAILS) {
          servo_error |= (1u << i);
          __HAL_TIM_SET_COMPARE(&htim3, servo_channel(i) , 0);
        }
      }
    }
    

  }
  /* USER CODE END StartServoUpdate */
}

/* USER CODE BEGIN Header_StartErrorLED */
/**
* @brief Function implementing the ErrorLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartErrorLED */
void StartErrorLED(void *argument)
{
  /* USER CODE BEGIN StartErrorLED */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);

    if (esc_error != 0 || servo_error != 0){
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
    }
    else {
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END StartErrorLED */
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
