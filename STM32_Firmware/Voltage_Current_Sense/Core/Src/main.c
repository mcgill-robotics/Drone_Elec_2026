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

// Error variable
volatile uint8_t error = 0;

// Current measurement adc buffer
volatile uint16_t current_raw[3] = {0};

// Struct for averaging power data
typedef struct {
  float data;
  uint16_t count;
} get_average;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;

/* Definitions for LedBlink */
osThreadId_t LedBlinkHandle;
const osThreadAttr_t LedBlink_attributes = {
  .name = "LedBlink",
  .priority = (osPriority_t) osPriorityLow6,
  .stack_size = 128 * 4
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
/* Definitions for CanStatus */
osThreadId_t CanStatusHandle;
const osThreadAttr_t CanStatus_attributes = {
  .name = "CanStatus",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for MeasurePower */
osThreadId_t MeasurePowerHandle;
const osThreadAttr_t MeasurePower_attributes = {
  .name = "MeasurePower",
  .priority = (osPriority_t) osPriorityNormal,
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
static AMC130M03_Handle_t g_adc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
void StartLedBlink(void *argument);
void StartCanRx(void *argument);
void StartCanTx(void *argument);
void StartCanStatus(void *argument);
void StartMeasurePower(void *argument);
void StartErrorLED(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Initializes ADC dma
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)current_raw, 3);

  // Initializes External ADC library
  g_adc.hspi     = &hspi1;
  g_adc.cs_port  = CS1_GPIO_Port;
  g_adc.cs_pin   = CS1_Pin;
  g_adc.rst_port = RST_GPIO_Port;
  g_adc.rst_pin  = RST_Pin;
  AMC130M03_Status_t ret = AMC130M03_Init(&g_adc);
  if (ret != AMC130M03_OK)
  {
      /* Handle init failure – check CLKIN frequency and SPI wiring   */
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
      Error_Handler();
  }

  // Start PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    // Setup canbus interupt
  HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // Initialize dronecan
  can_node_init(&hfdcan1);

  // Start Canbus
  HAL_FDCAN_Start(&hfdcan1);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);

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
  /* creation of LedBlink */
  LedBlinkHandle = osThreadNew(StartLedBlink, NULL, &LedBlink_attributes);

  /* creation of CanRx */
  CanRxHandle = osThreadNew(StartCanRx, NULL, &CanRx_attributes);

  /* creation of CanTx */
  CanTxHandle = osThreadNew(StartCanTx, NULL, &CanTx_attributes);

  /* creation of CanStatus */
  CanStatusHandle = osThreadNew(StartCanStatus, NULL, &CanStatus_attributes);

  /* creation of MeasurePower */
  MeasurePowerHandle = osThreadNew(StartMeasurePower, NULL, &MeasurePower_attributes);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS2_Pin|RST_Pin|GRN_LED_Pin
                          |RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS1_Pin CS2_Pin RST_Pin GRN_LED_Pin
                           RED_LED_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS2_Pin|RST_Pin|GRN_LED_Pin
                          |RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Calculate voltage from adc
float inverse_voltage_divider(float voltage){
  return voltage * (float)(vdiv_r1 + vdiv_r2) / (float)vdiv_r2;
}

// Calculate current from adc
float calculate_current(float adc_read){
  return adc_read * (current_sense_max - current_sense_min) / adc_max + current_sense_min;
}

// DRDY interupt handler for voltage sensor
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DRDY_Pin)          /* DRDY_Pin defined by CubeMX    */
        AMC130M03_DRDY_Callback(&g_adc);
}
 
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == g_adc.hspi)
        AMC130M03_SPI_CpltCallback(&g_adc);
}

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

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLedBlink */
/**
  * @brief  Function implementing the LedBlink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLedBlink */
void StartLedBlink(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);
    osDelay(900);
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

/* USER CODE BEGIN Header_StartCanStatus */
/**
* @brief Function implementing the CanStatus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanStatus */
void StartCanStatus(void *argument)
{
  /* USER CODE BEGIN StartCanStatus */
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
  /* USER CODE END StartCanStatus */
}

/* USER CODE BEGIN Header_StartMeasurePower */
/**
* @brief Function implementing the MeasurePower thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMeasurePower */
void StartMeasurePower(void *argument)
{
  /* USER CODE BEGIN StartMeasurePower */

  get_average voltage = {0};
  get_average current = {0};
  uint8_t cycle_count = 0;

  /* Infinite loop */
  for(;;)
  {
    osDelay(10);

    uint16_t current_snap[3] = {0};

    // Enter an uniterupted task where voltage read and current
    // snapshots are taken from their DMA's
    taskENTER_CRITICAL();
    AMC130M03_Data_t snap = g_adc.latest;
    memcpy(current_snap, current_raw, sizeof(current_raw));
    taskEXIT_CRITICAL();

    // Parse voltage and add to averaging struct
    if (snap.valid)
    {
      voltage.data += AMC130M03_CountsToVolts(snap.ch[1]);
      voltage.count++;
    }
    
    // Parse current and add to averaging struct;
    for(int i = 0; i < (sizeof(current_snap)/sizeof(current_snap[0])); i++){
      current.data += (float)current_snap[i];
      current.count ++;
    }
    cycle_count++;

    // Every (voltage_read) number of cycles calculate voltage and current average
    // and send over dronecan
    if(cycle_count >= voltage_read) {
      float current_avg = 0;
      float voltage_avg = 0;
      if (current.count != 0) {
        current_avg = calculate_current(current.data / current.count);
        error &= ~(1u << 0);
      }
      else error |= (1u << 0);


      if (voltage.count != 0) {
        voltage_avg = inverse_voltage_divider(voltage.data / voltage.count);
        error &= ~(1u << 1);
      }
      else error |= (1u << 1);

      send_battery_info(voltage_avg, current_avg);
      current = (get_average){0};
      voltage = (get_average){0};
      cycle_count = 0;
    }
  }
  /* USER CODE END StartMeasurePower */
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
    osDelay(1000);
    if (error != 0) HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
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
