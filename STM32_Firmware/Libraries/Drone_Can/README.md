# DroneCan Canbus library

Canbus library to implement the following functionality with stm32g4:\
1. Canbus to PWM ESC
2. Canbus to PWM Servo
3. Regular pitot tube to Canbus pitot tube
4. Voltage and current measurement over Canbus
<br>

## File structure
**UAVCAN_DSDL:** Scripts generate using a python file found in Libraries/DSDL_Generator which include all the message definitions for libcanard.

**Canard:** STM32 implementation of UAVCAN which provides API calls

**Can_node:** Custom library implementation of UAVCAN which includes stm32g4 canbus library support and implements device specific functionality.

**CMakeList.txt:** Cmake file to link libraries together

## To implement in your code you must include the following
<br>
Create a new folder and file

```
{your_code_home_directory}/Library/can_config.h
```

In that file copy the following code and comment out any functionality that isn't needed.

```C
#pragma once


//---------------------------------------------------------


// Include pitot tube library to allow function call
#define USE_PITOT


//---------------------------------------------------------


// Include battery library to allow function call
#define USE_BATTERY


//---------------------------------------------------------

// Include ESC library and set it to update when new information is sent
#define USE_ESC

// Define number of connected esc's
#define ESC_COUNT 2

// Set a different value per device so they don't read from same esc channel:
//   Device 0: ESC_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: ESC_CHANNEL_OFFSET 2  → channels 2, 3
#define ESC_CHANNEL_OFFSET  0


//---------------------------------------------------------


// Include servo library set it to update when new information is sent
#define USE_SERVO

// Define number of connected servos
#define SERVO_COUNT 2

// Set a different value per device so they don't read from same servo channel:
// EX: if servo_count = 2
//   Device 0: SERVO_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: SERVO_CHANNEL_OFFSET 2  → channels 2, 3
#define SERVO_CHANNEL_OFFSET  0
```
<br>
Then in your codes cmake file include the following at the end of the file: 

``` CMAKE
# Canbus library include

# Include library as subdirectory
# Set the build location to folder where can_config is stored
add_subdirectory(
    ../Libraries/Drone_Can
    ${CMAKE_BINARY_DIR}/Library
)

# Allow library access to hal libraries
target_link_libraries(Drone_Can PUBLIC
    stm32cubemx
)

# Give the LIBRARY the path to can_config.h
target_include_directories(Drone_Can PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Library

)

```
<br>
In the pre-existing function within the same CMakeList.txt file:

``` cmake
# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)
```
Add the Drone_Can library to be able to access it
``` cmake
# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
    Drone_Can 
)
```
<br>

Now include the library header file in your main.h file
```c 
#include <drone_can.h>
```
<br>

In the main.c main(void) function add the following before the main loop:
```c
  // Setup canbus interupt
  HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // Initialize dronecan
  can_node_init(&hfdcan1);

  // Start Canbus
  HAL_FDCAN_Start(&hfdcan1);
```
<br>

The library is meant to be used with freertos here is an example which implements the bare minimum required to function. One mutex is needed to protect the dronecan canbus implementation which isn't thread safe.

``` c
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

// Freertos functions
// Must be configured in stm32 cube mx

// Sends canbus heatbeat every 1 second
void StartCanStatus(void *argument)
{
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
}

// Transmit data over canbus
// Starts by aquiring a dynamic node allocation
void StartCanRx(void *argument)
{
  (void) argument;

  // Getting node allocated
  uint8_t can_id_status = 0;

  // Turn red led on to indicate wait in progress
  HAL_GPIO_WritePin(TMP_RED_LED_GPIO_Port, TMP_RED_LED_Pin, GPIO_PIN_SET);

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
  HAL_GPIO_WritePin(TMP_RED_LED_GPIO_Port, TMP_RED_LED_Pin, GPIO_PIN_RESET);


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
}

void StartCanTx(void *argument)
{
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
}

```