#pragma once

// Generic library include
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdbool.h>

// Private file include
#include <canard.h>
#include <dronecan_msgs.h>
#include <can_esc.h>

// Define functions for different functionality
#define USE_ESC;
//#define USE_SERVO;

// --- Init / poll ---
void    can_node_init(FDCAN_HandleTypeDef *hfdcan);

// Call from your RX task after taking the canard mutex.
// Feeds one raw frame into libcanard. Returns false if queue empty.
bool    can_node_dequeue_and_process(void);

// Call from your 1 Hz task (under mutex).
void    can_node_1hz_tasks(void);

// Call from your DNA poller task (under mutex). Returns 1 once node ID assigned.
int8_t  can_node_poll_dna(void);

// Call from your TX task (under mutex).
void    can_node_flush_tx(void);

// --- ISR: call from HAL_FDCAN_RxFifo0Callback ---
// Returns pdTRUE if a higher-priority task was woken (pass to portYIELD_FROM_ISR).
BaseType_t can_node_rx_isr(FDCAN_HandleTypeDef *hfdcan);

// --- Hooks for main.c to wire up RTOS signalling ---
// Register a function to call from ISR when frames arrive.
// main.c passes a wrapper that does vTaskNotifyGiveFromISR.
typedef void (*can_isr_notify_fn)(BaseType_t *pxHigherPriorityTaskWoken);
void can_node_set_rx_notify(can_isr_notify_fn fn);

// Register a function to call when the library queues TX frames.
// main.c passes a wrapper that does xSemaphoreGiveFromISR / xSemaphoreGive.
typedef void (*can_tx_ready_fn)(void);
void can_node_set_tx_ready(can_tx_ready_fn fn);