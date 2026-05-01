#pragma once

// Generic library include
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdbool.h>

// Include configuration library
// Must be written in each function which implements the library
// See readme for example
#include <can_config.h>


// Private file include
#include <canard.h>
#include <dronecan_msgs.h>
#include <can_arm.h>

// If configured include esc library
#ifdef USE_ESC
    #include <can_esc.h>
#endif

// If configured include servo library
#ifdef USE_SERVO
    #include <can_servo.h>
#endif


// --- Init / poll ---
void    can_node_init(FDCAN_HandleTypeDef *hfdcan);

// Call from your RX task after taking the canard mutex.
// Feeds one raw frame into libcanard. Returns 0 if queue empty.
uint8_t    can_node_dequeue_and_process(void);

// Call from your 1 Hz task (under mutex).
void    can_node_1hz_tasks(void);

// Call from your DNA poller task (under mutex). 
// Returns 1 once node ID assigned, return 2 when transmission required.
int8_t  can_node_poll_dna(void);

// Call from your TX task (under mutex).
void    can_node_flush_tx(void);

// Drain can buffer to ring buffer
void can_node_rx_isr(FDCAN_HandleTypeDef *hfdcan);

//canbus broadcast function to use libcanard externally
int16_t can_node_broadcast(uint64_t data_type_signature,
                        uint16_t data_type_id,
                        uint8_t *inout_transfer_id,
                        uint8_t priority,
                        const void *payload,
                        uint16_t payload_len);


// Timer access to view time since last servo/motor update
uint32_t millis32(void);
