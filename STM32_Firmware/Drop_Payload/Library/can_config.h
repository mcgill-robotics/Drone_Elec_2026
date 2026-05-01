// Example dronecan configuration library for stm32
// To use the library include the drone_can.h library
// and implement a copy of this file renaming it "can_config.h"
// and comment out any functionality you don't need

/* 
Library supports the following dronecan devices
    - ESC           Import can_esc.h and use the automatically updating struct esc[]
    - Servo         Import can_servo.h and use automatically updating sturct servo[]
    - Battery       Import can_battery.h and call function send_battery_info() 
    - Pitot Tube    Import can_pitot.h and call function send_pitot_info()
*/


#pragma once

//----------------------------------------------------------------------------------


// Include pitot tube library to allow function call
//#define USE_PITOT


//----------------------------------------------------------------------------------


// Include battery library to allow function call
//#define USE_BATTERY


//----------------------------------------------------------------------------------


// Include ESC library and set it to update when new information is sent
// #define USE_ESC

// Define number of connected esc's
#define ESC_COUNT 2

// Set a different value per device so they don't read from same esc channel:
//   Device 0: ESC_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: ESC_CHANNEL_OFFSET 2  → channels 2, 3
//   Device 2: ESC_CHANNEL_OFFSET 4  → channels 4, 5
#define ESC_CHANNEL_OFFSET  2


//----------------------------------------------------------------------------------


// Include servo library set it to update when new information is sent
#define USE_SERVO

// Define number of connected servos
#define SERVO_COUNT 1

// Set a different value per device so they don't read from same servo channel:
// EX: if servo_count = 2
//   Device 0: SERVO_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: SERVO_CHANNEL_OFFSET 2  → channels 2, 3
//   Device 2: SERVO_CHANNEL_OFFSET 4  → channels 4, 5
#define SERVO_CHANNEL_OFFSET  5
 