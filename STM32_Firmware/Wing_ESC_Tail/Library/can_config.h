#pragma once


//---------------------------------------------------------


// Include pitot tube library to allow function call
//#define USE_PITOT


//---------------------------------------------------------


// Include battery library to allow function call
//#define USE_BATTERY


//---------------------------------------------------------

// Include ESC library and set it to update when new information is sent
#define USE_ESC

// Define number of connected esc's
#define ESC_COUNT 2

// Set a different value per device so they don't read from same esc channel:
//   Device 0: ESC_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: ESC_CHANNEL_OFFSET 2  → channels 2, 3
#define ESC_CHANNEL_OFFSET  2


//---------------------------------------------------------


// Include servo library set it to update when new information is sent
#define USE_SERVO

// Define number of connected servos
#define SERVO_COUNT 3

// Set a different value per device so they don't read from same servo channel:
// EX: if servo_count = 2
//   Device 0: SERVO_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: SERVO_CHANNEL_OFFSET 2  → channels 2, 3
#define SERVO_CHANNEL_OFFSET  3