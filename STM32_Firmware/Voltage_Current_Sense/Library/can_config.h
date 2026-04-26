#pragma once


//---------------------------------------------------------

// Include ESC library and set it to update when new information is sent
//#define USE_ESC

// Define number of connected esc's
// Don't comment out
#define ESC_COUNT 2

// Set a different value per device so they don't read from same esc channel:
//   Device 0: ESC_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: ESC_CHANNEL_OFFSET 2  → channels 2, 3
// Don't comment out
#define ESC_CHANNEL_OFFSET  0


//---------------------------------------------------------


// Include servo library set it to update when new information is sent
//#define USE_SERVO

// Define number of connected servos
// Don't comment out
#define SERVO_COUNT 2

// Set a different value per device so they don't read from same servo channel:
// EX: if servo_count = 2
//   Device 0: SERVO_CHANNEL_OFFSET 0  → channels 0, 1
//   Device 1: SERVO_CHANNEL_OFFSET 2  → channels 2, 3
// Don't comment out
#define SERVO_CHANNEL_OFFSET  0