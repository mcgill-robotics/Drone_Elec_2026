/*
    Include this file which will include the other files required to use
    the library based on which parameters have been configured in the can_config.h file
*/

#pragma once

// Include main canbus library
// This library includes the 
#include <can_node.h>

// Include other canbus libraries
// Will be compiler optimized out if not used
#include <can_pitot.h>

#include <can_battery.h>

#include <can_esc.h>

#include <can_servo.h>
