/*
    Include this file which will include the other files required to use
    the library based on which parameters have been configured in the can_config.h file
*/

#pragma once

// Include main canbus library
// This library includes the 
#include <can_node.h>


#ifdef USE_PITOT
    #include <can_pitot.h>
#endif

#ifdef USE_BATTERY
    #include <can_battery.h>
#endif

#ifdef USE_ESC
    #include <can_esc.h>
#endif

#ifdef USE_SERVO
    #include <can_servo.h>
#endif
