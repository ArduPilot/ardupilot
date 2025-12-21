#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SRV_CHANNELS_ENABLED
#define AP_SRV_CHANNELS_ENABLED 1
#endif

#ifndef ACTUATOR_CHANNELS
#define ACTUATOR_CHANNELS 6
#endif

#define ACTUATOR_DEFAULT_INCREMENT 0.01

#ifndef NUM_SERVO_CHANNELS
    #if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
        #define NUM_SERVO_CHANNELS 32
    #else
        #define NUM_SERVO_CHANNELS 16
    #endif
#endif
