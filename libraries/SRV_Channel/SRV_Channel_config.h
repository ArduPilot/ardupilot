#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef NUM_SERVO_CHANNELS
#if defined(HAL_BUILD_AP_PERIPH) && defined(HAL_PWM_COUNT)
    #define NUM_SERVO_CHANNELS HAL_PWM_COUNT
#elif defined(HAL_BUILD_AP_PERIPH)
    #define NUM_SERVO_CHANNELS 0
#else
    #if !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
        #define NUM_SERVO_CHANNELS 32
    #else
        #define NUM_SERVO_CHANNELS 16
    #endif
#endif
#endif
