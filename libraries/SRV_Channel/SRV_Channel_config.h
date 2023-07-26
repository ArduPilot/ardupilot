#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef NUM_SERVO_CHANNELS
    #if BOARD_FLASH_SIZE > 1024
        #define NUM_SERVO_CHANNELS 32
    #else
        #define NUM_SERVO_CHANNELS 16
    #endif
#endif
