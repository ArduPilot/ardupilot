#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_AIS_ENABLED
#if HAL_PROGRAM_SIZE_LIMIT_KB <= 1024
    #define AP_AIS_ENABLED 0
#else
    #define AP_AIS_ENABLED 2
#endif
#endif
