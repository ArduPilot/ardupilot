#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>


#include "HAL_ESP32_Class.h"

#ifndef NDEBUG
    #define DBG_PRINTF(fmt, args ...)  do { printf(fmt, ## args); } while(0)
#else
    #define DBG_PRINTF(fmt, args ...)
#endif
