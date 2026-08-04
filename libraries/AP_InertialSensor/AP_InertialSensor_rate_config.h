#pragma once

// board-specific values for this subsystem are generated into a hwdef
// fragment so they only enter the include closure of code using them
#if __has_include(<hwdef_ins.h>)
#include <hwdef_ins.h>
#endif

#ifndef HAL_INS_RATE_LOOP
#define HAL_INS_RATE_LOOP 0
#endif

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_InertialSensor/AP_InertialSensor_config.h>

#ifndef AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
#define AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED (AP_INERTIALSENSOR_ENABLED && HAL_INS_RATE_LOOP && AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduCopter))
#endif
