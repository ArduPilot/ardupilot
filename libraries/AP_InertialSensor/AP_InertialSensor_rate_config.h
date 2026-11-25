#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_InertialSensor/AP_InertialSensor_config.h>

// for the vehicles with a rate thread: Copter, and Plane for QuadPlane where the flash has room for it
#ifndef AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
#define AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED (AP_INERTIALSENSOR_ENABLED && HAL_INS_RATE_LOOP && AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED && (APM_BUILD_TYPE(APM_BUILD_ArduCopter) || (APM_BUILD_TYPE(APM_BUILD_ArduPlane) && HAL_PROGRAM_SIZE_LIMIT_KB > 1024)))
#endif
