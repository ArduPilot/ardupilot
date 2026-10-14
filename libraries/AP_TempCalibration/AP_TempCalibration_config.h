#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_TEMPCALIBRATION_ENABLED
#define AP_TEMPCALIBRATION_ENABLED !APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#endif
