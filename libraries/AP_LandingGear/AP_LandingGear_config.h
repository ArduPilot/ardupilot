#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_LANDINGGEAR_ENABLED
#define AP_LANDINGGEAR_ENABLED APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#endif
