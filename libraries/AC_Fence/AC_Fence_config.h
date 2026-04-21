#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include <GCS_MAVLink/GCS_config.h>

// Enabled 0 is compiled out
// Enabled 1 is always enabled on all vehicles
// Enabled 2 is enabled with dummy methods for tracker and blimp

#ifndef AP_FENCE_ENABLED
#define AP_FENCE_ENABLED 2
#endif

// CODE_REMOVAL
// ArduPilot 4.6 sends deprecation warnings for FENCE_POINT/FENCE_FETCH_POINT
// ArduPilot 4.7 stops compiling them in
// ArduPilot 4.8 removes the code entirely
#ifndef AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
#define AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT HAL_GCS_ENABLED && AP_FENCE_ENABLED
#endif
