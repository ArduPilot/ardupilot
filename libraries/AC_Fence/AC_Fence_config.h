#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include <GCS_MAVLink/GCS_config.h>

// Enabled 0 is compiled out
// Enabled 1 is always enabled on all vehicles
// Enabled 2 is enabled with dummy methods for tracker and blimp

#ifndef AP_FENCE_ENABLED
#define AP_FENCE_ENABLED 2
#endif

#ifndef AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
#define AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT HAL_GCS_ENABLED && AP_FENCE_ENABLED
#endif
