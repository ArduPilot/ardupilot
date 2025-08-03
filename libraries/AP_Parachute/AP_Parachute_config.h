#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_PARACHUTE_ENABLED
// default to parachute enabled to match previous configs
#define HAL_PARACHUTE_ENABLED 1
#endif

#ifndef AP_PARACHUTE_UNAVAILABLE_ENABLED
#define AP_PARACHUTE_UNAVAILABLE_ENABLED 0
#endif
