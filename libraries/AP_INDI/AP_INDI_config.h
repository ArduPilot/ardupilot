#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// INDI rate control with online control-effectiveness identification.
// Used by the Copter and Plane INDI custom controller backends. Enabled on
// SITL only until the validation in SPEC.md is complete.
#ifndef AP_INDI_ENABLED
#define AP_INDI_ENABLED CONFIG_HAL_BOARD == HAL_BOARD_SITL
#endif
