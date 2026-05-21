#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_ACTUATORS_ENABLED
// AP_Periph builds don't link AP_Actuators; default the feature off
// there so callers (e.g. handle_command_do_set_actuator in
// GCS_Common.cpp) compile out cleanly.
#define AP_ACTUATORS_ENABLED !defined(HAL_BUILD_AP_PERIPH)
#endif

#ifndef AP_ACTUATORS_MAX_INSTANCES
#define AP_ACTUATORS_MAX_INSTANCES 6
#endif

#define ACTUATOR_DEFAULT_INCREMENT 0.01
