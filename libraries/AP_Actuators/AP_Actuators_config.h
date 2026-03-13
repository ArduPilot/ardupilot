#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#ifndef AP_ACTUATORS_ENABLED
#define AP_ACTUATORS_ENABLED APM_BUILD_TYPE(APM_BUILD_ArduSub)
#endif

#ifndef AP_ACTUATORS_MAX_INSTANCES
#define AP_ACTUATORS_MAX_INSTANCES 6
#endif

#define ACTUATOR_DEFAULT_INCREMENT 0.01
