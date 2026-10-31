#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#ifndef AP_ACTUATORS_ENABLED
#define AP_ACTUATORS_ENABLED APM_BUILD_TYPE(APM_BUILD_ArduSub)
#endif

// one per SRV_Channel function k_actuator1..k_actuator6
#define AP_ACTUATORS_MAX_INSTANCES 6

#define AP_ACTUATORS_DEFAULT_INCREMENT 0.01
