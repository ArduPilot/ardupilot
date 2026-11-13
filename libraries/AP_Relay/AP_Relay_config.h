#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Scripting/AP_Scripting_config.h>

#ifndef AP_RELAY_ENABLED
#define AP_RELAY_ENABLED 1
#endif

#define AP_RELAY_DRONECAN_ENABLED AP_RELAY_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS

// relays switched on with a timeout, for scripts and modules
#ifndef AP_RELAY_TIMEOUT_ENABLED
#define AP_RELAY_TIMEOUT_ENABLED AP_RELAY_ENABLED && AP_EXTERNAL_BINDING_ENABLED
#endif
