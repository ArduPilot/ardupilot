#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Scripting/AP_Scripting_config.h>

// reset_home(), the in-flight return point reset, is only reachable from Lua scripting
#ifndef AP_SMARTRTL_RESET_ENABLED
#define AP_SMARTRTL_RESET_ENABLED AP_SCRIPTING_LUA_ENABLED
#endif
