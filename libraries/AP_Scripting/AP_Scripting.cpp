/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Scripting/AP_Scripting.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include "lua_bindings.h"

// ensure that we have a set of stack sizes, and enforce constraints around it
// except for the minimum size, these are allowed to be defined by the build system
#undef SCRIPTING_STACK_MIN_SIZE
#define SCRIPTING_STACK_MIN_SIZE 2048

#if !defined(SCRIPTING_STACK_SIZE)
  #define SCRIPTING_STACK_SIZE 8192
#endif // !defined(SCRIPTING_STACK_SIZE)

#if !defined(SCRIPTING_STACK_MAX_SIZE)
  #define SCRIPTING_STACK_MAX_SIZE 16384
#endif // !defined(SCRIPTING_STACK_MAX_SIZE)

static_assert(SCRIPTING_STACK_SIZE >= SCRIPTING_STACK_MIN_SIZE, "Scripting requires a larger minimum stack size");
static_assert(SCRIPTING_STACK_SIZE <= SCRIPTING_STACK_MAX_SIZE, "Scripting requires a smaller stack size");

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Scripting::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Scripting
    // @Description: Controls if scripting is enabled
    // @Values: 0:None,1:Lua Scripts
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Scripting, _enable, 1, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_Scripting::AP_Scripting() {       
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Scripting must be a singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

bool AP_Scripting::init(void) {
    if (!_enable) {
        return true;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Scripting::thread, void),
                                      "Scripting", SCRIPTING_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0)) {
        return false;
    }

    _running = true;
    return true;
}

void AP_Scripting::thread(void) {
    unsigned int loop = 0;
    lua_State *state = luaL_newstate();
    luaL_openlibs(state);
    load_lua_bindings(state);

    luaL_loadstring(state, "gcs.send_text(string.format(\"1 + 2 = %d\", 1+2))");

    while (true) {
        hal.scheduler->delay(1000);
        gcs().send_text(MAV_SEVERITY_INFO, "Scripting Loop: %u", loop++);

        const uint32_t startMem = hal.util->available_memory();
        const uint32_t loadEnd = AP_HAL::micros();
        lua_pushvalue(state, -1);
        if(lua_pcall(state, 0, LUA_MULTRET, 0)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Lua: %s", lua_tostring(state, -1));
            hal.console->printf("Lua: %s", lua_tostring(state, -1));
            lua_pop(state, 1);
            continue;
        }
        const uint32_t runEnd = AP_HAL::micros();
        const uint32_t endMem = hal.util->available_memory();
        gcs().send_text(MAV_SEVERITY_INFO, "Execution: %d Memory: %d", runEnd - loadEnd, startMem - endMem);
    }
}

AP_Scripting *AP_Scripting::_singleton = nullptr;

namespace AP {
    AP_Scripting *scripting() {
        return AP_Scripting::get_singleton();
    }
}
