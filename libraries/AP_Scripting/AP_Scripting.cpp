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
#include <AP_ROMFS/AP_ROMFS.h>

#include "lua_bindings.h"

// ensure that we have a set of stack sizes, and enforce constraints around it
// except for the minimum size, these are allowed to be defined by the build system
#undef SCRIPTING_STACK_MIN_SIZE
#define SCRIPTING_STACK_MIN_SIZE 2048

#if !defined(SCRIPTING_STACK_SIZE)
  #define SCRIPTING_STACK_SIZE 16384
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

    // @Param: VM_I_COUNT
    // @DisplayName: Scripting Virtual Machine Instruction Count
    // @Description: The number virtual machine instructions that can be run before considering a script to have taken an excessive amount of time
    // @Range: 1000 1000000
    // @Increment: 10000
    // @User: Advanced
    AP_GROUPINFO("VM_I_COUNT", 2, AP_Scripting, _script_vm_exec_count, 10000),

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

struct {
    bool overtime; // script exceeded it's execution slot, and we are bailing out with pcalls
} current_script;

void hook(lua_State *state, lua_Debug *ar) {
    gcs().send_text(MAV_SEVERITY_INFO, "got a debug hook");

    current_script.overtime = true;

    // we need to aggressively bail out as we are over time
    // so we will aggressively trap errors until we clear out
    lua_sethook(state, hook, LUA_MASKCOUNT, 1);

    luaL_error(state, "Exceeded CPU time");
}

void AP_Scripting::thread(void) {
    unsigned int loop = 0;
    lua_State *state = luaL_newstate();
    luaL_openlibs(state);
    load_lua_bindings(state);

    // load the sandbox creation function
    uint32_t sandbox_size;
    char *sandbox_data = (char *)AP_ROMFS::find_decompress("sandbox.lua", sandbox_size);
    if (sandbox_data == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Scripting: Could not find sandbox");
        return;
    }

    if (luaL_dostring(state, sandbox_data)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Scripting: Loading sandbox: %s", lua_tostring(state, -1));
        return;
    }
    free(sandbox_data);

    luaL_loadstring(state, "gcs.send_text(string.format(\"math.cos(1 + 2) = %f\", math.cos(1+2)))");
//    luaL_loadfile(state, "test.lua");
    lua_getglobal(state, "get_sandbox_env"); // find the sandbox creation function
    if (lua_pcall(state, 0, LUA_MULTRET, 0)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Scripting: Could not create sandbox: %s", lua_tostring(state, -1));
        return;
    }
    lua_setupvalue(state, -2, 1);

    while (true) {
        hal.scheduler->delay(1000);

        gcs().send_text(MAV_SEVERITY_INFO, "Scripting Loop: %u", loop++);

        // number of VM instructions to run, enforce a minimum script length, don't cap the upper end
        const int32_t vm_steps = MAX(_script_vm_exec_count, 1000);

        // reset the current script tracking information
        memset(&current_script, 0, sizeof(current_script));
        // always reset the hook in case we bailed out
        lua_sethook(state, hook, LUA_MASKCOUNT, vm_steps);

        const uint32_t startMem = hal.util->available_memory();
        const uint32_t loadEnd = AP_HAL::micros();
        lua_pushvalue(state, -1);
        if(lua_pcall(state, 0, LUA_MULTRET, 0)) {
            if (current_script.overtime) {
                // script has consumed an excessive amount of CPU time
                // FIXME: Find the chunkname for the error message
                // FIXME: Remove from future execution/restart?
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: exceeded time limit (%d)",  vm_steps);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Lua: %s", lua_tostring(state, -1));
                hal.console->printf("Lua: %s", lua_tostring(state, -1));
            }
            lua_pop(state, 1);
            continue;
        }
        const uint32_t runEnd = AP_HAL::micros();
        const uint32_t endMem = hal.util->available_memory();
        gcs().send_text(MAV_SEVERITY_INFO, "Time: %d Memory: %d", runEnd - loadEnd, startMem - endMem);
    }
}

AP_Scripting *AP_Scripting::_singleton = nullptr;

namespace AP {
    AP_Scripting *scripting() {
        return AP_Scripting::get_singleton();
    }
}
