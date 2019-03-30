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

#include "lua_scripts.h"

// ensure that we have a set of stack sizes, and enforce constraints around it
// except for the minimum size, these are allowed to be defined by the build system
#undef SCRIPTING_STACK_MIN_SIZE
#define SCRIPTING_STACK_MIN_SIZE (8 * 1024)

#if !defined(SCRIPTING_STACK_SIZE)
  #define SCRIPTING_STACK_SIZE (16 * 1024)
#endif // !defined(SCRIPTING_STACK_SIZE)

#if !defined(SCRIPTING_STACK_MAX_SIZE)
  #define SCRIPTING_STACK_MAX_SIZE (64 * 1024)
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

    // @Param: HEAP_SIZE
    // @DisplayName: Scripting Heap Size
    // @Description: Amount of memory available for scripting
    // @Range: 1024 1048576
    // @Increment: 1024
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HEAP_SIZE", 3, AP_Scripting, _script_heap_size, 32*1024),

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
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Could not create scripting stack (%d)", SCRIPTING_STACK_SIZE);
        return false;
    }

    return true;
}

void AP_Scripting::thread(void) {
    lua_scripts *lua = new lua_scripts(_script_vm_exec_count, _script_heap_size);
    if (lua == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Unable to allocate scripting memory");
        return;
    }
    lua->run();

    // only reachable if the lua backend has died for any reason
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Scripting has died");
}

AP_Scripting *AP_Scripting::_singleton = nullptr;

namespace AP {
    AP_Scripting *scripting() {
        return AP_Scripting::get_singleton();
    }
}
