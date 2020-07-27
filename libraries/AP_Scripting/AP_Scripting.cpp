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
  #define SCRIPTING_STACK_SIZE (17 * 1024) // Linux experiences stack corruption at ~16.25KB when handed bad scripts
#endif // !defined(SCRIPTING_STACK_SIZE)

#if !defined(SCRIPTING_STACK_MAX_SIZE)
  #define SCRIPTING_STACK_MAX_SIZE (64 * 1024)
#endif // !defined(SCRIPTING_STACK_MAX_SIZE)

#if !defined(SCRIPTING_HEAP_SIZE)
  #if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #define SCRIPTING_HEAP_SIZE (64 * 1024)
  #else
    #define SCRIPTING_HEAP_SIZE (43 * 1024)
  #endif
#endif // !defined(SCRIPTING_HEAP_SIZE)

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
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Scripting, _enable, 0, AP_PARAM_FLAG_ENABLE),

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
    AP_GROUPINFO("HEAP_SIZE", 3, AP_Scripting, _script_heap_size, SCRIPTING_HEAP_SIZE),

    // @Param: DEBUG_LVL
    // @DisplayName: Scripting Debug Level
    // @Description: The higher the number the more verbose builtin scripting debug will be.
    // @User: Advanced
    AP_GROUPINFO("DEBUG_LVL", 4, AP_Scripting, _debug_level, 0),

    // @Param: USER1
    // @DisplayName: Scripting User Parameter1
    // @Description: General purpose user variable input for scripts
    // @User: Standard
    AP_GROUPINFO("USER1", 5, AP_Scripting, _user[0], 0.0),

    // @Param: USER2
    // @DisplayName: Scripting User Parameter2
    // @Description: General purpose user variable input for scripts
    // @User: Standard
    AP_GROUPINFO("USER2", 6, AP_Scripting, _user[1], 0.0),

    // @Param: USER3
    // @DisplayName: Scripting User Parameter3
    // @Description: General purpose user variable input for scripts
    // @User: Standard
    AP_GROUPINFO("USER3", 7, AP_Scripting, _user[2], 0.0),

    // @Param: USER4
    // @DisplayName: Scripting User Parameter4
    // @Description: General purpose user variable input for scripts
    // @User: Standard
    AP_GROUPINFO("USER4", 8, AP_Scripting, _user[3], 0.0),

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

void AP_Scripting::init(void) {
    if (!_enable) {
        return;
    }

    const char *dir_name = SCRIPTING_DIRECTORY;
    if (AP::FS().mkdir(dir_name)) {
        if (errno != EEXIST) {
            gcs().send_text(MAV_SEVERITY_INFO, "Lua: failed to create (%s)", dir_name);
            return;
        }
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Scripting::thread, void),
                                      "Scripting", SCRIPTING_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Could not create scripting stack (%d)", SCRIPTING_STACK_SIZE);
        gcs().send_text(MAV_SEVERITY_ERROR, "Scripting failed to start");
        _init_failed = true;
    }
}

MAV_RESULT AP_Scripting::handle_command_int_packet(const mavlink_command_int_t &packet) {
    switch ((SCRIPTING_CMD)packet.param1) {
        case SCRIPTING_CMD_REPL_START:
            return repl_start() ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        case SCRIPTING_CMD_REPL_STOP:
            repl_stop();
            return MAV_RESULT_ACCEPTED;
        case SCRIPTING_CMD_ENUM_END: // cope with MAVLink generator appending to our enum
            break;
    }

    return MAV_RESULT_UNSUPPORTED;
}

bool AP_Scripting::repl_start(void) {
    if (terminal.session) { // it's already running, this is fine
        return true;
    }

    // nuke the old folder and all contents
    struct stat st;
    if ((AP::FS().stat(REPL_DIRECTORY, &st) == -1) &&
        (AP::FS().unlink(REPL_DIRECTORY)  == -1) &&
        (errno != EEXIST)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Unable to delete old REPL %s", strerror(errno));
    }

    // create a new folder
    AP::FS().mkdir(REPL_DIRECTORY);
    // delete old files in case we couldn't
    AP::FS().unlink(REPL_DIRECTORY "/in");
    AP::FS().unlink(REPL_DIRECTORY "/out");

    // make the output pointer
    terminal.output_fd = AP::FS().open(REPL_OUT, O_WRONLY|O_CREAT|O_TRUNC);
    if (terminal.output_fd == -1) {
        gcs().send_text(MAV_SEVERITY_INFO, "Unable to make new REPL");
        return false;
    }

    terminal.session = true;
    return true;
}

void AP_Scripting::repl_stop(void) {
    terminal.session = false;
    // can't do any more cleanup here, closing the open FD's is the REPL's responsibility
}

void AP_Scripting::thread(void) {
    lua_scripts *lua = new lua_scripts(_script_vm_exec_count, _script_heap_size, _debug_level, terminal);
    if (lua == nullptr || !lua->heap_allocated()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Unable to allocate scripting memory");
        delete lua;
        _init_failed = true;
        return;
    }
    lua->run();

    // only reachable if the lua backend has died for any reason
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Scripting has stopped");
}

AP_Scripting *AP_Scripting::_singleton = nullptr;

namespace AP {
    AP_Scripting *scripting() {
        return AP_Scripting::get_singleton();
    }
}
