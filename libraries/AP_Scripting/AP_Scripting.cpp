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

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

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
  #if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX || HAL_MEM_CLASS >= HAL_MEM_CLASS_1000
    #define SCRIPTING_HEAP_SIZE (200 * 1024)
  #elif HAL_MEM_CLASS >= HAL_MEM_CLASS_500
    #define SCRIPTING_HEAP_SIZE (100 * 1024)
  #else
    #define SCRIPTING_HEAP_SIZE (43 * 1024)
  #endif
#endif // !defined(SCRIPTING_HEAP_SIZE)

static_assert(SCRIPTING_STACK_SIZE >= SCRIPTING_STACK_MIN_SIZE, "Scripting requires a larger minimum stack size");
static_assert(SCRIPTING_STACK_SIZE <= SCRIPTING_STACK_MAX_SIZE, "Scripting requires a smaller stack size");

#ifndef SCRIPTING_ENABLE_DEFAULT
#define SCRIPTING_ENABLE_DEFAULT 0
#endif

#if AP_NETWORKING_ENABLED
#include <AP_HAL/utility/Socket.h>
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Scripting::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Scripting
    // @Description: Controls if scripting is enabled
    // @Values: 0:None,1:Lua Scripts
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Scripting, _enable, SCRIPTING_ENABLE_DEFAULT, AP_PARAM_FLAG_ENABLE),

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

    // @Param: DEBUG_OPTS
    // @DisplayName: Scripting Debug Level
    // @Description: Debugging options
    // @Bitmask: 0: No Scripts to run message if all scripts have stopped
    // @Bitmask: 1: Runtime messages for memory usage and execution time
    // @Bitmask: 2: Suppress logging scripts to dataflash
    // @Bitmask: 3: log runtime memory usage and execution time
    // @Bitmask: 4: Disable pre-arm check
    // @Bitmask: 5: Save CRC of current scripts to loaded and running checksum parameters enabling pre-arm
    // @User: Advanced
    AP_GROUPINFO("DEBUG_OPTS", 4, AP_Scripting, _debug_options, 0),

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

    // @Param: USER5
    // @DisplayName: Scripting User Parameter5
    // @Description: General purpose user variable input for scripts
    // @User: Standard
    AP_GROUPINFO("USER5", 10, AP_Scripting, _user[4], 0.0),

    // @Param: USER6
    // @DisplayName: Scripting User Parameter6
    // @Description: General purpose user variable input for scripts
    // @User: Standard
    AP_GROUPINFO("USER6", 11, AP_Scripting, _user[5], 0.0),
    
    // @Param: DIR_DISABLE
    // @DisplayName: Directory disable
    // @Description: This will stop scripts being loaded from the given locations
    // @Bitmask: 0:ROMFS, 1:APM/scripts
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("DIR_DISABLE", 9, AP_Scripting, _dir_disable, 0),

    // @Param: LD_CHECKSUM
    // @DisplayName: Loaded script checksum
    // @Description: Required XOR of CRC32 checksum of loaded scripts, vehicle will not arm with incorrect scripts loaded, -1 disables
    // @User: Advanced
    AP_GROUPINFO("LD_CHECKSUM", 12, AP_Scripting, _required_loaded_checksum, -1),

    // @Param: RUN_CHECKSUM
    // @DisplayName: Running script checksum
    // @Description: Required XOR of CRC32 checksum of running scripts, vehicle will not arm with incorrect scripts running, -1 disables
    // @User: Advanced
    AP_GROUPINFO("RUN_CHECKSUM", 13, AP_Scripting, _required_running_checksum, -1),

    // @Param: THD_PRIORITY
    // @DisplayName: Scripting thread priority
    // @Description: This sets the priority of the scripting thread. This is normally set to a low priority to prevent scripts from interfering with other parts of the system. Advanced users can change this priority if scripting needs to be prioritised for realtime applications. WARNING: changing this parameter can impact the stability of your flight controller. The scipting thread priority in this parameter is chosen based on a set of system level priorities for other subsystems. It is strongly recommended that you use the lowest priority that is sufficient for your application. Note that all scripts run at the same priority, so if you raise this priority you must carefully audit all lua scripts for behaviour that does not interfere with the operation of the system.
    // @Values: 0:Normal, 1:IO Priority, 2:Storage Priority, 3:UART Priority, 4:I2C Priority, 5:SPI Priority, 6:Timer Priority, 7:Main Priority, 8:Boost Priority
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("THD_PRIORITY", 14, AP_Scripting, _thd_priority, uint8_t(ThreadPriority::NORMAL)),
    
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

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    if ((_dir_disable & uint16_t(AP_Scripting::SCR_DIR::SCRIPTS)) == 0) {
        // Only try creating scripts directory if loading from it is enabled
        const char *dir_name = SCRIPTING_DIRECTORY;
        if (AP::FS().mkdir(dir_name)) {
            if (errno != EEXIST) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting: failed to create (%s)", dir_name);
            }
        }
    }
#endif

    AP_HAL::Scheduler::priority_base priority = AP_HAL::Scheduler::PRIORITY_SCRIPTING;
    static const struct {
        ThreadPriority scr_priority;
        AP_HAL::Scheduler::priority_base hal_priority;
    } priority_map[] = {
        { ThreadPriority::NORMAL, AP_HAL::Scheduler::PRIORITY_SCRIPTING },
        { ThreadPriority::IO, AP_HAL::Scheduler::PRIORITY_IO },
        { ThreadPriority::STORAGE, AP_HAL::Scheduler::PRIORITY_STORAGE },
        { ThreadPriority::UART, AP_HAL::Scheduler::PRIORITY_UART },
        { ThreadPriority::I2C, AP_HAL::Scheduler::PRIORITY_I2C },
        { ThreadPriority::SPI, AP_HAL::Scheduler::PRIORITY_SPI },
        { ThreadPriority::TIMER, AP_HAL::Scheduler::PRIORITY_TIMER },
        { ThreadPriority::MAIN, AP_HAL::Scheduler::PRIORITY_MAIN },
        { ThreadPriority::BOOST, AP_HAL::Scheduler::PRIORITY_BOOST },
    };
    for (const auto &p : priority_map) {
        if (p.scr_priority == _thd_priority) {
            priority = p.hal_priority;
        }
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Scripting::thread, void),
                                      "Scripting", SCRIPTING_STACK_SIZE, priority, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Scripting: %s", "failed to start");
        _thread_failed = true;
    }
}

#if HAL_GCS_ENABLED
MAV_RESULT AP_Scripting::handle_command_int_packet(const mavlink_command_int_t &packet) {
    switch ((SCRIPTING_CMD)packet.param1) {
        case SCRIPTING_CMD_REPL_START:
            return repl_start() ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        case SCRIPTING_CMD_REPL_STOP:
            repl_stop();
            return MAV_RESULT_ACCEPTED;
        case SCRIPTING_CMD_STOP:
            _restart = false;
            _stop = true;
            return MAV_RESULT_ACCEPTED;
        case SCRIPTING_CMD_STOP_AND_RESTART:
            _restart = true;
            _stop = true;
            return MAV_RESULT_ACCEPTED;
        case SCRIPTING_CMD_ENUM_END: // cope with MAVLink generator appending to our enum
            break;
    }

    return MAV_RESULT_UNSUPPORTED;
}
#endif

bool AP_Scripting::repl_start(void) {
    if (terminal.session) { // it's already running, this is fine
        return true;
    }

    // nuke the old folder and all contents
    struct stat st;
    if ((AP::FS().stat(REPL_DIRECTORY, &st) == -1) &&
        (AP::FS().unlink(REPL_DIRECTORY)  == -1) &&
        (errno != EEXIST)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting: Unable to delete old REPL %s", strerror(errno));
    }

    // create a new folder
    AP::FS().mkdir(REPL_DIRECTORY);
    // delete old files in case we couldn't
    AP::FS().unlink(REPL_DIRECTORY "/in");
    AP::FS().unlink(REPL_DIRECTORY "/out");

    // make the output pointer
    terminal.output_fd = AP::FS().open(REPL_OUT, O_WRONLY|O_CREAT|O_TRUNC);
    if (terminal.output_fd == -1) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting: %s", "Unable to make new REPL");
        return false;
    }

    terminal.session = true;
    return true;
}

void AP_Scripting::repl_stop(void) {
    terminal.session = false;
    // can't do any more cleanup here, closing the open FD's is the REPL's responsibility
}

/*
  avoid optimisation of the thread function. This avoids nasty traps
  where setjmp/longjmp does not properly handle save/restore of
  floating point registers on exceptions. This is an extra protection
  over the top of the fix in luaD_rawrunprotected() for the same issue
 */
#pragma GCC push_options
#pragma GCC optimize ("O0")

void AP_Scripting::thread(void) {
    while (true) {
        // reset flags
        _stop = false;
        _restart = false;
        _init_failed = false;

        lua_scripts *lua = new lua_scripts(_script_vm_exec_count, _script_heap_size, _debug_options, terminal);
        if (lua == nullptr || !lua->heap_allocated()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Scripting: %s", "Unable to allocate memory");
            _init_failed = true;
        } else {
            // run won't return while scripting is still active
            lua->run();

            // only reachable if the lua backend has died for any reason
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Scripting: %s", "stopped");
        }
        delete lua;
        lua = nullptr;

        // clear allocated i2c devices
        for (uint8_t i=0; i<SCRIPTING_MAX_NUM_I2C_DEVICE; i++) {
            delete _i2c_dev[i];
            _i2c_dev[i] = nullptr;
        }
        num_i2c_devices = 0;

        // clear allocated PWM sources
        for (uint8_t i=0; i<SCRIPTING_MAX_NUM_PWM_SOURCE; i++) {
            if (_pwm_source[i] != nullptr) {
                delete _pwm_source[i];
                _pwm_source[i] = nullptr;
            }
        }
        num_pwm_source = 0;

#if AP_NETWORKING_ENABLED
        // clear allocated sockets
        for (uint8_t i=0; i<SCRIPTING_MAX_NUM_NET_SOCKET; i++) {
            if (_net_sockets[i] != nullptr) {
                delete _net_sockets[i];
                _net_sockets[i] = nullptr;
            }
        }
#endif // AP_NETWORKING_ENABLED
        
        // Clear blocked commands
        {
            WITH_SEMAPHORE(mavlink_command_block_list_sem);
            while (mavlink_command_block_list != nullptr) {
                command_block_list *next_item = mavlink_command_block_list->next;
                delete mavlink_command_block_list;
                mavlink_command_block_list = next_item;
            }
        }

        bool cleared = false;
        while(true) {
            // 1hz check if we should restart
            hal.scheduler->delay(1000);
            if (!enabled()) {
                // enable must be put to 0 and back to 1 to restart from params
                cleared = true;
                continue;
            }
            // must be enabled to get this far
            if (cleared || _restart) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Scripting: %s", "restarted");
                break;
            }
            if ((_debug_options.get() & uint8_t(lua_scripts::DebugLevel::NO_SCRIPTS_TO_RUN)) != 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Scripting: %s", "stopped");
            }
        }
    }
}
#pragma GCC pop_options

void AP_Scripting::handle_mission_command(const AP_Mission::Mission_Command& cmd_in)
{
#if AP_MISSION_ENABLED
    if (!_enable) {
        return;
    }

    if (mission_data == nullptr) {
        // load buffer
        mission_data = new ObjectBuffer<struct AP_Scripting::scripting_mission_cmd>(mission_cmd_queue_size);
        if (mission_data != nullptr && mission_data->get_size() == 0) {
            delete mission_data;
            mission_data = nullptr;
        }
        if (mission_data == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Scripting: %s", "unable to receive mission command");
            return;
        }
    }

    struct scripting_mission_cmd cmd {cmd_in.p1,
                                      cmd_in.content.scripting.p1,
                                      cmd_in.content.scripting.p2,
                                      cmd_in.content.scripting.p3,
                                      AP_HAL::millis()};

    mission_data->push(cmd);
#endif
}

bool AP_Scripting::arming_checks(size_t buflen, char *buffer) const
{
    if (!enabled() || ((_debug_options.get() & uint8_t(lua_scripts::DebugLevel::DISABLE_PRE_ARM)) != 0)) {
        return true;
    }

    if (_thread_failed) {
        hal.util->snprintf(buffer, buflen, "Scripting: %s", "failed to start");
        return false;
    }

    if (_init_failed) {
        hal.util->snprintf(buffer, buflen, "Scripting: %s", "out of memory");
        return false;
    }

    lua_scripts::get_last_error_semaphore()->take_blocking();
    const char *error_buf = lua_scripts::get_last_error_message();
    if (error_buf != nullptr) {
        hal.util->snprintf(buffer, buflen, "Scripting: %s", error_buf);
        lua_scripts::get_last_error_semaphore()->give();
        return false;
    }
    lua_scripts::get_last_error_semaphore()->give();

    // Use -1 for disabled, this means we don't have to avoid 0 in the CRC, the sign bit is masked off anyway
    if (_required_loaded_checksum != -1) {
        const uint32_t expected_loaded = (uint32_t)_required_loaded_checksum.get() & checksum_param_mask;
        const uint32_t loaded = lua_scripts::get_loaded_checksum() & checksum_param_mask;
        if (expected_loaded != loaded) {
            hal.util->snprintf(buffer, buflen, "Scripting: loaded CRC incorrect want: 0x%x", (unsigned int)loaded);
            return false;
        }
    }

    if (_required_running_checksum != -1) {
        const uint32_t expected_running = (uint32_t)_required_running_checksum.get() & checksum_param_mask;
        const uint32_t running = lua_scripts::get_running_checksum() & checksum_param_mask;
        if (expected_running != running) {
            hal.util->snprintf(buffer, buflen, "Scripting: running CRC incorrect want: 0x%x", (unsigned int)running);
            return false;
        }
    }

    return true;
}

void AP_Scripting::restart_all()
{
    _restart = true;
    _stop = true;
}

#if HAL_GCS_ENABLED
void AP_Scripting::handle_message(const mavlink_message_t &msg, const mavlink_channel_t chan) {
    if (mavlink_data.rx_buffer == nullptr) {
        return;
    }

    struct mavlink_msg data {msg, chan, AP_HAL::millis()};

    WITH_SEMAPHORE(mavlink_data.sem);
    for (uint16_t i = 0; i < mavlink_data.accept_msg_ids_size; i++) {
        if (mavlink_data.accept_msg_ids[i] == UINT32_MAX) {
            return;
        }
        if (mavlink_data.accept_msg_ids[i] == msg.msgid) {
            mavlink_data.rx_buffer->push(data);
            return;
        }
    }
}

bool AP_Scripting::is_handling_command(uint16_t cmd_id)
{
    WITH_SEMAPHORE(mavlink_command_block_list_sem);

    // Look in linked list to see if id is registered
    if (mavlink_command_block_list != nullptr) {
        for (command_block_list *item = mavlink_command_block_list; item; item = item->next) {
            if (item->id == cmd_id) {
                return true;
            }
        }
    }

    return false;
}
#endif // HAL_GCS_ENABLED

// Update called at 1Hz from AP_Vehicle
void AP_Scripting::update() {

    save_checksum();

}

// Check if DEBUG_OPTS bit has been set to save current checksum values to params
void AP_Scripting::save_checksum() {

    const uint8_t opts = _debug_options.get();
    const uint8_t save_bit = uint8_t(lua_scripts::DebugLevel::SAVE_CHECKSUM);
    if ((opts & save_bit) == 0) {
        // Bit not set, nothing to do
        return;
    }

    // Save two checksum parameters to there current values
    _required_loaded_checksum.set_and_save(lua_scripts::get_loaded_checksum() & checksum_param_mask);
    _required_running_checksum.set_and_save(lua_scripts::get_running_checksum() & checksum_param_mask);

    // Un-set debug option bit
    _debug_options.set_and_save(opts & ~save_bit);

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Scripting: %s", "saved checksums");

}

AP_Scripting *AP_Scripting::_singleton = nullptr;

namespace AP {
    AP_Scripting *scripting() {
        return AP_Scripting::get_singleton();
    }
}

#endif  // AP_SCRIPTING_ENABLED
