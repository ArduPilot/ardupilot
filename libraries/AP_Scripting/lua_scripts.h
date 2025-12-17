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
#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/ap_setjmp.h>

#include <AP_Filesystem/posix_compat.h>
#include <AP_Scripting/AP_Scripting.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/Semaphores.h>
#include <AP_MultiHeap/AP_MultiHeap.h>
#include "lua_common_defs.h"

#include "lua/src/lua.hpp"

class lua_scripts
{
public:
    lua_scripts(const AP_Int32 &vm_steps, const AP_Int32 &heap_size, AP_Int8 &debug_options);

    ~lua_scripts();

    CLASS_NO_COPY(lua_scripts);

    // return true if initialisation failed
    bool heap_allocated() const { return _heap.available(); }

    // run scripts, does not return unless an error occured
    void run(void);

    static bool overtime; // script exceeded it's execution slot, and we are bailing out

private:

    void create_sandbox(lua_State *L);

    typedef struct script_info {
       int env_ref;          // reference to the script's environment table
       int run_ref;          // reference to the function to run
       uint64_t next_run_ms; // time (in milliseconds) the script should next be run at
       uint32_t crc;         // crc32 checksum
       char *name;           // filename for the script // FIXME: This information should be available from Lua
       script_info *next;
    } script_info;

    script_info *load_script(lua_State *L, char *filename);

    void reset_loop_overtime(lua_State *L);

    void load_all_scripts_in_dir(lua_State *L, const char *dirname);

    void run_next_script(lua_State *L);

    void remove_script(lua_State *L, script_info *script);

    // reschedule the script for execution. It is assumed the script is not in the list already
    void reschedule_script(script_info *script);

    script_info *scripts; // linked list of scripts to be run, sorted by next run time (soonest first)

    // hook will be run when CPU time for a script is exceeded
    // it must be static to be passed to the C API
    static void hook(lua_State *L, lua_Debug *ar);

    // lua panic handler, will jump back to the start of run
    static int atpanic(lua_State *L);
    static ap_jmp_buf panic_jmp;

    lua_State *lua_state;

    const AP_Int32 & _vm_steps;
    AP_Int8 & _debug_options;

    bool option_is_set(AP_Scripting::DebugOption option) const {
        return (uint8_t(_debug_options.get()) & uint8_t(option)) != 0;
    }

    static void *alloc(void *ud, void *ptr, size_t osize, size_t nsize);

    static MultiHeap _heap;

    // helper for print and log of runtime stats
    void update_stats(const char *name, uint32_t run_time, int total_mem, int run_mem);

    // must be static for use in atpanic
    static void print_error(MAV_SEVERITY severity);
    static char *error_msg_buf;
    static HAL_Semaphore error_msg_buf_sem;
    static uint8_t print_error_count;
    static uint32_t last_print_ms;

    // XOR of crc32 of running scripts
    static uint32_t loaded_checksum;
    static uint32_t running_checksum;
    static HAL_Semaphore crc_sem;

public:
    // must be static for use in atpanic, public to allow bindings to issue none fatal warnings
    static void set_and_print_new_error_message(MAV_SEVERITY severity, const char *fmt, ...) FMT_PRINTF(2,3);

    // return last error message, nullptr if none, must use semaphore as this is updated in the scripting thread
    static const char* get_last_error_message() { return error_msg_buf; }

    // get semaphore for above error buffer
    static AP_HAL::Semaphore* get_last_error_semaphore() { return &error_msg_buf_sem; }

    // Return the file checksums of running and loaded scripts
    static uint32_t get_loaded_checksum();
    static uint32_t get_running_checksum();

};

#endif  // AP_SCRIPTING_ENABLED
