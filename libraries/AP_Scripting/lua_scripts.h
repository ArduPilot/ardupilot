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

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <setjmp.h>

#include <AP_Filesystem/posix_compat.h>
#include <AP_Scripting/AP_Scripting.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/Semaphores.h>

#include "lua/src/lua.hpp"

#ifndef REPL_DIRECTORY
  #if HAL_OS_FATFS_IO
    #define REPL_DIRECTORY "/APM/repl"
  #else
    #define REPL_DIRECTORY "./repl"
  #endif //HAL_OS_FATFS_IO
#endif // REPL_DIRECTORY

#ifndef SCRIPTING_DIRECTORY
  #if HAL_OS_FATFS_IO
    #define SCRIPTING_DIRECTORY "/APM/scripts"
  #else
    #define SCRIPTING_DIRECTORY "./scripts"
  #endif //HAL_OS_FATFS_IO
#endif // SCRIPTING_DIRECTORY

#ifndef REPL_IN
  #define REPL_IN REPL_DIRECTORY "/in"
#endif // REPL_IN

#ifndef REPL_OUT
  #define REPL_OUT REPL_DIRECTORY "/out"
#endif // REPL_OUT

class lua_scripts
{
public:
    lua_scripts(const AP_Int32 &vm_steps, const AP_Int32 &heap_size, const AP_Int8 &debug_options, struct AP_Scripting::terminal_s &_terminal);

    ~lua_scripts();

    CLASS_NO_COPY(lua_scripts);

    // return true if initialisation failed
    bool heap_allocated() const { return _heap != nullptr; }

    // run scripts, does not return unless an error occured
    void run(void);

    static bool overtime; // script exceeded it's execution slot, and we are bailing out

    enum class DebugLevel {
        NO_SCRIPTS_TO_RUN = 1U << 0,
        RUNTIME_MSG = 1U << 1,
        SUPPRESS_SCRIPT_LOG = 1U << 2,
        LOG_RUNTIME = 1U << 3,
        DISABLE_PRE_ARM = 1U << 4,
    };

private:

    void create_sandbox(lua_State *L);

    void repl_cleanup(void);

    typedef struct script_info {
       int lua_ref;          // reference to the loaded script object
       uint64_t next_run_ms; // time (in milliseconds) the script should next be run at
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

    // REPL stuff
    struct AP_Scripting::terminal_s &terminal;
    void doREPL(lua_State *L);
    void l_print(lua_State *L);
    void terminal_print(const char *str);
    int loadline(lua_State *L);
    int multiline(lua_State *L);
    int addreturn(lua_State *L);
    int pushline(lua_State *L, int firstline);
    int incomplete(lua_State *L, int status);
    const char * get_prompt(lua_State *L, int firstline);
    int docall(lua_State *L, int narg, int nres) const;
    int sandbox_ref;

    script_info *scripts; // linked list of scripts to be run, sorted by next run time (soonest first)

    // hook will be run when CPU time for a script is exceeded
    // it must be static to be passed to the C API
    static void hook(lua_State *L, lua_Debug *ar);

    // lua panic handler, will jump back to the start of run
    static int atpanic(lua_State *L);
    static jmp_buf panic_jmp;

    lua_State *lua_state;

    const AP_Int32 & _vm_steps;
    const AP_Int8 & _debug_options;

    static void *alloc(void *ud, void *ptr, size_t osize, size_t nsize);

    static void *_heap;

    // must be static for use in atpanic
    static void print_error(MAV_SEVERITY severity);
    static char *error_msg_buf;
    static HAL_Semaphore error_msg_buf_sem;
    static uint8_t print_error_count;
    static uint32_t last_print_ms;

public:
    // must be static for use in atpanic, public to allow bindings to issue none fatal warnings
    static void set_and_print_new_error_message(MAV_SEVERITY severity, const char *fmt, ...) FMT_PRINTF(2,3);

    // return last error message, nullptr if none, must use semaphore as this is updated in the scripting thread
    static const char* get_last_error_message() { return error_msg_buf; }

    // get semaphore for above error buffer
    static AP_HAL::Semaphore* get_last_error_semaphore() { return &error_msg_buf_sem; }

};
