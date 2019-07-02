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

#include "lua_bindings.h"

class lua_scripts
{
public:
    lua_scripts(const AP_Int32 &vm_steps, const AP_Int32 &heap_size, const AP_Int8 &debug_level);

    /* Do not allow copies */
    lua_scripts(const lua_scripts &other) = delete;
    lua_scripts &operator=(const lua_scripts&) = delete;

    // run scripts, does not return unless an error occured
    void run(void);

    static bool overtime; // script exceeded it's execution slot, and we are bailing out
private:

    typedef struct script_info {
       int lua_ref;          // reference to the loaded script object
       uint64_t next_run_ms; // time (in milliseconds) the script should next be run at
       char *name;           // filename for the script // FIXME: This information should be available from Lua
       script_info *next;
    } script_info;

    script_info *load_script(lua_State *L, char *filename);

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
    static jmp_buf panic_jmp;

    lua_State *lua_state;

    const AP_Int32 & _vm_steps;
    const AP_Int8 & _debug_level;

    static void *alloc(void *ud, void *ptr, size_t osize, size_t nsize);

    static void *_heap;
};
