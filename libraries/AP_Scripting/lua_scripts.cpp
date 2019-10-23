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

#include "lua_scripts.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Scripting.h"
#include <AP_ROMFS/AP_ROMFS.h>

#include "lua_generated_bindings.h"

#ifndef SCRIPTING_DIRECTORY
  #if HAL_OS_FATFS_IO
    #define SCRIPTING_DIRECTORY "/APM/scripts"
  #else
    #define SCRIPTING_DIRECTORY "./scripts"
  #endif //HAL_OS_FATFS_IO
#endif // SCRIPTING_DIRECTORY

extern const AP_HAL::HAL& hal;

bool lua_scripts::overtime;
jmp_buf lua_scripts::panic_jmp;

lua_scripts::lua_scripts(const AP_Int32 &vm_steps, const AP_Int32 &heap_size, const AP_Int8 &debug_level)
    : _vm_steps(vm_steps),
      _debug_level(debug_level) {
    _heap = hal.util->allocate_heap_memory(heap_size);
}

void lua_scripts::hook(lua_State *L, lua_Debug *ar) {
    lua_scripts::overtime = true;

    // we need to aggressively bail out as we are over time
    // so we will aggressively trap errors until we clear out
    lua_sethook(L, hook, LUA_MASKCOUNT, 1);

    luaL_error(L, "Exceeded CPU time");
}

int lua_scripts::atpanic(lua_State *L) {
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: Panic: %s", lua_tostring(L, -1));
    hal.console->printf("Lua: Panic: %s\n", lua_tostring(L, -1));
    longjmp(panic_jmp, 1);
    return 0;
}

lua_scripts::script_info *lua_scripts::load_script(lua_State *L, char *filename) {
    if (int error = luaL_loadfile(L, filename)) {
        switch (error) {
            case LUA_ERRSYNTAX:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: Syntax error in %s", filename);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: Error: %s", lua_tostring(L, -1));
                lua_pop(L, lua_gettop(L));
                return nullptr;
            case LUA_ERRMEM:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: Insufficent memory loading %s", filename);
                lua_pop(L, lua_gettop(L));
                return nullptr;
            case LUA_ERRFILE:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: Unable to load the file: %s", lua_tostring(L, -1));
                hal.console->printf("Lua: File error: %s\n", lua_tostring(L, -1));
                lua_pop(L, lua_gettop(L));
                return nullptr;
            default:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: Unknown error (%d) loading %s", error, filename);
                lua_pop(L, lua_gettop(L));
                return nullptr;
        }
    }

    script_info *new_script = (script_info *)hal.util->heap_realloc(_heap, nullptr, sizeof(script_info));
    if (new_script == nullptr) {
        // No memory, shouldn't happen, we even attempted to do a GC
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: Insufficent memory loading %s", filename);
        lua_pop(L, 1); // we can't use the function we just loaded, so ditch it
        return nullptr;
    }

    new_script->name = filename;
    new_script->next = nullptr;


    // find and create a sandbox for the new chunk
    lua_getglobal(L, "get_sandbox_env");
    if (lua_pcall(L, 0, LUA_MULTRET, 0)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Scripting: Could not create sandbox: %s", lua_tostring(L, -1));
        hal.console->printf("Lua: Scripting: Could not create sandbox: %s", lua_tostring(L, -1));
        return nullptr;
    }
    load_generated_sandbox(L);
    lua_setupvalue(L, -2, 1);

    new_script->lua_ref = luaL_ref(L, LUA_REGISTRYINDEX);   // cache the reference
    new_script->next_run_ms = AP_HAL::millis64() - 1; // force the script to be stale

    return new_script;
}

void lua_scripts::load_all_scripts_in_dir(lua_State *L, const char *dirname) {
    if (dirname == nullptr) {
        return;
    }

    DIR *d = AP::FS().opendir(dirname);
    if (d == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Lua: Could not find a scripts directory");
        return;
    }

    // load anything that ends in .lua
    for (struct dirent *de=AP::FS().readdir(d); de; de=AP::FS().readdir(d)) {
        uint8_t length = strlen(de->d_name);
        if (length < 5) {
            // not long enough
            continue;
        }

        if (strncmp(&de->d_name[length-4], ".lua", 4)) {
            // doesn't end in .lua
            continue;
        }

        // FIXME: because chunk name fetching is not working we are allocating and storing an extra string we shouldn't need to
        size_t size = strlen(dirname) + strlen(de->d_name) + 2;
        char * filename = (char *) hal.util->heap_realloc(_heap, nullptr, size);
        if (filename == nullptr) {
            continue;
        }
        snprintf(filename, size, "%s/%s", dirname, de->d_name);

        // we have something that looks like a lua file, attempt to load it
        script_info * script = load_script(L, filename);
        if (script == nullptr) {
            hal.util->heap_realloc(_heap, filename, 0);
            continue;
        }
        reschedule_script(script);

    }
    AP::FS().closedir(d);
}

void lua_scripts::run_next_script(lua_State *L) {
    if (scripts == nullptr) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
        AP_HAL::panic("Lua: Attempted to run a script without any scripts queued");
#endif // defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
        return;
    }

    // reset the current script tracking information
    overtime = false;

    // strip the selected script out of the list
    script_info *script = scripts;
    scripts = script->next;

    // reset the hook to clear the counter
    const int32_t vm_steps = MAX(_vm_steps, 1000);
    lua_sethook(L, hook, LUA_MASKCOUNT, vm_steps);

    // store top of stack so we can calculate the number of return values
    int stack_top = lua_gettop(L);

    // pop the function to the top of the stack
    lua_rawgeti(L, LUA_REGISTRYINDEX, script->lua_ref);

    if(lua_pcall(L, 0, LUA_MULTRET, 0)) {
        if (overtime) {
            // script has consumed an excessive amount of CPU time
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: %s exceeded time limit (%d)", script->name,  (int)vm_steps);
            remove_script(L, script);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Lua: %s", lua_tostring(L, -1));
            hal.console->printf("Lua: Error: %s\n", lua_tostring(L, -1));
            remove_script(L, script);
        }
        lua_pop(L, 1);
        return;
    } else {
        int returned = lua_gettop(L) - stack_top;
        switch (returned) {
            case 0:
                // no time to reschedule so bail out
                remove_script(L, script);
                break;
            case 2:
                {
                   // sanity check the return types
                   if (lua_type(L, -1) != LUA_TNUMBER) {
                       gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: %s did not return a delay (0x%d)", script->name, lua_type(L, -1));
                       lua_pop(L, 2);
                       remove_script(L, script);
                       return;
                   }
                   if (lua_type(L, -2) != LUA_TFUNCTION) {
                       gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: %s did not return a function (0x%d)", script->name, lua_type(L, -2));
                       lua_pop(L, 2);
                       remove_script(L, script);
                       return;
                   }

                   // types match the expectations, go ahead and reschedule
                   script->next_run_ms = AP_HAL::millis64() + (uint64_t)luaL_checknumber(L, -1);
                   lua_pop(L, 1);
                   int old_ref = script->lua_ref;
                   script->lua_ref = luaL_ref(L, LUA_REGISTRYINDEX);
                   luaL_unref(L, LUA_REGISTRYINDEX, old_ref);
                   reschedule_script(script);
                   break;
                }
            default:
                {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: %s returned bad result count (%d)", script->name, returned);
                    remove_script(L, script);
                    // pop all the results we got that we didn't expect
                    lua_pop(L, returned);
                    break;
                 }
         }
     }
}

void lua_scripts::remove_script(lua_State *L, script_info *script) {
    if (script == nullptr) {
        return;
    }

    // ensure that the script isn't in the loaded list for any reason
    if (scripts == nullptr) {
        // nothing to do, already not in the list
    } else if (scripts == script) {
        scripts = script->next;
    } else {
        for(script_info * current = scripts; current->next != nullptr; current = current->next) {
            if (current->next == script) {
                current->next = script->next;
                break;
            }
        }
    }

    if (L != nullptr) {
        // state could be null if we are force killing all scripts
        luaL_unref(L, LUA_REGISTRYINDEX, script->lua_ref);
    }
    hal.util->heap_realloc(_heap, script->name, 0);
    hal.util->heap_realloc(_heap, script, 0);
}

void lua_scripts::reschedule_script(script_info *script) {
    if (script == nullptr) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
       AP_HAL::panic("Lua: Attempted to schedule a null pointer");
#endif // defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
       return;
    }

    script->next = nullptr;
    if (scripts == nullptr) {
        scripts = script;
        return;
    }

    uint64_t next_run_ms = script->next_run_ms;

    if (scripts->next_run_ms > next_run_ms) {
        script->next = scripts;
        scripts = script;
        return;
    }

    script_info *previous = scripts;
    while (previous->next != nullptr) {
        if (previous->next->next_run_ms > next_run_ms) {
            script->next = previous->next;
            previous->next = script;
            return;
        }
        previous = previous->next;
    }

    previous->next = script;
}

void *lua_scripts::_heap;

void *lua_scripts::alloc(void *ud, void *ptr, size_t osize, size_t nsize) {
    (void)ud; (void)osize;  /* not used */
    return hal.util->heap_realloc(_heap, ptr, nsize);
}

void lua_scripts::run(void) {
    if (_heap == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Lua: Unable to allocate a heap");
        return;
    }

    // panic should be hooked first
    if (setjmp(panic_jmp)) {
        if (lua_state != nullptr) {
            lua_close(lua_state); // shutdown the old state
        }
        // remove all the old scheduled scripts
        for (script_info *script = scripts; script != nullptr; script = scripts) {
            remove_script(nullptr, script);
        }
        scripts = nullptr;
        overtime = false;
    }

    lua_state = lua_newstate(alloc, NULL);
    lua_State *L = lua_state;
    if (L == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: Couldn't allocate a lua state");
        return;
    }
    lua_atpanic(L, atpanic);
    luaL_openlibs(L);
    load_lua_bindings(L);

    // load the sandbox creation function
    uint32_t sandbox_size;
    const char *sandbox_data = (const char *)AP_ROMFS::find_decompress("sandbox.lua", sandbox_size);
    if (sandbox_data == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Scripting: Could not find sandbox");
        return;
    }

    if (luaL_dostring(L, sandbox_data)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Scripting: Loading sandbox: %s", lua_tostring(L, -1));
        return;
    }
    AP_ROMFS::free((const uint8_t *)sandbox_data);

    // Scan the filesystem in an appropriate manner and autostart scripts
    load_all_scripts_in_dir(L, SCRIPTING_DIRECTORY);

    while (AP_Scripting::get_singleton()->enabled()) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
        if (lua_gettop(L) != 0) {
            AP_HAL::panic("Lua: Stack should be empty before running scripts");
        }
#endif // defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1

        if (scripts != nullptr) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
              // Sanity check that the scripts list is ordered correctly
              script_info *sanity = scripts;
              while (sanity->next != nullptr) {
                  if (sanity->next_run_ms > sanity->next->next_run_ms) {
                      AP_HAL::panic("Lua: Script tasking order has been violated");
                  }
                  sanity = sanity->next;
              }
#endif // defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1

            // compute delay time
            uint64_t now_ms = AP_HAL::millis64();
            if (now_ms < scripts->next_run_ms) {
                hal.scheduler->delay(scripts->next_run_ms - now_ms);
            }

            if (_debug_level > 1) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "Lua: Running %s", scripts->name);
            }

            const int startMem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);
            const uint32_t loadEnd = AP_HAL::micros();

            run_next_script(L);

            const uint32_t runEnd = AP_HAL::micros();
            const int endMem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);
            if (_debug_level > 1) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "Lua: Time: %u Mem: %d + %d",
                                                    (unsigned int)(runEnd - loadEnd),
                                                    (int)endMem,
                                                    (int)(endMem - startMem));
            }

            // garbage collect after each script, this shouldn't matter, but seems to resolve a memory leak
            lua_gc(L, LUA_GCCOLLECT, 0);

        } else {
            gcs().send_text(MAV_SEVERITY_DEBUG, "Lua: No scripts to run");
            hal.scheduler->delay(10000);
        }

    }
}
