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
#include "AP_Scripting.h"
#include <AP_Logger/AP_Logger.h>

#include <AP_Scripting/lua_generated_bindings.h>

#define DISABLE_INTERRUPTS_FOR_SCRIPT_RUN 0

extern const AP_HAL::HAL& hal;

bool lua_scripts::overtime;
jmp_buf lua_scripts::panic_jmp;
char *lua_scripts::error_msg_buf;
uint8_t lua_scripts::print_error_count;
uint32_t lua_scripts::last_print_ms;

lua_scripts::lua_scripts(const AP_Int32 &vm_steps, const AP_Int32 &heap_size, const AP_Int8 &debug_options, struct AP_Scripting::terminal_s &_terminal)
    : _vm_steps(vm_steps),
      _debug_options(debug_options),
     terminal(_terminal) {
    _heap = hal.util->allocate_heap_memory(heap_size);
}

lua_scripts::~lua_scripts() {
    free(_heap);
}

void lua_scripts::hook(lua_State *L, lua_Debug *ar) {
    lua_scripts::overtime = true;

    // we need to aggressively bail out as we are over time
    // so we will aggressively trap errors until we clear out
    lua_sethook(L, hook, LUA_MASKCOUNT, 1);

    luaL_error(L, "Exceeded CPU time");
}

void lua_scripts::print_error(MAV_SEVERITY severity) {
    if (error_msg_buf == nullptr) {
        return;
    }
    last_print_ms = AP_HAL::millis();
    GCS_SEND_TEXT(severity, "Lua: %s", error_msg_buf);
}

void lua_scripts::set_and_print_new_error_message(MAV_SEVERITY severity, const char *fmt, ...) {
    // reset buffer and print count
    print_error_count = 0;
    if (error_msg_buf) {
        hal.util->heap_realloc(_heap, error_msg_buf, 0);
        error_msg_buf = nullptr;
    }

    // generate va_list and create a copy
    va_list arg_list, arg_list_copy;
    va_start(arg_list, fmt);
    va_copy(arg_list_copy, arg_list);

    // dry run to work out the required length
    int len = hal.util->vsnprintf(nullptr, 0, fmt, arg_list_copy);

    // finished with copy
    va_end(arg_list_copy);

    if (len <= 0) {
        // nothing to print, something has gone wrong
        va_end(arg_list);
        return;
    }

    // allocate buffer on scripting heap
    error_msg_buf = (char *)hal.util->heap_realloc(_heap, nullptr, len+1);
    if (!error_msg_buf) {
        // allocation failed
        va_end(arg_list);
        return;
    }

    // do actual print to buffer and clear va list
    hal.util->vsnprintf(error_msg_buf, len+1, fmt, arg_list);
    va_end(arg_list);

    // print to cosole and GCS
    DEV_PRINTF("Lua: %s\n", error_msg_buf);
    print_error(severity);
}

int lua_scripts::atpanic(lua_State *L) {
    set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Panic: %s", lua_tostring(L, -1));
    longjmp(panic_jmp, 1);
    return 0;
}

lua_scripts::script_info *lua_scripts::load_script(lua_State *L, char *filename) {
    if (int error = luaL_loadfile(L, filename)) {
        switch (error) {
            case LUA_ERRSYNTAX:
                set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Error: %s", lua_tostring(L, -1));
                lua_pop(L, lua_gettop(L));
                return nullptr;
            case LUA_ERRMEM:
                set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Insufficent memory loading %s", filename);
                lua_pop(L, lua_gettop(L));
                return nullptr;
            case LUA_ERRFILE:
                set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Unable to load the file: %s", lua_tostring(L, -1));
                lua_pop(L, lua_gettop(L));
                return nullptr;
            default:
                set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Unknown error (%d) loading %s", error, filename);
                lua_pop(L, lua_gettop(L));
                return nullptr;
        }
    }

    script_info *new_script = (script_info *)hal.util->heap_realloc(_heap, nullptr, sizeof(script_info));
    if (new_script == nullptr) {
        // No memory, shouldn't happen, we even attempted to do a GC
        set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "Insufficent memory loading %s", filename);
        lua_pop(L, 1); // we can't use the function we just loaded, so ditch it
        return nullptr;
    }

    new_script->name = filename;
    new_script->next = nullptr;

    create_sandbox(L);
    lua_setupvalue(L, -2, 1);

    new_script->lua_ref = luaL_ref(L, LUA_REGISTRYINDEX);   // cache the reference
    new_script->next_run_ms = AP_HAL::millis64() - 1; // force the script to be stale

    return new_script;
}

void lua_scripts::create_sandbox(lua_State *L) {
    lua_newtable(L);
    luaopen_base_sandbox(L);
    lua_pushstring(L, "math");
    luaopen_math(L);
    lua_settable(L, -3);
    lua_pushstring(L, "table");
    luaopen_table(L);
    lua_settable(L, -3);
    lua_pushstring(L, "string");
    luaopen_string(L);
    lua_settable(L, -3);
    lua_pushstring(L, "io");
    luaopen_io(L);
    lua_settable(L, -3);
    lua_pushstring(L, "utf8");
    luaopen_utf8(L);
    lua_settable(L, -3);
    load_generated_sandbox(L);

}

void lua_scripts::load_all_scripts_in_dir(lua_State *L, const char *dirname) {
    if (dirname == nullptr) {
        return;
    }

    auto *d = AP::FS().opendir(dirname);
    if (d == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Lua: open directory (%s) failed", dirname);
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

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
        if ((_debug_options.get() & uint8_t(DebugLevel::SUPPRESS_SCRIPT_LOG)) == 0) {
            AP::logger().log_file_content(filename);
        }
#endif
    }
    AP::FS().closedir(d);
}

void lua_scripts::reset_loop_overtime(lua_State *L) {
    overtime = false;
    // reset the hook to clear the counter
    const int32_t vm_steps = MAX(_vm_steps, 1000);
    lua_sethook(L, hook, LUA_MASKCOUNT, vm_steps);
}

void lua_scripts::run_next_script(lua_State *L) {
    if (scripts == nullptr) {
#if defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
        AP_HAL::panic("Lua: Attempted to run a script without any scripts queued");
#endif // defined(AP_SCRIPTING_CHECKS) && AP_SCRIPTING_CHECKS >= 1
        return;
    }

    uint64_t start_time_ms = AP_HAL::millis64();
    // strip the selected script out of the list
    script_info *script = scripts;
    scripts = script->next;

    // reset the hook to clear the counter
    reset_loop_overtime(L);

    // store top of stack so we can calculate the number of return values
    int stack_top = lua_gettop(L);

    // pop the function to the top of the stack
    lua_rawgeti(L, LUA_REGISTRYINDEX, script->lua_ref);

    if(lua_pcall(L, 0, LUA_MULTRET, 0)) {
        if (overtime) {
            // script has consumed an excessive amount of CPU time
            set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "%s exceeded time limit", script->name);
        } else {
            set_and_print_new_error_message(MAV_SEVERITY_INFO, "%s", lua_tostring(L, -1));
        }
        remove_script(L, script);
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
                        set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "%s did not return a delay (0x%d)", script->name, lua_type(L, -1));
                        lua_pop(L, 2);
                        remove_script(L, script);
                        return;
                    }
                    if (lua_type(L, -2) != LUA_TFUNCTION) {
                        set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "%s did not return a function (0x%d)", script->name, lua_type(L, -2));
                        lua_pop(L, 2);
                        remove_script(L, script);
                        return;
                    }

                    // types match the expectations, go ahead and reschedule
                    script->next_run_ms = start_time_ms + (uint64_t)luaL_checknumber(L, -1);
                    lua_pop(L, 1);
                    int old_ref = script->lua_ref;
                    script->lua_ref = luaL_ref(L, LUA_REGISTRYINDEX);
                    luaL_unref(L, LUA_REGISTRYINDEX, old_ref);
                    reschedule_script(script);
                    break;
                }
            default:
                {
                    set_and_print_new_error_message(MAV_SEVERITY_CRITICAL, "%s returned bad result count (%d)", script->name, returned);
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

void lua_scripts::repl_cleanup (void) {
    if (terminal.session) {
        terminal.session = false;
        if (terminal.output_fd != -1) {
            AP::FS().close(terminal.output_fd);
            terminal.output_fd = -1;
            AP::FS().unlink(REPL_DIRECTORY "/in");
            AP::FS().unlink(REPL_DIRECTORY "/out");
            AP::FS().unlink(REPL_DIRECTORY);
        }
    }
}

void lua_scripts::run(void) {
    bool succeeded_initial_load = false;

    if (_heap == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Lua: Unable to allocate a heap");
        return;
    }

    // panic should be hooked first
    if (setjmp(panic_jmp)) {
        if (!succeeded_initial_load) {
            return;
        }
        if (lua_state != nullptr) {
            lua_close(lua_state); // shutdown the old state
        }
        // remove all the old scheduled scripts
        for (script_info *script = scripts; script != nullptr; script = scripts) {
            remove_script(nullptr, script);
        }
        scripts = nullptr;
        overtime = false;
        // end any open REPL sessions
        repl_cleanup();
    }

    lua_state = lua_newstate(alloc, NULL);
    lua_State *L = lua_state;
    if (L == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: Couldn't allocate a lua state");
        return;
    }
    lua_atpanic(L, atpanic);
    load_generated_bindings(L);

    // Scan the filesystem in an appropriate manner and autostart scripts
    // Skip those directores disabled with SCR_DIR_DISABLE param
    uint16_t dir_disable = AP_Scripting::get_singleton()->get_disabled_dir();
    bool loaded = false;
    if ((dir_disable & uint16_t(AP_Scripting::SCR_DIR::SCRIPTS)) == 0) {
        load_all_scripts_in_dir(L, SCRIPTING_DIRECTORY);
        loaded = true;
    }
    if ((dir_disable & uint16_t(AP_Scripting::SCR_DIR::ROMFS)) == 0) {
        load_all_scripts_in_dir(L, "@ROMFS/scripts");
        loaded = true;
    }
    if (!loaded) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Lua: All directory's disabled see SCR_DIR_DISABLE");
    }

#ifndef __clang_analyzer__
    succeeded_initial_load = true;
#endif // __clang_analyzer__

    while (AP_Scripting::get_singleton()->should_run()) {
        // handle terminal data if we have any
        if (terminal.session) {
            doREPL(L);
            continue;
        }

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

            if ((_debug_options.get() & uint8_t(DebugLevel::RUNTIME_MSG)) != 0) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "Lua: Running %s", scripts->name);
            }
            // copy name for logging, cant do it after as script reschedule moves the pointers
            const char * script_name = scripts->name;

#if DISABLE_INTERRUPTS_FOR_SCRIPT_RUN
            void *istate = hal.scheduler->disable_interrupts_save();
#endif

            const int startMem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);
            const uint32_t loadEnd = AP_HAL::micros();

            run_next_script(L);

            const uint32_t runEnd = AP_HAL::micros();
            const int endMem = lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);

#if DISABLE_INTERRUPTS_FOR_SCRIPT_RUN
            hal.scheduler->restore_interrupts(istate);
#endif

            if ((_debug_options.get() & uint8_t(DebugLevel::RUNTIME_MSG)) != 0) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "Lua: Time: %u Mem: %d + %d",
                                                    (unsigned int)(runEnd - loadEnd),
                                                    (int)endMem,
                                                    (int)(endMem - startMem));
            }
            if ((_debug_options.get() & uint8_t(DebugLevel::LOG_RUNTIME)) != 0) {
                struct log_Scripting pkt{
                    LOG_PACKET_HEADER_INIT(LOG_SCRIPTING_MSG),
                    time_us      : AP_HAL::micros64(),
                    name         : {},
                    run_time     : runEnd - loadEnd,
                    total_mem    : endMem,
                    run_mem      : endMem - startMem,
                };
                const char * name_short = strrchr(script_name, '/');
                if ((strlen(script_name) > sizeof(pkt.name)) && (name_short != nullptr)) {
                    strncpy_noterm(pkt.name, name_short+1, sizeof(pkt.name));
                } else {
                    strncpy_noterm(pkt.name, script_name, sizeof(pkt.name));
                }
                AP::logger().WriteBlock(&pkt, sizeof(pkt));
            }


            // garbage collect after each script, this shouldn't matter, but seems to resolve a memory leak
            lua_gc(L, LUA_GCCOLLECT, 0);

        } else {
            if ((_debug_options.get() & uint8_t(DebugLevel::NO_SCRIPTS_TO_RUN)) != 0) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "Lua: No scripts to run");
            }
            hal.scheduler->delay(1000);
        }

        // re-print the latest error message every 10 seconds 10 times
        const uint8_t error_prints = 10;
        if ((print_error_count < error_prints) && (AP_HAL::millis() - last_print_ms > 10000)) {
            print_error(MAV_SEVERITY_DEBUG);
            print_error_count++;
            if ((print_error_count >= error_prints) && (error_msg_buf != nullptr)) {
                hal.util->heap_realloc(_heap, error_msg_buf, 0);
                error_msg_buf = nullptr;
            }
        }
    }

    // make sure all scripts have been removed
    while (scripts != nullptr) {
        remove_script(lua_state, scripts);
    }

    if (lua_state != nullptr) {
        lua_close(lua_state); // shutdown the old state
        lua_state = nullptr;
    }

    if (error_msg_buf != nullptr) {
        hal.util->heap_realloc(_heap, error_msg_buf, 0);
        error_msg_buf = nullptr;
    }
}
