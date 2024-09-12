#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef SCRIPTING_DIRECTORY
  #if HAL_OS_FATFS_IO
    #define SCRIPTING_DIRECTORY "/APM/scripts"
  #else
    #define SCRIPTING_DIRECTORY "./scripts"
  #endif //HAL_OS_FATFS_IO
#endif // SCRIPTING_DIRECTORY

int lua_get_current_env_ref();
const char* lua_get_modules_path();
void lua_abort(void) __attribute__((noreturn));

