#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

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

int lua_get_current_ref();
