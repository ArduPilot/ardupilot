#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#if AP_SCRIPTING_ENABLED
    #include <AP_Filesystem/AP_Filesystem_config.h>
    #if !AP_FILESYSTEM_FILE_READING_ENABLED
        #error "Scripting requires a filesystem"
    #endif
#endif
