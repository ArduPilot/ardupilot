#pragma once

// board-specific values for this subsystem are generated into a hwdef
// fragment so they only enter the include closure of code using them
#if __has_include(<hwdef_storage.h>)
#include <hwdef_storage.h>
#endif

#ifndef HAL_STORAGE_FLASH_PAGE_ENABLED
#define HAL_STORAGE_FLASH_PAGE_ENABLED 0
#endif

#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_Filesystem/AP_Filesystem_config.h>

#ifndef AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED
#define AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED AP_FILESYSTEM_FILE_READING_ENABLED
#endif

#ifndef FORCE_APJ_DEFAULT_PARAMETERS
#define FORCE_APJ_DEFAULT_PARAMETERS 0
#endif
