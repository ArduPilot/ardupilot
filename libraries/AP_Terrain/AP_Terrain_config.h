#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

#ifndef AP_TERRAIN_AVAILABLE
#define AP_TERRAIN_AVAILABLE AP_FILESYSTEM_FILE_READING_ENABLED
#endif
