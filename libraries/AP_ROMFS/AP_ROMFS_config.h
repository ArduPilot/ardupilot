#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// true when the build has embedded files in ROMFS.  The build system
// decides this, so it is not a build option
#ifndef AP_ROMFS_ENABLED
#define AP_ROMFS_ENABLED 0
#endif
