#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// true when the build has embedded files in ROMFS.  The build system
// decides this, so it is not a build option
#ifndef AP_ROMFS_ENABLED
#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_H
#define AP_ROMFS_ENABLED 1
#else
#define AP_ROMFS_ENABLED 0
#endif
#endif
