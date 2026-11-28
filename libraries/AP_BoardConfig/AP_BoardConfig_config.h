#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

// whether the board can be mounted in an orientation other than the
// default one for its type.  The orientation comes from
// AHRS_ORIENTATION, which is looked up by name; a build without that
// parameter simply finds nothing and stays at ROTATION_NONE.
#ifndef AP_BOARDCONFIG_ORIENTATION_ENABLED
#define AP_BOARDCONFIG_ORIENTATION_ENABLED 1
#endif  // AP_BOARDCONFIG_ORIENTATION_ENABLED

#ifndef AP_FEATURE_BOARD_DETECT
#if defined(HAL_CHIBIOS_ARCH_FMUV3)
#define AP_FEATURE_BOARD_DETECT 1
#else
#define AP_FEATURE_BOARD_DETECT 0
#endif
#endif

#ifndef AP_FEATURE_RTSCTS
#define AP_FEATURE_RTSCTS 0
#endif

#ifndef AP_FEATURE_SBUS_OUT
#define AP_FEATURE_SBUS_OUT 0
#endif

#ifndef HAL_WATCHDOG_ENABLED_DEFAULT
#define HAL_WATCHDOG_ENABLED_DEFAULT false
#endif

#ifndef AP_SDCARD_STORAGE_ENABLED
#define AP_SDCARD_STORAGE_ENABLED (HAL_MEM_CLASS >= HAL_MEM_CLASS_1000) && (AP_FILESYSTEM_POSIX_ENABLED || AP_FILESYSTEM_FATFS_ENABLED) && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif
