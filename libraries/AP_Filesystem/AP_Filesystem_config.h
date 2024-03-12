#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_Mission/AP_Mission_config.h>

// backends:

#ifndef AP_FILESYSTEM_ESP32_ENABLED
#define AP_FILESYSTEM_ESP32_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_ESP32)
#endif

#ifndef AP_FILESYSTEM_FATFS_ENABLED
#define AP_FILESYSTEM_FATFS_ENABLED HAL_OS_FATFS_IO
#endif

#ifndef AP_FILESYSTEM_MISSION_ENABLED
#define AP_FILESYSTEM_MISSION_ENABLED AP_MISSION_ENABLED
#endif

#ifndef AP_FILESYSTEM_PARAM_ENABLED
#define AP_FILESYSTEM_PARAM_ENABLED 1
#endif

#ifndef AP_FILESYSTEM_POSIX_ENABLED
#define AP_FILESYSTEM_POSIX_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#endif

#ifndef AP_FILESYSTEM_ROMFS_ENABLED
#define AP_FILESYSTEM_ROMFS_ENABLED defined(HAL_HAVE_AP_ROMFS_EMBEDDED_H)
#endif

#ifndef AP_FILESYSTEM_SYS_ENABLED
#define AP_FILESYSTEM_SYS_ENABLED 1
#endif

// AP_FILESYSTEM_FILE_WRITING_ENABLED is true if you could expect to
// be able to open and write a non-virtual file.  Notably this
// excludes virtual files like SYSFS, and the magic param/mission
// upload targets, and also excludes ROMFS (where you can read but not
// write!)
#ifndef AP_FILESYSTEM_FILE_WRITING_ENABLED
#define AP_FILESYSTEM_FILE_WRITING_ENABLED (AP_FILESYSTEM_ESP32_ENABLED || AP_FILESYSTEM_FATFS_ENABLED || AP_FILESYSTEM_POSIX_ENABLED)
#endif

// AP_FILESYSTEM_FILE_READING_ENABLED is true if you could expect to
// be able to open and read a non-virtual file.  Notably this excludes
// virtual files like SYSFS, and the magic param/mission upload targets.
#ifndef AP_FILESYSTEM_FILE_READING_ENABLED
#define AP_FILESYSTEM_FILE_READING_ENABLED (AP_FILESYSTEM_FILE_WRITING_ENABLED || AP_FILESYSTEM_ROMFS_ENABLED)
#endif

#ifndef AP_FILESYSTEM_SYS_FLASH_ENABLED
#define AP_FILESYSTEM_SYS_FLASH_ENABLED CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#endif
