#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// backends:

#ifndef AP_FILESYSTEM_ESP32_ENABLED
#define AP_FILESYSTEM_ESP32_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_ESP32)
#endif

#ifndef AP_FILESYSTEM_FATFS_ENABLED
#define AP_FILESYSTEM_FATFS_ENABLED HAL_OS_FATFS_IO
#endif

#ifndef AP_FILESYSTEM_LITTLEFS_ENABLED
#define AP_FILESYSTEM_LITTLEFS_ENABLED HAL_OS_LITTLEFS_IO
#endif

#ifndef AP_FILESYSTEM_PARAM_ENABLED
#define AP_FILESYSTEM_PARAM_ENABLED 1
#endif

#ifndef AP_FILESYSTEM_POSIX_ENABLED
#define AP_FILESYSTEM_POSIX_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_QURT)
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
#define AP_FILESYSTEM_FILE_WRITING_ENABLED (AP_FILESYSTEM_ESP32_ENABLED || AP_FILESYSTEM_FATFS_ENABLED || AP_FILESYSTEM_LITTLEFS_ENABLED || AP_FILESYSTEM_POSIX_ENABLED)
#endif

// AP_FILESYSTEM_FILE_READING_ENABLED is true if you could expect to
// be able to open and read a non-virtual file.  Notably this excludes
// virtual files like SYSFS, and the magic param/mission upload targets.
#ifndef AP_FILESYSTEM_FILE_READING_ENABLED
#define AP_FILESYSTEM_FILE_READING_ENABLED (AP_FILESYSTEM_FILE_WRITING_ENABLED || AP_FILESYSTEM_ROMFS_ENABLED || AP_FILESYSTEM_SYS_ENABLED || AP_FILESYSTEM_PARAM_ENABLED)
#endif

#ifndef AP_FILESYSTEM_SYS_FLASH_ENABLED
#define AP_FILESYSTEM_SYS_FLASH_ENABLED CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#endif

#ifndef AP_FILESYSTEM_MISSION_ENABLED
#include <AP_Mission/AP_Mission_config.h>
#define AP_FILESYSTEM_MISSION_ENABLED AP_MISSION_ENABLED
#endif

// true when AP_Filesystem.cpp picks LittleFS as the local filesystem.  an
// alias can't sit on it; see AP_Filesystem.cpp
#define AP_FILESYSTEM_LOCAL_IS_LITTLEFS (AP_FILESYSTEM_LITTLEFS_ENABLED && !AP_FILESYSTEM_FATFS_ENABLED && !AP_FILESYSTEM_ESP32_ENABLED)

// @MAV_LOG is an alias for the directory this board writes its logs to,
// where those logs are written to a filesystem
#ifndef AP_FILESYSTEM_MAVLOG_ENABLED
#define AP_FILESYSTEM_MAVLOG_ENABLED (AP_FILESYSTEM_FILE_WRITING_ENABLED && !AP_FILESYSTEM_LOCAL_IS_LITTLEFS && defined(HAL_BOARD_LOG_DIRECTORY) && HAL_LOGGING_FILESYSTEM_ENABLED)
#endif  // AP_FILESYSTEM_MAVLOG_ENABLED

// only @MAV_LOG uses the alias support
#ifndef AP_FILESYSTEM_ALIAS_ENABLED
#define AP_FILESYSTEM_ALIAS_ENABLED AP_FILESYSTEM_MAVLOG_ENABLED
#endif  // AP_FILESYSTEM_ALIAS_ENABLED

// longest alias path: root + '/' + 238 byte FTP request + '/' + 255 byte name
#ifndef AP_FILESYSTEM_ALIAS_PATH_MAX
#define AP_FILESYSTEM_ALIAS_PATH_MAX 544
#endif  // AP_FILESYSTEM_ALIAS_PATH_MAX

// last, for HAL_LOGGING_FILESYSTEM_ENABLED: AP_Logger_config.h includes this
// header, so nothing above may test that value, only name it
#include <AP_Logger/AP_Logger_config.h>
