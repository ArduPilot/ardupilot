#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 1
#endif

// set default for HAL_LOGGING_DATAFLASH_ENABLED
#ifndef HAL_LOGGING_DATAFLASH_ENABLED
#define HAL_LOGGING_DATAFLASH_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef HAL_LOGGING_MAVLINK_ENABLED
    #define HAL_LOGGING_MAVLINK_ENABLED HAL_LOGGING_ENABLED
#endif

#ifndef HAL_LOGGING_FILESYSTEM_ENABLED
#define HAL_LOGGING_FILESYSTEM_ENABLED HAL_LOGGING_ENABLED && AP_FILESYSTEM_FILE_WRITING_ENABLED
#endif

#if HAL_LOGGING_DATAFLASH_ENABLED
    #define HAL_LOGGING_BLOCK_ENABLED 1
#else
    #define HAL_LOGGING_BLOCK_ENABLED 0
#endif

#if HAL_LOGGING_FILESYSTEM_ENABLED

#if !defined (HAL_BOARD_LOG_DIRECTORY)
#error Need HAL_BOARD_LOG_DIRECTORY for filesystem backend support
#endif

#endif

#ifndef HAL_LOGGER_FILE_CONTENTS_ENABLED
#define HAL_LOGGER_FILE_CONTENTS_ENABLED HAL_LOGGING_FILESYSTEM_ENABLED
#endif

// range of IDs to allow for new messages during replay. It is very
// useful to be able to add new messages during a replay, but we need
// to avoid colliding with existing messages
#define REPLAY_LOG_NEW_MSG_MAX 230
#define REPLAY_LOG_NEW_MSG_MIN 220

#include <AC_Fence/AC_Fence_config.h>
#define HAL_LOGGER_FENCE_ENABLED AP_FENCE_ENABLED
