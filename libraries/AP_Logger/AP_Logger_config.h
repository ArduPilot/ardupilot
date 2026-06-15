#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <GCS_MAVLink/GCS_config.h>

#ifndef AP_LOGGER_ENCRYPTION_ENABLED
#define AP_LOGGER_ENCRYPTION_ENABLED 0
#endif

#ifndef AP_LOGGER_ENCRYPTION_PUBLIC_KEY
// RFC7748 X25519 test public key used for SITL/development only. Production
// builds must provision a fleet or device public key at build time.
#define AP_LOGGER_ENCRYPTION_PUBLIC_KEY { \
    0x85, 0x20, 0xf0, 0x09, 0x89, 0x30, 0xa7, 0x54, \
    0x74, 0x8b, 0x7d, 0xdc, 0xb4, 0x3e, 0xf7, 0x5a, \
    0x0d, 0xbf, 0x3a, 0x0d, 0x26, 0x38, 0x1a, 0xf4, \
    0xeb, 0xa4, 0xa9, 0x8e, 0xaa, 0x9b, 0x4e, 0x6a  \
}
#endif

#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 1
#endif

#ifndef HAL_LOGGING_BACKEND_DEFAULT_ENABLED
#define HAL_LOGGING_BACKEND_DEFAULT_ENABLED HAL_LOGGING_ENABLED
#endif

// set default for HAL_LOGGING_DATAFLASH_ENABLED
#ifndef HAL_LOGGING_DATAFLASH_ENABLED
#define HAL_LOGGING_DATAFLASH_ENABLED HAL_LOGGING_BACKEND_DEFAULT_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef HAL_LOGGING_MAVLINK_ENABLED
    #define HAL_LOGGING_MAVLINK_ENABLED HAL_LOGGING_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
#endif

#ifndef HAL_LOGGING_FILESYSTEM_ENABLED
#define HAL_LOGGING_FILESYSTEM_ENABLED HAL_LOGGING_BACKEND_DEFAULT_ENABLED && AP_FILESYSTEM_FILE_WRITING_ENABLED
#endif

#if HAL_LOGGING_DATAFLASH_ENABLED
    #define HAL_LOGGING_BLOCK_ENABLED HAL_LOGGING_ENABLED
#else
    #define HAL_LOGGING_BLOCK_ENABLED 0
#endif

#ifndef HAL_LOGGING_FLASH_W25NXX_ENABLED
#define HAL_LOGGING_FLASH_W25NXX_ENABLED HAL_LOGGING_BLOCK_ENABLED
#endif

#ifndef HAL_LOGGING_FLASH_JEDEC_ENABLED
#define HAL_LOGGING_FLASH_JEDEC_ENABLED HAL_LOGGING_BLOCK_ENABLED
#endif

#if HAL_LOGGING_FILESYSTEM_ENABLED

#if !defined (HAL_BOARD_LOG_DIRECTORY)
#error Need HAL_BOARD_LOG_DIRECTORY for filesystem backend support
#endif

#endif

#ifndef HAL_LOGGER_FILE_CONTENTS_ENABLED
#define HAL_LOGGER_FILE_CONTENTS_ENABLED HAL_LOGGING_FILESYSTEM_ENABLED && !AP_FILESYSTEM_LITTLEFS_ENABLED
#endif

#if AP_FILESYSTEM_LITTLEFS_ENABLED && HAL_LOGGING_BLOCK_ENABLED
#error LittleFS requires block logging to be disabled
#endif

// range of IDs to allow for new messages during replay. It is very
// useful to be able to add new messages during a replay, but we need
// to avoid colliding with existing messages
#define REPLAY_LOG_NEW_MSG_MAX 230
#define REPLAY_LOG_NEW_MSG_MIN 220

#include <AC_Fence/AC_Fence_config.h>
#define HAL_LOGGER_FENCE_ENABLED HAL_LOGGING_ENABLED && AP_FENCE_ENABLED

#include <AP_Rally/AP_Rally_config.h>
#define HAL_LOGGER_RALLY_ENABLED HAL_LOGGING_ENABLED && HAL_RALLY_ENABLED
