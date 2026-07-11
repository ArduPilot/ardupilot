#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_OSD/AP_OSD_config.h>
#include <AP_MSP/AP_MSP_config.h>

#ifndef AP_VIDEOTX_ENABLED
#define AP_VIDEOTX_ENABLED 1
#endif

// user-definable VTX band table (Betaflight-style), stored as a compact
// binary blob and edited over MAVLink FTP; when present it supersedes the
// compiled-in default bands. Only boards with 32k of storage have a region to
// store one, so it is enabled there and in SITL (for testing); elsewhere the
// default bands are used.
#ifndef AP_VIDEOTX_TABLE_ENABLED
#define AP_VIDEOTX_TABLE_ENABLED (AP_VIDEOTX_ENABLED && (HAL_STORAGE_SIZE >= 32768 || CONFIG_HAL_BOARD == HAL_BOARD_SITL))
#endif

#ifndef AP_TRAMP_ENABLED
#define AP_TRAMP_ENABLED AP_VIDEOTX_ENABLED && OSD_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB>1024
#endif

#ifndef AP_SMARTAUDIO_ENABLED
#define AP_SMARTAUDIO_ENABLED AP_VIDEOTX_ENABLED
#endif

// AP_MSP_VIDEOTX_ENABLED defaults in AP_MSP/AP_MSP_config.h (included above)

