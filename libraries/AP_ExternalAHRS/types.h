/**
 * @file types.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 7/3/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include <inttypes.h>

// Define facility bitmasks. Use powers of 2 for unique bits.
#define IS_LOG_FACILITY_NONE       0x0000                          // This facility is ALWAYS enabled (but still followed LOG_LEVEL)
#define IS_LOG_PORT                0x0001
#define IS_LOG_FWUPDATE            ((IS_LOG_PORT << 1))
#define IS_LOG_ISDEVICE            ((IS_LOG_FWUPDATE << 1))
#define IS_LOG_PORT_FACTORY        ((IS_LOG_ISDEVICE << 1))
#define IS_LOG_PORT_MANAGER        ((IS_LOG_PORT_FACTORY << 1))
#define IS_LOG_DEVICE_FACTORY      ((IS_LOG_PORT_MANAGER << 1))
#define IS_LOG_DEVICE_MANAGER      ((IS_LOG_DEVICE_FACTORY << 1))
#define IS_LOG_CHRONO_STATS        ((IS_LOG_DEVICE_MANAGER << 1))
#define IS_LOG_FACILITY_MDNS       ((IS_LOG_CHRONO_STATS << 1))
#define IS_LOG_FACILITY_ALL        0xFFFF

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,         //!< use this to disable all log messages
    IS_LOG_LEVEL_ERROR = 1,         //!< errors - that that prevents the application from proceeding as normal
    IS_LOG_LEVEL_WARN  = 2,         //!< warnings - important to now, but we can move on anyway
    IS_LOG_LEVEL_INFO  = 3,         //!< informative for the customer regarding normal operations
    IS_LOG_LEVEL_MORE_INFO = 4,     //!< additional information for the customer regarding normal operations - might be annoying
    IS_LOG_LEVEL_DEBUG = 5,         //!< informative for support & basic troubleshooting
    IS_LOG_LEVEL_MORE_DEBUG = 6,    //!< informative for advanced troubleshooting - maybe annoying
    IS_LOG_LEVEL_BOMBASTIC = 7      //!< excessive on all but the most extreme cases - will definitely be annoying
} eLogLevel;

typedef enum {
    IS_BL_TYPE_NONE = 0,
    IS_BL_TYPE_SAMBA,
    IS_BL_TYPE_ISB,
    IS_BL_TYPE_APP,
    IS_BL_TYPE_DFU,
} eBootLoaderType;

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
