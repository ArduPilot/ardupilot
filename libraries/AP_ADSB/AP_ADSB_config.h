#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include <GCS_MAVLink/GCS_config.h>

#ifndef HAL_ADSB_ENABLED
#define HAL_ADSB_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

#ifndef HAL_ADSB_BACKEND_DEFAULT_ENABLED
#define HAL_ADSB_BACKEND_DEFAULT_ENABLED HAL_ADSB_ENABLED
#endif

#ifndef HAL_ADSB_UAVIONIX_MAVLINK_ENABLED
#define HAL_ADSB_UAVIONIX_MAVLINK_ENABLED HAL_ADSB_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
#endif

#ifndef HAL_ADSB_SAGETECH_ENABLED
#define HAL_ADSB_SAGETECH_ENABLED HAL_ADSB_BACKEND_DEFAULT_ENABLED
#endif

#ifndef HAL_ADSB_UCP_ENABLED
#define HAL_ADSB_UCP_ENABLED HAL_ADSB_BACKEND_DEFAULT_ENABLED
#endif

#ifndef HAL_ADSB_UCP_SET_CONFIG
    // NOTE: this feature is disabled by default because it is only meant to be used only by certified aircraft maintenance
    // personnel and not by your typical pilot. If used incorrectly this may void the TSO certification of the hardware
    #define HAL_ADSB_UCP_SET_CONFIG 0
#endif

#ifndef HAL_ADSB_SAGETECH_MXS_ENABLED
    // this feature is only enabled by default by select hardware
    #define HAL_ADSB_SAGETECH_MXS_ENABLED HAL_ADSB_BACKEND_DEFAULT_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL
#endif
