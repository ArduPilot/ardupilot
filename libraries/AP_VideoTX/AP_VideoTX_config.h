#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_OSD/AP_OSD_config.h>
#include <AP_MSP/AP_MSP_config.h>

#ifndef AP_VIDEOTX_ENABLED
#define AP_VIDEOTX_ENABLED 1
#endif

#ifndef AP_TRAMP_ENABLED
#define AP_TRAMP_ENABLED AP_VIDEOTX_ENABLED && OSD_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB>1024
#endif

#ifndef AP_SMARTAUDIO_ENABLED
#define AP_SMARTAUDIO_ENABLED AP_VIDEOTX_ENABLED
#endif

// MSP VTX control needs both VideoTX and the MSP layer that implements it;
// default to >1MB boards as it is non-essential, and force it on for FPV boards
// via minimize_fpv_osd.inc
#ifndef AP_MSP_VIDEOTX_ENABLED
#define AP_MSP_VIDEOTX_ENABLED (AP_VIDEOTX_ENABLED && HAL_MSP_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

