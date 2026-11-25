#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_OSD/AP_OSD_config.h>
#include <AP_MSP/AP_MSP_config.h>

#ifndef AP_VIDEOTX_ENABLED
#define AP_VIDEOTX_ENABLED 1
#endif

// user-definable VTX bands (VTX_BNDn_* parameters), added to or replacing
// the compiled-in factory bands. Off by default to save flash
#ifndef AP_VIDEOTX_TABLE_ENABLED
#define AP_VIDEOTX_TABLE_ENABLED (AP_VIDEOTX_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

// user-definable VTX power table (VTX_PWRTBL_EN, VTX_PWRTBL1 to VTX_PWRTBL6)
// for the VTX power channel, off by default like the user bands
#ifndef AP_VIDEOTX_POWER_TABLE_ENABLED
#define AP_VIDEOTX_POWER_TABLE_ENABLED (AP_VIDEOTX_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_TRAMP_ENABLED
#define AP_TRAMP_ENABLED AP_VIDEOTX_ENABLED && OSD_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB>1024
#endif

#ifndef AP_SMARTAUDIO_ENABLED
#define AP_SMARTAUDIO_ENABLED AP_VIDEOTX_ENABLED
#endif

// AP_MSP_VIDEOTX_ENABLED defaults in AP_MSP/AP_MSP_config.h (included above)

