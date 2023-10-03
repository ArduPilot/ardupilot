#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <SRV_Channel/SRV_Channel_config.h>

#ifndef HAL_WITH_ESC_TELEM
#define HAL_WITH_ESC_TELEM ((NUM_SERVO_CHANNELS > 0) && ((HAL_SUPPORT_RCOUT_SERIAL || HAL_MAX_CAN_PROTOCOL_DRIVERS) && !defined(HAL_BUILD_AP_PERIPH)))
#endif

#ifndef AP_ESC_TELEM_BACKEND_DEFAULT_ENABLED
#define AP_ESC_TELEM_BACKEND_DEFAULT_ENABLED (HAL_WITH_ESC_TELEM && BOARD_FLASH_SIZE > 1024)
#endif

// enable the driver itself if the ESC telem backend is enabled.  An
// AP_Periph may enable this without the ESC Telem backend being
// available so it can parse data directly from the port
#ifndef AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED
#define AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED AP_ESC_TELEM_BACKEND_DEFAULT_ENABLED
#endif

// enable the driver itself if the ESC telem backend is enabled.  An
// AP_Periph may enable this without the ESC Telem backend being
// available so it can parse data directly from the port
#ifndef AP_HOBBYWING_PLATINUM_V4_ENABLED
#define AP_HOBBYWING_PLATINUM_V4_ENABLED AP_ESC_TELEM_BACKEND_DEFAULT_ENABLED
#endif

// enable the driver itself if the ESC telem backend is enabled.  An
// AP_Periph may enable this without the ESC Telem backend being
// available so it can parse data directly from the port
#ifndef AP_HOBBYWING_XROTOR_V4_ENABLED
#define AP_HOBBYWING_XROTOR_V4_ENABLED AP_ESC_TELEM_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_HOBBYWING_DATALINK_ENABLED
#define AP_HOBBYWING_DATALINK_ENABLED AP_ESC_TELEM_BACKEND_DEFAULT_ENABLED
#endif

// base class for these backends; remove it if not required or it
// takes up space on (e.g.) 1MB boards
#ifndef AP_HOBBYWING_ESC_ENABLED
#define AP_HOBBYWING_ESC_ENABLED (              \
    AP_HOBBYWING_DATALINK_ENABLED ||            \
    AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED ||     \
    AP_HOBBYWING_PLATINUM_V4_ENABLED ||         \
    AP_HOBBYWING_XROTOR_V4_ENABLED              \
        )
#endif

#ifndef AP_ESC_TELEM_MAX_PROTOCOL_GROUPS
#if AP_HOBBYWING_ESC_ENABLED
#define AP_ESC_TELEM_MAX_PROTOCOL_GROUPS 2
#else
#define AP_ESC_TELEM_MAX_PROTOCOL_GROUPS 0
#endif
#endif
