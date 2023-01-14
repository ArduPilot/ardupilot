#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <SRV_Channel/SRV_Channel_config.h>

#ifndef HAL_WITH_ESC_TELEM
#define HAL_WITH_ESC_TELEM ((NUM_SERVO_CHANNELS > 0) && ((HAL_SUPPORT_RCOUT_SERIAL || HAL_MAX_CAN_PROTOCOL_DRIVERS)))
#endif

#ifndef AP_EXTENDED_ESC_TELEM_ENABLED
#define AP_EXTENDED_ESC_TELEM_ENABLED HAL_ENABLE_DRONECAN_DRIVERS
#endif

#if AP_EXTENDED_ESC_TELEM_ENABLED && !HAL_WITH_ESC_TELEM
    #error "AP_EXTENDED_ESC_TELEM_ENABLED requires HAL_WITH_ESC_TELEM"
#endif

// enable the ESC Telemetry backend if ESC Telem is in use:
#ifndef AP_ESC_TELEM_HOBBYWING_ENABLED
#define AP_ESC_TELEM_HOBBYWING_ENABLED (HAL_WITH_ESC_TELEM && BOARD_FLASH_SIZE > 1024)
#endif

// enable the driver itself if the ESC telem backend is enabled.  An
// AP_Periph may enable this without the ESC Telem backend being
// available so it can parse data directly from the port
#ifndef AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED
#define AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED AP_ESC_TELEM_HOBBYWING_ENABLED
#endif

// enable the driver itself if the ESC telem backend is enabled.  An
// AP_Periph may enable this without the ESC Telem backend being
// available so it can parse data directly from the port
#ifndef AP_HOBBYWING_PLATINUM_V4_ENABLED
#define AP_HOBBYWING_PLATINUM_V4_ENABLED AP_ESC_TELEM_HOBBYWING_ENABLED
#endif

// enable the driver itself if the ESC telem backend is enabled.  An
// AP_Periph may enable this without the ESC Telem backend being
// available so it can parse data directly from the port
#ifndef AP_HOBBYWING_XROTOR_V4_ENABLED
#define AP_HOBBYWING_XROTOR_V4_ENABLED AP_ESC_TELEM_HOBBYWING_ENABLED
#endif
