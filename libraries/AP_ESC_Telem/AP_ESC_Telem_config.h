#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <SRV_Channel/SRV_Channel_config.h>

#ifndef HAL_WITH_ESC_TELEM
#define HAL_WITH_ESC_TELEM ((NUM_SERVO_CHANNELS > 0) && ((HAL_SUPPORT_RCOUT_SERIAL || HAL_MAX_CAN_PROTOCOL_DRIVERS)))
#endif

#ifndef AP_ESC_TELEM_DDS_ENABLED
#define AP_ESC_TELEM_DDS_ENABLED AP_DDS_ENABLED
#endif
