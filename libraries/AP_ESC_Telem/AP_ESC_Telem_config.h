#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <SRV_Channel/SRV_Channel_config.h>
#include <AP_BLHeli/AP_BLHeli_config.h>

#ifndef HAL_WITH_ESC_TELEM
#define HAL_WITH_ESC_TELEM ((NUM_SERVO_CHANNELS > 0) && ((AP_BLHELI_ENABLED || HAL_MAX_CAN_PROTOCOL_DRIVERS)))
#endif
