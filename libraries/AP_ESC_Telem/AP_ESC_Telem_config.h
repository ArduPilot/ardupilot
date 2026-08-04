#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <SRV_Channel/SRV_Channel_config.h>

/*
  Bidirectional DShot is counted here because delivering telemetry is the whole
  point of it. On STM32 that made no difference - a board with bidir DShot also
  has serial passthrough - but RP2350 drives DShot from the PIO and supports
  neither passthrough nor CAN, so the eRPM was being decoded and then dropped
  because every publish site is compiled out.
 */
#ifndef HAL_WITH_ESC_TELEM
#define HAL_WITH_ESC_TELEM ((NUM_SERVO_CHANNELS > 0) && (HAL_SUPPORT_RCOUT_SERIAL || HAL_MAX_CAN_PROTOCOL_DRIVERS || defined(HAL_WITH_BIDIR_DSHOT)))
#endif

#ifndef AP_EXTENDED_ESC_TELEM_ENABLED
#define AP_EXTENDED_ESC_TELEM_ENABLED HAL_ENABLE_DRONECAN_DRIVERS
#endif

#if AP_EXTENDED_ESC_TELEM_ENABLED && !HAL_WITH_ESC_TELEM
    #error "AP_EXTENDED_ESC_TELEM_ENABLED requires HAL_WITH_ESC_TELEM"
#endif
