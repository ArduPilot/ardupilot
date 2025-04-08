#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_ICENGINE_ENABLED
#define AP_ICENGINE_ENABLED 1
#endif

/*
  optional TCA9554 I2C for starter control
 */
#ifndef AP_ICENGINE_TCA9554_STARTER_ENABLED
// enable on SITL by default to ensure code is built
#define AP_ICENGINE_TCA9554_STARTER_ENABLED AP_ICENGINE_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
