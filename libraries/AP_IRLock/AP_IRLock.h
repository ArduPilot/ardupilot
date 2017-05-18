/*
 * AP_IRLock.h
 *
 *  Created on: Nov 10, 2014
 *      Author: MLandes
 */

// @file AP_IRLock.h
// @brief Catch-all headerthat defines all supported irlock classes.

#include "IRLock.h"
#include "AP_IRLock_I2C.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_IRLock_SITL.h"
#endif
