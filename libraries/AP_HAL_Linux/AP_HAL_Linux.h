
#ifndef __AP_HAL_LINUX_H__
#define __AP_HAL_LINUX_H__

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

/**
 * Umbrella header for AP_HAL_Linux module.
 * The module header exports singleton instances which must conform the
 * AP_HAL::HAL interface. It may only expose implementation details (class
 * names, headers) via the Linux namespace.
 * The class implementing AP_HAL::HAL should be called HAL_Linux and exist
 * in the global namespace. There should be a single const instance of the
 * HAL_Linux class called AP_HAL_Linux, instantiated in the HAL_Linux_Class.cpp
 * and exported as `extern const HAL_Linux AP_HAL_Linux;` in HAL_Linux_Class.h
 *
 * All declaration and compilation should be guarded by CONFIG_HAL_BOARD macros.
 * In this case, we're using CONFIG_HAL_BOARD == HAL_BOARD_LINUX.
 * When creating a new HAL, declare a new HAL_BOARD_ in AP_HAL/AP_HAL_Boards.h
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "HAL_Linux_Class.h"

#endif // CONFIG_HAL_BOARD
#endif //__AP_HAL_LINUX_H__

