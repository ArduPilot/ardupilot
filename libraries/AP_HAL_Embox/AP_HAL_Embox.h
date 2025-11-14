#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

/**
 * Umbrella header for AP_HAL_Embox module.
 * The module header exports singleton instances which must conform the
 * AP_HAL::HAL interface. It may only expose implementation details (class
 * names, headers) via the Embox namespace.
 * The class implementing AP_HAL::HAL should be called HAL_Embox and exist
 * in the global namespace. There should be a single const instance of the
 * HAL_Embox class called AP_HAL_Embox, instantiated in the HAL_Embox_Class.cpp
 * and exported as `extern const HAL_Embox AP_HAL_Embox;` in HAL_Embox_Class.h
 *
 * All declaration and compilation should be guarded by CONFIG_HAL_BOARD macros.
 * In this case, we're using CONFIG_HAL_BOARD == HAL_BOARD_EMPTY.
 * When creating a new HAL, declare a new HAL_BOARD_ in AP_HAL/AP_HAL_Boards.h
 */

#include "HAL_Embox_Class.h"
