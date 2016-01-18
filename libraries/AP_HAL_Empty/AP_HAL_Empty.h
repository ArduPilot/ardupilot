
#ifndef __AP_HAL_EMPTY_H__
#define __AP_HAL_EMPTY_H__

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL.h>

/**
 * Umbrella header for AP_HAL_Empty module.
 * The module header exports singleton instances which must conform the
 * AP_HAL::HAL interface. It may only expose implementation details (class
 * names, headers) via the Empty namespace.
 * The class implementing AP_HAL::HAL should be called HAL_Empty and exist
 * in the global namespace. There should be a single const instance of the
 * HAL_Empty class called AP_HAL_Empty, instantiated in the HAL_Empty_Class.cpp
 * and exported as `extern const HAL_Empty AP_HAL_Empty;` in HAL_Empty_Class.h
 *
 * All declaration and compilation should be guarded by CONFIG_HAL_BOARD macros.
 * In this case, we're using CONFIG_HAL_BOARD == HAL_BOARD_EMPTY.
 * When creating a new HAL, declare a new HAL_BOARD_ in AP_HAL/AP_HAL_Boards.h
 *
 * The module should also export an appropriate AP_HAL_MAIN() macro iff the
 * appropriate CONFIG_HAL_BOARD value is set.
 * The AP_HAL_MAIN macro expands to a main function (either an `int main (void)`
 * or `int main (int argc, const char * argv[]), depending on platform) of an
 * ArduPilot application, whose entry points are the c++ functions
 * `void setup()` and `void loop()`, ala Arduino.
 */

#include "HAL_Empty_Class.h"
#include "AP_HAL_Empty_Main.h"

#endif //__AP_HAL_EMPTY_H__

