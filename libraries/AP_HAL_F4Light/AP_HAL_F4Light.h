
#pragma once


/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

#ifndef IN_CCM
#define IN_CCM  __attribute__((section(".ccm")))
#endif


/**
 * Umbrella header for AP_HAL_F4Light module.
 * The module header exports singleton instances which must conform the
 * AP_HAL::HAL interface. It may only expose implementation details (class
 * names, headers) via the F4Light namespace.
 * The class implementing AP_HAL::HAL should be called HAL_F4Light and exist
 * in the global namespace. There should be a single const instance of the
 * HAL_F4Light class called AP_HAL_F4Light, instantiated in the HAL_F4Light_Class.cpp
 * and exported as `extern const HAL_F4Light AP_HAL_F4Light;` in HAL_F4Light_Class.h
 *
 * All declaration and compilation should be guarded by CONFIG_HAL_BOARD macros.
 * In this case, we're using CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT.
 * When creating a new HAL, declare a new HAL_BOARD_ in AP_HAL/AP_HAL_Boards.h
 *
 * The module should also export an appropriate AP_HAL_MAIN() macro iff the
 * appropriate CONFIG_HAL_BOARD value is set.
 * The AP_HAL_MAIN macro expands to a main function (either an `int main (void)`
 * or `int main (int argc, const char * argv[]), depending on platform) of an
 * ArduPilot application, whose entry points are the c++ functions
 * `void setup()` and `void loop()`, ala Arduino.
 */

#include "HAL_F4Light_Class.h"


