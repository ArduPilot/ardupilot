/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Flymaple port by Mike McCauley
 */

#ifndef __AP_HAL_FLYMAPLE_H__
#define __AP_HAL_FLYMAPLE_H__

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL.h>

/**
 * Umbrella header for AP_HAL_FLYMAPLE module.
 * The module header exports singleton instances which must conform the
 * AP_HAL::HAL interface. It may only expose implementation details (class
 * names, headers) via the FLYMAPLE namespace.
 * The class implementing AP_HAL::HAL should be called HAL_FLYMAPLE and exist
 * in the global namespace. There should be a single const instance of the
 * HAL_FLYMAPLE class called AP_HAL_FLYMAPLE, instantiated in the HAL_FLYMAPLE_Class.cpp
 * and exported as `extern const HAL_FLYMAPLE AP_HAL_FLYMAPLE;` in HAL_FLYMAPLE_Class.h
 *
 * All declaration and compilation should be guarded by CONFIG_HAL_BOARD macros.
 * In this case, we're using CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE.
 * When creating a new HAL, declare a new HAL_BOARD_ in AP_HAL/AP_HAL_Boards.h
 *
 * The module should also export an appropriate AP_HAL_MAIN() macro iff the
 * appropriate CONFIG_HAL_BOARD value is set.
 * The AP_HAL_MAIN macro expands to a main function (either an `int main (void)`
 * or `int main (int argc, const char * argv[]), depending on platform) of an
 * ArduPilot application, whose entry points are the c++ functions
 * `void setup()` and `void loop()`, ala Arduino.
 */

#include "HAL_FLYMAPLE_Class.h"
#include "AP_HAL_FLYMAPLE_Main.h"

#endif //__AP_HAL_FLYMAPLE_H__

