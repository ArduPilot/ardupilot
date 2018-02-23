/*
   Joypad Interface Driver for URUS and Ardupilot.
   Copyright (c) 2017-2018 Hiroshi Takey <htakey@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#if ((CONFIG_HAL_BOARD == HAL_BOARD_URUS) && (CONFIG_SHAL_CORE == SHAL_CORE_APM)) || (CONFIG_HAL_BOARD == HAL_BOARD_SITL)

#include "AP_Joypad.h"

class AP_Joypad_Backend {
public:

    AP_Joypad_Backend(AP_Joypad &joypad);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_Joypad_Backend(void) {}

    /** Process the backend.
      * @param  process_mode:
      *         [AutoProcess] - Update process run in the scheduled
      *         callback. This is the default mode.
      *         [LoopProcess] - Update process run not in scheduled
      *         callback. udpate() need to be called in somewhere
      *         to see the action, otherwise nothing happen.
      * @return None.
      */
    virtual void process(AP_Joypad::ProcessMode process_mode);

    /** See update function on top class.
      */
    virtual void update();

protected:
    // access to frontend
    AP_Joypad &_joypad;
};

#endif
