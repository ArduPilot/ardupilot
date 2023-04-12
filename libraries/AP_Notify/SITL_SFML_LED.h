/*
  Copyright (C) 2019 Peter Barker. All rights reserved.

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
#pragma once

/*
  cp libraries/AP_Scripting/examples/LED_roll.lua scripts/
  param set SCR_ENABLE 1
  param set SIM_LED_LAYOUT 1
  param set SERVO10_FUNCTION 94  # script1
  param set SERVO11_FUNCTION 132  # profiled clock
  param set NTF_LED_TYPES 1024
  reboot
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include <SITL/SITL.h>

#ifdef WITH_SITL_RGBLED

#ifdef HAVE_SFML_GRAPHICS_H
#include <SFML/Graphics.h>
#else
#include <SFML/Graphics.hpp>
#endif

class SITL_SFML_LED : public NotifyDevice
{
public:
    SITL_SFML_LED() {}
    bool init(void) override;
    void update() override;

private:

    pthread_t thread;
    void update_thread(void);
    void update_serial_LEDs(void);
    static void *update_thread_start(void *obj);

    static constexpr uint8_t serialLED_size = 16;

    static const uint8_t MAX_LEDS = 64;

    // LED layout control
    bool layout_size(SITL::LedLayout layout, uint16_t &xsize, uint16_t &ysize);
    bool layout_pos(SITL::LedLayout layout, uint8_t chan, uint8_t led, uint16_t &xpos, uint16_t &ypos);
};

#endif
