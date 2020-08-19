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

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>

#ifdef WITH_SITL_RGBLED

#include "RGBLed.h"

#ifdef HAVE_SFML_GRAPHICS_H
#include <SFML/Graphics.h>
#else
#include <SFML/Graphics.hpp>
#endif

class SITL_SFML_LED: public RGBLed
{
public:
    SITL_SFML_LED();

protected:
    bool hw_init(void) override;
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:

    pthread_t thread;
    void update_thread(void);
    void update_serial_LEDs(void);
    static void *update_thread_start(void *obj);

    static constexpr uint8_t height = 50;
    static constexpr uint8_t width = height;

    static constexpr uint8_t serialLED_size = 16;

    enum class brightness {
        LED_LOW    = 0x33,
        LED_MEDIUM = 0x7F,
        LED_HIGH   = 0xFF,
        LED_OFF    = 0x00
    };

    uint32_t last_colour;

    uint8_t red;
    uint8_t green;
    uint8_t blue;

    static const uint8_t MAX_LEDS = 64;

    // LED layout control
    bool layout_size(SITL::LedLayout layout, uint16_t &xsize, uint16_t &ysize);
    bool layout_pos(SITL::LedLayout layout, uint8_t chan, uint8_t led, uint16_t &xpos, uint16_t &ypos);
};

#endif
