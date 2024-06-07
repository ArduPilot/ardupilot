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
  AP_SerialLED for controlling serial connected LEDs using WS2812B protocol
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/RCOutput.h>

#include "AP_SerialLED_config.h"

#if AP_SERIALLED_ENABLED

#include <stdint.h>

// limit number of LEDs, mostly to keep DMA memory consumption within
// reasonable bounds
#define AP_SERIALLED_MAX_LEDS 128

class AP_SerialLED {
public:
    // set number of LEDs per pin
    bool set_num_neopixel(uint8_t chan, uint8_t num_leds);
    bool set_num_profiled(uint8_t chan, uint8_t num_leds);
    // set number of LEDs per pin in RGB mode
    bool set_num_neopixel_rgb(uint8_t chan, uint8_t num_leds);

    // set RGB value on mask of LEDs. chan is PWM output, 1..16
    void set_RGB_mask(uint8_t chan, uint32_t ledmask, uint8_t red, uint8_t green, uint8_t blue);

    // set RGB value on LED. LED -1 is all LEDs. LED 0 is first LED. chan is PWM output, 1..16
    bool set_RGB(uint8_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue);

    // trigger sending of LED changes to LEDs
    bool send(uint8_t chan);

    // singleton support
    static AP_SerialLED *get_singleton(void) {
        return &singleton;
    }

private:
    static AP_SerialLED singleton;
};

#endif  // AP_SERIALLED_ENABLED
