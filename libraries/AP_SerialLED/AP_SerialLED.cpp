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

#include "AP_SerialLED.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

AP_SerialLED AP_SerialLED::singleton;

AP_SerialLED::AP_SerialLED()
{
}

// set number of LEDs per pin
bool AP_SerialLED::set_num_LEDs(uint8_t chan, uint8_t num_leds)
{
    if (chan >= 1 && chan <= 16 && num_leds <= 32) {
        return hal.rcout->set_neopixel_num_LEDs(chan-1, num_leds);
    }
    return false;
}

// set RGB value on mask of LEDs. chan is PWM output, 1..16
void AP_SerialLED::set_RGB(uint8_t chan, uint32_t ledmask, uint8_t red, uint8_t green, uint8_t blue)
{
    if (chan >= 1 && chan <= 16) {
        hal.rcout->set_neopixel_rgb_data(chan-1, ledmask, red, green, blue);
    }
}

// trigger sending of LED changes to LEDs
void AP_SerialLED::send(void)
{
    hal.rcout->neopixel_send();
}
