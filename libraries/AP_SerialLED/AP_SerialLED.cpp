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

#if AP_SERIALLED_ENABLED

#include <AP_Math/AP_Math.h>
#include "SRV_Channel/SRV_Channel.h"

extern const AP_HAL::HAL& hal;

AP_SerialLED AP_SerialLED::singleton;

// set number of NeoPixels per pin
bool AP_SerialLED::set_num_neopixel(uint8_t chan, uint8_t num_leds)
{
    if (chan >= 1 && chan <= 16 && num_leds <= AP_SERIALLED_MAX_LEDS) {
        return hal.rcout->set_serial_led_num_LEDs(chan-1, num_leds, AP_HAL::RCOutput::MODE_NEOPIXEL);
    }
    return false;
}

// set number of NeoPixels per pin in RGB mode
bool AP_SerialLED::set_num_neopixel_rgb(uint8_t chan, uint8_t num_leds)
{
    if (chan >= 1 && chan <= 16 && num_leds <= AP_SERIALLED_MAX_LEDS) {
        return hal.rcout->set_serial_led_num_LEDs(chan-1, num_leds, AP_HAL::RCOutput::MODE_NEOPIXELRGB);
    }
    return false;
}

// set number of ProfiLEDs per pin
bool AP_SerialLED::set_num_profiled(uint8_t chan, uint8_t num_leds)
{
    if (chan >= 1 && chan <= 16 && num_leds <= AP_SERIALLED_MAX_LEDS - 2) {
        // must have a clock
        uint32_t Clock_mask = 0;
        if (!SRV_Channels::function_assigned((SRV_Channel::Function)((uint8_t)SRV_Channel::k_ProfiLED_Clock))) {
            return false;
        }
        Clock_mask = SRV_Channels::get_output_channel_mask((SRV_Channel::Function)((uint8_t)SRV_Channel::k_ProfiLED_Clock));

        return hal.rcout->set_serial_led_num_LEDs(chan-1, num_leds, AP_HAL::RCOutput::MODE_PROFILED, Clock_mask);
    }
    return false;
}

// set RGB value on LED number. LED number -1 is all LEDs. First LED is 0. chan is PWM output, 1..16
bool AP_SerialLED::set_RGB(uint8_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    if (chan >= 1 && chan <= 16) {
        return hal.rcout->set_serial_led_rgb_data(chan-1, led, red, green, blue);
    }
    return false;
}

// trigger sending of LED changes to LEDs
bool AP_SerialLED::send(uint8_t chan)
{
    if (chan >= 1 && chan <= 16) {
        return hal.rcout->serial_led_send(chan-1);
    }
    return false;
}

#endif  // AP_SERIALLED_ENABLED
