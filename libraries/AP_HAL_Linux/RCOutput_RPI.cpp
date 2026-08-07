/*
 * Code by Andy Piper <github@andypiper.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * RCOutput driver for Raspberry Pi boards.
 */
#include "RCOutput_RPI.h"

#include <AP_HAL/AP_HAL.h>

namespace Linux {

void RCOutput_RPI::init()
{
#if HAL_LINUX_SERIALLED_ENABLED
    // Initialize serial LED driver using SPI0
    _serial_led.init("/dev/spidev0.0", 1);
#endif
}

bool RCOutput_RPI::set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode, uint32_t clock_mask)
{
#if HAL_LINUX_SERIALLED_ENABLED
    if (!_serial_led.initialized()) {
        return false;
    }
    return _serial_led.set_num_leds(chan, num_leds, mode);
#else
    return false;
#endif
}

bool RCOutput_RPI::set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
#if HAL_LINUX_SERIALLED_ENABLED
    if (!_serial_led.initialized()) {
        return false;
    }
    return _serial_led.set_rgb_data(chan, led, red, green, blue);
#else
    return false;
#endif
}

bool RCOutput_RPI::serial_led_send(const uint16_t chan)
{
#if HAL_LINUX_SERIALLED_ENABLED
    if (!_serial_led.initialized()) {
        return false;
    }
    return _serial_led.send(chan);
#else
    return false;
#endif
}

}  // namespace Linux
