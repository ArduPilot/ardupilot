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

#include "AP_Notify/AP_Notify.h"
#include "SerialLED.h"

extern const AP_HAL::HAL& hal;

SerialLED::SerialLED(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim) :
    RGBLed(led_off, led_bright, led_medium, led_dim)
{
}

bool SerialLED::hw_init()
{
    init_ports();
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&SerialLED::timer, void));
    return true;
}

void SerialLED::timer()
{
    WITH_SEMAPHORE(_sem);

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_init_ms >= 1000) {
        _last_init_ms = now_ms;
        enable_mask = init_ports();
    }
}

bool SerialLED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (enable_mask == 0) {
        // nothing is enabled, no pins set as LED output
        return true;
    }

    AP_SerialLED *led = AP_SerialLED::get_singleton();
    if (led == nullptr) {
        return false;
    }

    for (uint16_t chan=0; chan<16; chan++) {
        if ((1U<<chan) & enable_mask) {
            led->set_RGB(chan+1, -1, red, green, blue);
        }
    }

    for (uint16_t chan=0; chan<16; chan++) {
        if ((1U<<chan) & enable_mask) {
            led->send(chan+1);
        }
    }

    return true;
}
