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

#include "NeoPixel_Oreo.h"
#include "NeoPixel.h"

extern const AP_HAL::HAL& hal;

NeoPixel_Oreo::NeoPixel_Oreo(uint8_t theme) :
    NotifyDevice()
{
    oreo = new OreoLED_I2C(0,theme);
}

//
// Initialize the LEDs
//
bool NeoPixel_Oreo::init()
{
    enable_mask = NeoPixel::init_ports();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&NeoPixel_Oreo::timer, void));
    return true;
}

void NeoPixel_Oreo::update()
{
    oreo->update();
}

void NeoPixel_Oreo::timer()
{
    WITH_SEMAPHORE(_sem);

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_init_ms >= 1000) {
        _last_init_ms = now_ms;
        enable_mask = NeoPixel::init_ports();
    }
    if (enable_mask == 0) {
        // nothing is enabled, no pins set as LED output
        return;
    }

    // for each LED
    for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
        // check for state change
        switch (oreo->_state_desired[i].mode) {
        case OreoLED_I2C::OREOLED_MODE_MACRO:
            if ((now_ms - oreo->_state_desired[i].last_update_ms) >= (2000U + oreo->_state_desired[i].phase_delay_ms)) {
                oreo->_state_desired[i].phase_delay_ms = 0;
                // TODO: replace with something better than fixed LED pattern
                NeoPixel::RGB led {};
                if (oreo->_state_desired[i].period_toggle) {
                    led.r = 0xFF;
                    led.g = 0xFF;
                    led.b = 0;
                } else {
                    led.r = 0;
                    led.g = 0xFF;
                    led.b = 0xFF;
                }
                NeoPixel::write_LED(i,led);
                oreo->_state_desired[i].period_toggle = !oreo->_state_desired[i].period_toggle;
            }
            break;

        default:
        case OreoLED_I2C::OREOLED_MODE_NONE:
        case OreoLED_I2C::OREOLED_MODE_RGB:
            {
            NeoPixel::RGB led {};
            led.r = oreo->_state_desired[i].red;
            led.g = oreo->_state_desired[i].green;
            led.b = oreo->_state_desired[i].blue;
            NeoPixel::write_LED(i, led);
            }
            break;

        case OreoLED_I2C::OREOLED_MODE_RGB_EXTENDED:
            // last_update can be larger than now_ms if there are phase offsets applied
            if ((now_ms - oreo->_state_desired[i].last_update_ms) > (oreo->_state_desired[i].period + oreo->_state_desired[i].phase_delay_ms)) {
                oreo->_state_desired[i].phase_delay_ms = 0;
                oreo->_state_desired[i].last_update_ms = now_ms;
                NeoPixel::RGB led {};
                if (oreo->_state_desired[i].period_toggle) {
                    led.r = oreo->_state_desired[i].red;
                    led.g = oreo->_state_desired[i].green;
                    led.b = oreo->_state_desired[i].blue;
                } else {
                    led.r = oreo->_state_desired[i].amplitude_red;
                    led.g = oreo->_state_desired[i].amplitude_green;
                    led.b = oreo->_state_desired[i].amplitude_blue;
                }
                NeoPixel::write_LED(i, led);
                oreo->_state_desired[i].period_toggle = !oreo->_state_desired[i].period_toggle;
            }
            break;
        };
        // save state change
        oreo->_state_sent[i] = oreo->_state_desired[i];
    }
}

