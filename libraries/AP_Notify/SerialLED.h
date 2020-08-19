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
#pragma once

#include "RGBLed.h"
#include <AP_Common/AP_Common.h>
#include <AP_SerialLED/AP_SerialLED.h>

class SerialLED: public RGBLed {
public:
    SerialLED(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim);

    struct {
        uint8_t b;
        uint8_t r;
        uint8_t g;
    } RGB;

    virtual uint16_t init_ports() { return 0; };

protected:

    bool hw_init(void) override;

    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    uint16_t enable_mask;

    // perdiodic tick to re-init
    uint32_t    _last_init_ms;

    // periodic callback
    void timer();

    HAL_Semaphore _sem;
};
