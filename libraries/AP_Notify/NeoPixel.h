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

#define NEOPIXEL_WHITE_STROBE 0

class NeoPixel: public RGBLed {
public:
    NeoPixel();

    typedef union {
        struct PACKED {
            // **NOTE** These are GRB, not RGB
            uint8_t b;
            uint8_t r;
            uint8_t g;
            uint8_t unused;
        };
        uint32_t rgb;
    } RGB;

    static void write_LED(NeoPixel::RGB value);
    static void write_LED(uint16_t instance, NeoPixel::RGB value);
    static void write_LED(uint16_t instance, uint8_t red, uint8_t green, uint8_t blue);
    static uint16_t init_ports();

protected:
    bool hw_init(void) override;
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:

    uint16_t enable_mask;

    // perdiodic tick to re-init
    uint32_t    _last_init_ms;

    // periodic callback
    void timer();
    
    HAL_Semaphore_Recursive _sem;


#if NEOPIXEL_WHITE_STROBE
    // remember last RGB so we can resume after a white pulse
    RGB         _last_rgb;
    uint32_t    _white_long_ms;
    uint32_t    _white_short_ms;
#endif



};
