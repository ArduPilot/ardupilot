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
#include "NeoPixel.h"
#include "SRV_Channel/SRV_Channel.h"

// This limit is from the dshot driver rcout groups limit
#define AP_NOTIFY_NEOPIXEL_MAX_INSTANCES        4

// Datasheet: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
// 24bit msg as 3 byte GRB (not RGB) where first bit is G7, and last bit is B0
// (first) G7|G6|G5|G4|G3|G2|G1|G0|R7|R6|R5|R4|R3|R2|R1|R0|B7|B6|B5|B4|B3|B2|B1|B0 (last)

#define NEOPIXEL_LED_LOW    0x33
#define NEOPIXEL_LED_MEDIUM 0x7F
#define NEOPIXEL_LED_HIGH   0xFF
#define NEOPIXEL_LED_OFF    0x00

extern const AP_HAL::HAL& hal;

NeoPixel::NeoPixel() :
    RGBLed(NEOPIXEL_LED_OFF, NEOPIXEL_LED_HIGH, NEOPIXEL_LED_MEDIUM, NEOPIXEL_LED_LOW)
{
}

bool NeoPixel::hw_init()
{
    NeoPixel::init_ports();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&NeoPixel::timer, void));
    return true;
}

uint16_t NeoPixel::init_ports()
{
    static uint16_t last_mask = 0;
    uint16_t mask = 0;
    for (uint16_t i=0; i<AP_NOTIFY_NEOPIXEL_MAX_INSTANCES; i++) {
        const SRV_Channel::Aux_servo_function_t chan = (SRV_Channel::Aux_servo_function_t)((uint8_t)SRV_Channel::k_LED_neopixel1 + i);
        if (!SRV_Channels::function_assigned(chan)) {
            continue;
        }
        mask |= SRV_Channels::get_output_channel_mask(chan);
    }

    if (mask != 0 && mask != last_mask) {
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_NEOPIXEL);
    }
    last_mask = mask;
    return mask;
}

void NeoPixel::timer()
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


#if NEOPIXEL_WHITE_STROBE
    if (now_ms - _white_long_ms >= 2000) {
        //start white light
        _white_long_ms = now_ms;
        _white_short_ms = now_ms; // start 100ms WHITE pulse
        hw_set_rgb(0xFF,0xFF,0xFF);
    } else if (_white_short_ms > 0 && now_ms - _white_short_ms >= 100) {
        // stop white light
        _white_short_ms = 0;
        hw_set_rgb(_last_rgb.r, _last_rgb.g, _last_rgb.b);
    }
#endif
}

bool NeoPixel::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    // always rememebr this even when disabled so when we enable it will show correct color
    NeoPixel::RGB value {};
    value.r = red;
    value.g = green;
    value.b = blue;
    
#if NEOPIXEL_WHITE_STROBE
    if (_white_short_ms == 0) {
        // if not during a WHITE BLINK then record last LED
        _last_rgb.rgb = value.rgb;
    }
#endif

    if (enable_mask == 0) {
        // nothing is enabled, no pins set as LED output
        return true;
    }

    NeoPixel::write_LED(value);
    return true;
}

void NeoPixel::write_LED(NeoPixel::RGB value)
{
    for (uint16_t i=0; i<AP_NOTIFY_NEOPIXEL_MAX_INSTANCES; i++) {
        NeoPixel::write_LED(i, value);
    }
}

void NeoPixel::write_LED(uint16_t instance, NeoPixel::RGB value)
{
    if (instance >= AP_NOTIFY_NEOPIXEL_MAX_INSTANCES) {
        return;
    }
    hal.rcout->set_neopixel_rgb_data(instance, value.rgb);
}

void NeoPixel::write_LED(uint16_t instance, uint8_t red, uint8_t green, uint8_t blue)
{
    NeoPixel::RGB value {};
    value.r = red;
    value.g = green;
    value.b = blue;

    NeoPixel::write_LED(instance, value);
}
