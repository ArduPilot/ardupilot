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

#include "NeoPixel.h"

#if AP_NOTIFY_NEOPIXEL_ENABLED

#include "AP_Notify/AP_Notify.h"
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
    SerialLED(NEOPIXEL_LED_OFF, NEOPIXEL_LED_HIGH, NEOPIXEL_LED_MEDIUM, NEOPIXEL_LED_LOW)
{
}

uint16_t NeoPixel::init_ports()
{
    uint16_t mask = 0;
    for (uint16_t i=0; i<AP_NOTIFY_NEOPIXEL_MAX_INSTANCES; i++) {
        const SRV_Channel::Aux_servo_function_t fn = (SRV_Channel::Aux_servo_function_t)((uint8_t)SRV_Channel::k_LED_neopixel1 + i);
        if (!SRV_Channels::function_assigned(fn)) {
            continue;
        }
        mask |= SRV_Channels::get_output_channel_mask(fn);
    }

    if (mask == 0) {
        return 0;
    }

    AP_SerialLED *led = AP_SerialLED::get_singleton();
    if (led == nullptr) {
        return 0;
    }

    for (uint16_t chan=0; chan<16; chan++) {
        if ((1U<<chan) & mask) {
            led->set_num_neopixel(chan+1, (pNotify->get_led_len()));
        }
    }

    return mask;
}

#endif  // AP_NOTIFY_NEOPIXEL_ENABLED
