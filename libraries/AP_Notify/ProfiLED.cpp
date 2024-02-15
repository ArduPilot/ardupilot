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

#include "ProfiLED.h"

#include "AP_Notify/AP_Notify.h"

#include "SRV_Channel/SRV_Channel.h"
#include <utility>
#include <GCS_MAVLink/GCS.h>

// This limit is from the dshot driver rc out groups limit, we need at least one channel for clock
#define AP_NOTIFY_ProfiLED_MAX_INSTANCES        3

#define ProfiLED_LOW    0x33
#define ProfiLED_MEDIUM 0x7F
#define ProfiLED_HIGH   0xFF
#define ProfiLED_OFF    0x00

extern const AP_HAL::HAL& hal;

#if AP_NOTIFY_PROFILED_ENABLED

ProfiLED::ProfiLED() :
    SerialLED(ProfiLED_OFF, ProfiLED_HIGH, ProfiLED_MEDIUM, ProfiLED_LOW)
{
}

uint16_t ProfiLED::init_ports()
{
    uint16_t mask = 0;
    for (uint16_t i=0; i<AP_NOTIFY_ProfiLED_MAX_INSTANCES; i++) {
        const SRV_Channel::Aux_servo_function_t fn = (SRV_Channel::Aux_servo_function_t)((uint8_t)SRV_Channel::k_ProfiLED_1 + i);
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
            led->set_num_profiled(chan+1, (pNotify->get_led_len()));
        }
    }

    return mask;
}
#endif  // AP_NOTIFY_PROFILED_ENABLED

#if AP_NOTIFY_PROFILED_SPI_ENABLED
ProfiLED_SPI::ProfiLED_SPI() :
    RGBLed(ProfiLED_OFF, ProfiLED_HIGH, ProfiLED_MEDIUM, ProfiLED_LOW) {}

bool ProfiLED_SPI::init()
{
    num_leds = pNotify->get_led_len() + 1; // for some reason we have to send an additional LED data
    rgb = new ProfiLED_SPI::RGB[num_leds];
    if (!rgb) {
        return false;
    }
    memset(rgb, 0, num_leds * sizeof(ProfiLED_SPI::RGB));
    _dev = std::move(hal.spi->get_device("profiled"));
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&ProfiLED_SPI::_timer, void));
    _need_update = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Initialised ProfiLED over SPIs");
    return true;
}

void ProfiLED_SPI::rgb_set_id(uint8_t red, uint8_t green, uint8_t blue, uint8_t id)
{
    if (id >= num_leds) {
        return;
    }
    rgb[id] = {blue, red, green};
    _need_update = true;
}

bool ProfiLED_SPI::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    for (uint16_t i = 0; i < num_leds; i++) {
        rgb[i] = {blue, red, green};
    }
    _need_update = true;
    return true;
}

void ProfiLED_SPI::_timer(void)
{
    _need_update = false;
    update_led_strip();
}

void ProfiLED_SPI::update_led_strip() {
    const uint32_t min_bits = num_leds*25+50;
    const uint8_t num_leading_zeros = 8-min_bits%8 + 50;
    const uint32_t output_stream_byte_length = (min_bits+7)/8;

    WITH_SEMAPHORE(_dev->get_semaphore());

    uint32_t curr_led_idx = 0;

    uint8_t send_buf[32];
    uint8_t send_buf_len = 0;

    union {
        ProfiLED_SPI::RGB  struct_val;
        uint8_t bytes_val[3];
    } curr_led_color;
    
    curr_led_color.struct_val = rgb[curr_led_idx];
    for (uint32_t i=0; i<output_stream_byte_length; i++) {
        uint8_t byte = 0;
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint32_t out_bit_idx = i*8+bit;
            uint8_t bit_val;
            if (out_bit_idx < num_leading_zeros) {
                bit_val = 0;
            } else if ((out_bit_idx-num_leading_zeros) % 25 == 0) {
                bit_val = 1;
            } else {
                uint32_t in_bit_idx = out_bit_idx - num_leading_zeros - (out_bit_idx - num_leading_zeros)/25;
                uint32_t in_led_idx = in_bit_idx/24;

                if (curr_led_idx != in_led_idx) {
                    curr_led_idx = in_led_idx;
                    curr_led_color.struct_val = rgb[curr_led_idx];
                }

                bit_val = (curr_led_color.bytes_val[(in_bit_idx%24)/8] >> (8-in_bit_idx%8)) & 1;
            }

            byte |= bit_val << (7-bit);
        }
        send_buf[send_buf_len++] = byte;
        if (send_buf_len >= 32) {
            _dev->transfer(send_buf, send_buf_len, NULL, 0);
            send_buf_len = 0;
        }
    }
    if (send_buf_len > 0) {
        _dev->transfer(send_buf, send_buf_len, NULL, 0);
        send_buf_len = 0;
    }
}

#endif  // AP_NOTIFY_PROFILED_SPI_ENABLED
