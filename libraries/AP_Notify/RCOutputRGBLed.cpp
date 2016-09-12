/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
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
 */

#include "RCOutputRGBLed.h"

#include <AP_Math/AP_Math.h>

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define LED_OFF 0
#define LED_FULL_BRIGHT 255
#define LED_MEDIUM ((LED_FULL_BRIGHT / 5) * 4)
#define LED_DIM ((LED_FULL_BRIGHT / 5) * 2)

RCOutputRGBLed::RCOutputRGBLed(uint8_t red_channel, uint8_t green_channel, uint8_t blue_channel)
    : RCOutputRGBLed(red_channel, green_channel, blue_channel, LED_OFF,
                     LED_FULL_BRIGHT, LED_MEDIUM, LED_DIM)
{
}

RCOutputRGBLed::RCOutputRGBLed(uint8_t red_channel, uint8_t green_channel,
                               uint8_t blue_channel, uint8_t led_off,
                               uint8_t led_full, uint8_t led_medium,
                               uint8_t led_dim)
    : RGBLed(led_off, led_full, led_medium, led_dim)
    , _red_channel(red_channel)
    , _green_channel(green_channel)
    , _blue_channel(blue_channel)
{
}

bool RCOutputRGBLed::hw_init()
{
    hal.rcout->enable_ch(_red_channel);
    hal.rcout->enable_ch(_green_channel);
    hal.rcout->enable_ch(_blue_channel);

    return true;
}

bool RCOutputRGBLed::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint16_t freq_motor = hal.rcout->get_freq(0);
    const uint16_t freq = hal.rcout->get_freq(_red_channel);
    const uint16_t usec_period = hz_to_usec(freq);

    if (freq_motor != freq) {
        /*
         * keep at same frequency as the first RCOutput channel, some RCOutput
         * drivers can not operate in different frequency between channels
         */
        const uint32_t mask = 1 << _red_channel | 1 << _green_channel
                              | 1 << _blue_channel;
        hal.rcout->set_freq(mask, freq_motor);
    }

    /*
     * Not calling push() to have a better performance on RCOutput's that
     * implements cork()/push(), so this changes will be committed together
     * with the motors.
     */
    hal.rcout->cork();

    uint16_t usec_duty = usec_period * red / _led_bright;
    hal.rcout->write(_red_channel, usec_duty);

    usec_duty = usec_period * green / _led_bright;
    hal.rcout->write(_green_channel, usec_duty);

    usec_duty = usec_period * blue / _led_bright;
    hal.rcout->write(_blue_channel, usec_duty);

    return true;
}
