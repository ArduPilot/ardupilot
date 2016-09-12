/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2016  Andrew Tridgell. All rights reserved.
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

/*
  RCOutput on the Disco combines I2C motor output (for channel 3) with
  PWM output for the other channels. This class is a wrapper around
  the two other classes
 */
#include "RCOutput_Disco.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

namespace Linux {

RCOutput_Disco::RCOutput_Disco(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : bebop_out(std::move(dev))
{
}
    
void RCOutput_Disco::init()
{
    sysfs_out.init();
    bebop_out.init();
    printf("RCOutput_Disco: initialised\n");
}

void RCOutput_Disco::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(output_table); i++) {
        if (chmask & (1U << i)) {
            output_table[i].output.set_freq(1U<<output_table[i].channel, freq_hz);
        }
    }
}

uint16_t RCOutput_Disco::get_freq(uint8_t ch)
{
    if (ch >= ARRAY_SIZE(output_table)) {
        return 0;
    }
    return output_table[ch].output.get_freq(output_table[ch].channel);
}

void RCOutput_Disco::enable_ch(uint8_t ch)
{
    if (ch >= ARRAY_SIZE(output_table)) {
        return;
    }
    output_table[ch].output.enable_ch(output_table[ch].channel);
}

void RCOutput_Disco::disable_ch(uint8_t ch)
{
    if (ch >= ARRAY_SIZE(output_table)) {
        return;
    }
    output_table[ch].output.disable_ch(output_table[ch].channel);
}

void RCOutput_Disco::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= ARRAY_SIZE(output_table)) {
        return;
    }
    output_table[ch].output.write(output_table[ch].channel, period_us);
}

uint16_t RCOutput_Disco::read(uint8_t ch)
{
    if (ch >= ARRAY_SIZE(output_table)) {
        return 0;
    }
    return output_table[ch].output.read(output_table[ch].channel);
}

void RCOutput_Disco::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

void RCOutput_Disco::cork(void)
{
    sysfs_out.cork();
    bebop_out.cork();
}

void RCOutput_Disco::push(void)
{
    sysfs_out.push();
    bebop_out.push();
}

void RCOutput_Disco::set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm)
{
    sysfs_out.set_esc_scaling(min_pwm, max_pwm);
    bebop_out.set_esc_scaling(min_pwm, max_pwm);
}
    
}
