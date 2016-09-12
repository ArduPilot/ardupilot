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
#include "RCOutput_Sysfs.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

namespace Linux {

RCOutput_Sysfs::RCOutput_Sysfs(uint8_t chip, uint8_t channel_base, uint8_t channel_count)
    : _chip(chip)
    , _channel_base(channel_base)
    , _channel_count(channel_count)
    , _pwm_channels(new PWM_Sysfs_Base *[_channel_count])
{
}

RCOutput_Sysfs::~RCOutput_Sysfs()
{
    for (uint8_t i = 0; i < _channel_count; i++) {
        delete _pwm_channels[i];
    }

    delete _pwm_channels;
}

void RCOutput_Sysfs::init()
{
    for (uint8_t i = 0; i < _channel_count; i++) {
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
        _pwm_channels[i] = new PWM_Sysfs_Bebop(_channel_base+i);
#else
        _pwm_channels[i] = new PWM_Sysfs(_chip, _channel_base+i);
#endif
        if (!_pwm_channels[i]) {
            AP_HAL::panic("RCOutput_Sysfs_PWM: Unable to setup PWM pin.");
        }
        _pwm_channels[i]->enable(false);

        /* Set the initial frequency */
        _pwm_channels[i]->set_freq(50);
        _pwm_channels[i]->set_duty_cycle(0);
        _pwm_channels[i]->set_polarity(PWM_Sysfs::Polarity::NORMAL);
    }
}

void RCOutput_Sysfs::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    for (uint8_t i = 0; i < _channel_count; i++) {
        if (chmask & 1 << i) {
            _pwm_channels[i]->set_freq(freq_hz);
        }
    }
}

uint16_t RCOutput_Sysfs::get_freq(uint8_t ch)
{
    if (ch >= _channel_count) {
        return 0;
    }

    return _pwm_channels[ch]->get_freq();
}

void RCOutput_Sysfs::enable_ch(uint8_t ch)
{
    if (ch >= _channel_count) {
        return;
    }

    _pwm_channels[ch]->enable(true);
}

void RCOutput_Sysfs::disable_ch(uint8_t ch)
{
    if (ch >= _channel_count) {
        return;
    }

    _pwm_channels[ch]->enable(false);
}

void RCOutput_Sysfs::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= _channel_count) {
        return;
    }

    _pwm_channels[ch]->set_duty_cycle(usec_to_nsec(period_us));
}

uint16_t RCOutput_Sysfs::read(uint8_t ch)
{
    if (ch >= _channel_count) {
        return 1000;
    }

    return nsec_to_usec(_pwm_channels[ch]->get_duty_cycle());
}

void RCOutput_Sysfs::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < MIN(len, _channel_count); i++) {
        period_us[i] = read(i);
    }
    for (int i = _channel_count; i < len; i++) {
        period_us[i] = 1000;
    }
}
}
