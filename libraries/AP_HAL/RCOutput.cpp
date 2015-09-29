// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_HAL/AP_HAL.h>

using namespace AP_HAL;

void RCOutput::init_backend(RCOutput_Backend *backend)
{
    if (!backend->init()) {
        return;
    }

    uint8_t backend_offset = 0;

    for (uint8_t i = 0; i < _backend_count; i++) {
        backend_offset += _backends[i]->get_num_channels();
    }

    for (uint8_t i = 0; i < backend->get_num_channels(); i++) {
        _backend_by_channel[backend_offset + i] = _backend_count;
        _backend_channel_by_channel[backend_offset + i] = i;
    }

    _backends[_backend_count] = backend;

    _backend_count++;
}

void RCOutput::set_freq(uint64_t chmask, uint16_t freq_hz) {
    for (uint8_t i = 0; i < _backend_count; i++) {
        if (_backends[i] == NULL) {
            continue;
        }

        _backends[i]->set_freq(chmask & ((1U << _backends[i]->get_num_channels()) - 1), freq_hz);
        chmask >>=_backends[i]->get_num_channels();
    }
}

uint16_t RCOutput::get_freq(uint8_t ch) {
    return _backends[_backend_by_channel[ch]]->get_freq(_backend_channel_by_channel[ch]);
}

void RCOutput::enable_ch(uint8_t ch) { 
    _backends[_backend_by_channel[ch]]->enable_ch(_backend_channel_by_channel[ch]);
}

void RCOutput::disable_ch(uint8_t ch) {
    _backends[_backend_by_channel[ch]]->disable_ch(_backend_channel_by_channel[ch]);
}

void RCOutput::write(uint8_t ch, uint16_t period_us) {
    _backends[_backend_by_channel[ch]]->write(_backend_channel_by_channel[ch], period_us);
}

uint16_t RCOutput::read(uint8_t ch) {
    return _backends[_backend_by_channel[ch]]->read(_backend_channel_by_channel[ch]);
}

void RCOutput::set_safety_pwm(uint64_t chmask, uint16_t period_us) {
    for (uint8_t i = 0; i < _backend_count; i++) {
        if (_backends[i] == NULL) {
            continue;
        }

        _backends[i]->set_safety_pwm(chmask & ((1U << _backends[i]->get_num_channels()) - 1), period_us);
        chmask >>= _backends[i]->get_num_channels();
    }
}

void RCOutput::set_failsafe_pwm(uint64_t chmask, uint16_t period_us) {
    for (uint8_t i = 0; i < _backend_count; i++) {
        if (_backends[i] == NULL) {
            continue;
        }

        _backends[i]->set_failsafe_pwm(chmask & ((1U << _backends[i]->get_num_channels()) - 1), period_us);
        chmask >>= _backends[i]->get_num_channels();
    }
}

bool RCOutput::force_safety_on() {
    for (uint8_t i = 0; i < _backend_count; i++) {
        if (_backends[i] == NULL) {
            continue;
        }
           
        if (!_backends[i]->force_safety_on()) {
            // If forcing safety on one backend fails, revert by forcing 
            // safety off on all backends.

            force_safety_off();
            return false;
        }
    }

    return true;
}

void RCOutput::force_safety_off() {
    for (uint8_t i = 0; i < _backend_count; i++) {
        if (_backends[i] == NULL) {
            continue;
        }
           
        _backends[i]->force_safety_off();
    }
}

void RCOutput::set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) {
    for (uint8_t i = 0; i < _backend_count; i++) {
        if (_backends[i] == NULL) {
            continue;
        }
           
        _backends[i]->set_esc_scaling(min_pwm, max_pwm);
    }
}