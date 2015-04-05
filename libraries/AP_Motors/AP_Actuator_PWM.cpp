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

#include <AP_HAL.h>
#include "AP_Actuator_PWM.h"

extern const AP_HAL::HAL &hal;

void AP_Actuator_PWM::enable_ch() {
    hal.rcout->enable_ch(ch);
}

void AP_Actuator_PWM::disable_ch() {
    hal.rcout->disable_ch(ch);
}

void AP_Actuator_PWM::write(int16_t value) {
    hal.rcout->write(ch, value);
}

void AP_Actuator_PWM::set_freq(uint16_t freq_hz) {
    hal.rcout->set_freq((uint32_t) 1 << ch, freq_hz);
}
