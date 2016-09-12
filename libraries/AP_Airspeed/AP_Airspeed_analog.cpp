/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
/*
 *   analog airspeed driver
 */
#include "AP_Airspeed_analog.h"

#include <AP_ADC/AP_ADC.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Airspeed.h"

extern const AP_HAL::HAL &hal;

// scaling for 3DR analog airspeed sensor
#define VOLTS_TO_PASCAL 819

bool AP_Airspeed_Analog::init()
{
    _source = hal.analogin->channel(_pin);
    return true;
}

// read the airspeed sensor
bool AP_Airspeed_Analog::get_differential_pressure(float &pressure)
{
    if (_source == NULL) {
        return false;
    }
    _source->set_pin(_pin);
    pressure = _source->voltage_average_ratiometric() * VOLTS_TO_PASCAL / _psi_range.get();
    return true;
}
