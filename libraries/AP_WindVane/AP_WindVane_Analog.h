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

#include "AP_WindVane_Backend.h"

class AP_WindVane_Analog : public AP_WindVane_Backend
{
public:
    // constructor
    AP_WindVane_Analog(AP_WindVane &frontend);

    // update state
    void update_direction() override;
    void calibrate() override;

private:
    // pin for reading analog voltage
    class AP_HAL::AnalogSource *_dir_analog_source;

    float _current_analog_voltage;
    uint32_t  _cal_start_ms = 0;
    float _cal_volt_min;
    float _cal_volt_max;
};
