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
  backend driver for airspeed from I2C
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_I2C : public AP_Airspeed_Backend
{
public:
    // probe and initialise the sensor
    bool init();

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure);

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature);

private:
    void _measure();
    void _collect();
    void _timer();
    float _temperature;
    float _pressure;
    uint32_t _last_sample_time_ms;
    uint32_t _measurement_started_ms;
};
