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
  backend driver for airspeed from PX4Firmware
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_Airspeed_Backend.h"

class AP_Airspeed_PX4 : public AP_Airspeed_Backend {
public:
    // constructor
    AP_Airspeed_PX4() : _fd(-1) {}

    // probe and initialise the sensor
    bool init(void);

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure);

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature);

private:
    int _fd;
    uint64_t _last_timestamp;
    float _temperature;
};
