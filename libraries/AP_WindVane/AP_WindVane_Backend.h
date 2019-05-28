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

#include "AP_WindVane.h"

#include <GCS_MAVLink/GCS.h>
#include <Filter/Filter.h>

class AP_WindVane_Backend
{
public:
    // constructor. This incorporates initialization as well.
    AP_WindVane_Backend(AP_WindVane &frontend);

    // we declare a virtual destructor so that WindVane drivers can
    // override with a custom destructor if need be
    virtual ~AP_WindVane_Backend() {}

    // initialization
    virtual void init(const AP_SerialManager& serial_manager) {};

    // update the state structure
    virtual void update_speed() {};
    virtual void update_direction() {};
    virtual void calibrate();

protected:
    // update frontend
    void speed_update_frontend(float apparent_speed_in);
    void direction_update_frontend(float apparent_angle_ef);

    AP_WindVane &_frontend;

private:
    // low pass filters of direction and speed
    LowPassFilterFloat _dir_sin_filt = LowPassFilterFloat(2.0f);
    LowPassFilterFloat _dir_cos_filt = LowPassFilterFloat(2.0f);
    LowPassFilterFloat _speed_filt = LowPassFilterFloat(2.0f);
};
