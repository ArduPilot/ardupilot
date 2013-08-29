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

//
//  Hardware in the loop gps class.
//	Code by James Goppert
//
//	GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.1"
//

#include <AP_HAL.h>
#include "AP_GPS_HIL.h"

// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_HIL::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_port = s;
}

bool AP_GPS_HIL::read(void)
{
    bool result = _updated;

    // return true once for each update pushed in
    _updated = false;
    return result;
}

void AP_GPS_HIL::setHIL(uint32_t _time, float _latitude, float _longitude, float _altitude,
                        float _ground_speed, float _ground_course, float _speed_3d, uint8_t _num_sats)
{
    time                = _time;
    latitude            = _latitude*1.0e7f;
    longitude           = _longitude*1.0e7f;
    altitude_cm         = _altitude*1.0e2f;
    ground_speed_cm     = _ground_speed*1.0e2f;
    ground_course_cd    = _ground_course*1.0e2f;
    speed_3d_cm         = _speed_3d*1.0e2f;
    num_sats            = _num_sats;
    fix                 = FIX_3D;
    new_data            = true;
    _updated            = true;
}

