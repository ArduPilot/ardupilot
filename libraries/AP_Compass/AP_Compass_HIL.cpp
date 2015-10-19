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
 *       AP_Compass_HIL.cpp - HIL backend for AP_Compass
 *
 */


#include <AP_HAL.h>
#include "AP_Compass_HIL.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_Compass_HIL::AP_Compass_HIL(Compass &compass):
    AP_Compass_Backend(compass),
    _compass_instance(0)
{
    _compass._setup_earth_field();
}

// detect the sensor
AP_Compass_Backend *AP_Compass_HIL::detect(Compass &compass)
{
    AP_Compass_HIL *sensor = new AP_Compass_HIL(compass);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool
AP_Compass_HIL::init(void)
{
    // register the compass instance in the frontend
    _compass_instance = register_compass();
    return true;
}

void AP_Compass_HIL::read()
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    if (_count == 0) return;

    _sum /= _count;

    publish_filtered_field(_sum, _compass_instance);

    _sum.zero();
    _count = 0;
}

void AP_Compass_HIL::accumulate(void)
{
  while (_compass._hil.field_buffer.is_empty() == false){
    Vector3f raw_field;
    uint32_t time_us = 0;

    _compass._hil.field_buffer.pop_front(raw_field);
    _compass._hil.field_time_us.pop_front(time_us);

    // rotate raw_field from sensor frame to body frame
    rotate_field(raw_field, _compass_instance);

    // publish raw_field (uncorrected point sample) for calibration use
    publish_raw_field(raw_field, time_us, _compass_instance);

    // correct raw_field for known errors
    correct_field(raw_field, _compass_instance);

    // publish raw_field (corrected point sample) for EKF use
    publish_unfiltered_field(raw_field, time_us, _compass_instance);

    // accumulate into averaging filter
    _sum += raw_field;
    _count++;
  }
}
