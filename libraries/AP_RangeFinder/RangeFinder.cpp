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

/*
 *       AP_RangeFinder.cpp - Arduino Library for Sharpe GP2Y0A02YK0F
 *       infrared proximity sensor
 *       Code by Jose Julio and Randy Mackay. DIYDrones.com
 *
 *       This has the basic functions that all RangeFinders need implemented
 */
#include "RangeFinder.h"

// Public Methods //////////////////////////////////////////////////////////////

// Read Sensor data - post of the ahrd work is done by the child class's convert_raw_to_distance
int16_t RangeFinder::read()
{
    int16_t temp_dist;

    // convert analog value to distance in cm (using child implementation most likely)
    temp_dist = convert_raw_to_distance(_analog_source->read_average());

    // ensure distance is within min and max
    temp_dist = constrain_float(temp_dist, min_distance, max_distance);

    distance = _mode_filter->apply(temp_dist);
    return distance;
}

