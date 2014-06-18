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
 *       AP_RangeFinder_SharpGP2Y.cpp - Arduino Library for Sharpe GP2Y0A02YK0F
 *       infrared proximity sensor
 *       Code by Jose Julio and Randy Mackay. DIYDrones.com
 *
 *       Sensor should be conected to one of the analog ports
 *
 *       Sparkfun URL: http://www.sparkfun.com/products/8958
 *       datasheet: http://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf
 *
 *       Variables:
 *               int raw_value : raw value from the sensor
 *               int distance : distance in cm
 *               int max_distance : maximum measurable distance (in cm)
 *               int min_distance : minimum measurable distance (in cm)
 *
 *       Methods:
 *               read() : read value from analog port
 *
 */

#include "AP_RangeFinder_SharpGP2Y.h"

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_SharpGP2Y::AP_RangeFinder_SharpGP2Y(AP_HAL::AnalogSource *source, FilterInt16 *filter) :
    RangeFinder(source, filter)
{
    max_distance = AP_RANGEFINDER_SHARPEGP2Y_MAX_DISTANCE;
    min_distance = AP_RANGEFINDER_SHARPEGP2Y_MIN_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////
