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
 *       AP_Compass_HIL.cpp - Arduino Library for HIL model of HMC5843 I2C Magnetometer
 *       Code by James Goppert. DIYDrones.com
 *
 */


#include <AP_HAL.h>
#include "AP_Compass_HIL.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_Compass_HIL::AP_Compass_HIL(Compass &compass):
    AP_Compass_Backend(compass)    
{
    product_id = AP_COMPASS_TYPE_HIL;
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
    _compass_instance = _compass.register_compass();
    return true;
}

// Public Methods //////////////////////////////////////////////////////////////

bool AP_Compass_HIL::read()
{
    _compass._field[_compass_instance] = _compass._hil_mag;    
    _compass.apply_corrections(_compass._field[_compass_instance],_compass_instance);

    // values set by setHIL function
    _compass.last_update = hal.scheduler->micros();      // record time of update
    
    //_update_compass(_compass_instance, _compass._field[_compass_instance], _compass._healthy[_compass_instance])
    return true;
}

void AP_Compass_HIL::accumulate(void)
{
    // nothing to do
}
