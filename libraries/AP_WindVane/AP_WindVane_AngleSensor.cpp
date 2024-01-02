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

#include "AP_WindVane_config.h"

#if AP_WINDVANE_ANGLESENSOR_ENABLED

#include "AP_WindVane_AngleSensor.h"

// constructor
AP_WindVane_AngleSensor::AP_WindVane_AngleSensor(AP_WindVane &frontend) :
    AP_WindVane_Backend(frontend)
{
    _encoder_instance = _frontend._dir_analog_pin;
}

void AP_WindVane_AngleSensor::update_direction()
{
    _encoder_instance = _frontend._dir_analog_pin; // Allow Runtime parameter update
    
    const AP_AngleSensor* angle_sensor_driver = AP_AngleSensor::get_singleton();
    if (angle_sensor_driver != nullptr) {
        if (angle_sensor_driver->healthy(_encoder_instance)) {
            _frontend._direction_apparent_raw = angle_sensor_driver->get_angle_radians(_encoder_instance);
        }
    }
}

#endif  //AP_WINDVANE_ANGLESENSOR_ENABLED