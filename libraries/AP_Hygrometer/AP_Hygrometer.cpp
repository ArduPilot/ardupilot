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

#include "AP_Hygrometer.h"
#include "AP_Hygrometer_UAVCAN.h"

void AP_Hygrometer::init(void)
{
    sensor = AP_Hygrometer_UAVCAN::probe(*this, 1);
}


AP_Hygrometer::AP_Hygrometer()
{

}


bool AP_Hygrometer::get_humidity(float &humidity)
{

    if (sensor) {
        return sensor->get_humidity(humidity);
    }

    return false;
}


bool AP_Hygrometer::get_temperature(float &temperature)
{

    if (sensor) {
        return sensor->get_temperature(temperature);
    }

    return false;
}


bool AP_Hygrometer::get_id(uint8_t &id)
{
    if (sensor) {
        return sensor->get_id(id);
    }

    return false;
}
