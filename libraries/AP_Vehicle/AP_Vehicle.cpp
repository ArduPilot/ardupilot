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

#include "AP_Vehicle.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;


AP_Vehicle::AP_Vehicle()
{
    if (_singleton) {
        AP_HAL::panic("Too many AP_Vehicle instances");
    }
    _singleton = this;

    //AP_Param::setup_object_defaults(this, var_info);
}

AP_Vehicle *AP_Vehicle::_singleton = nullptr;

/*
 * Get the AP_Vehicle singleton
 */
AP_Vehicle *AP_Vehicle::get_singleton()
{
    return AP_Vehicle::_singleton;
}

namespace AP {

AP_Vehicle &Vehicle()
{
    return *AP_Vehicle::get_singleton();
}

};
