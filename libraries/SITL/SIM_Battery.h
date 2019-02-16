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
  simple battery simulation class
*/

#pragma once

#include "stdint.h"
#include <AP_Param/AP_Param.h>
#include "SITL_Input.h"

namespace SITL {

class Battery {
public:
    Battery() {
        AP_Param::setup_object_defaults(this, var_info);
    };

    static const struct AP_Param::GroupInfo var_info[];
    float batt_voltage() const { return static_cast<float>(base_voltage); };

private:
    AP_Float base_voltage; // battery base voltage
};

}
