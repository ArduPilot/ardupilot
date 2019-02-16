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
  simple sprayer battery class
*/

#include "SIM_Battery.h"
#include "AP_HAL/AP_HAL.h"
#include "AP_Math/AP_Math.h"

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo Battery::var_info[] = {
    AP_GROUPINFO("BATT_VOLTAGE", 0, Battery, base_voltage, 12.6f),
    AP_GROUPEND
};
