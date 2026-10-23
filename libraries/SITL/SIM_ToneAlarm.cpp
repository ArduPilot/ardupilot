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
  simple tonealarm simulator class
*/

#include <GCS_MAVLink/GCS.h>
#include <SITL/SITL.h>

#include "SIM_ToneAlarm.h"

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo ToneAlarm::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: ToneAlarm enable/disable
    // @Description: Allows you to enable (1) or disable (0) the simulated tonealarm
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, ToneAlarm, _enable, 1),

    AP_GROUPEND
};

/*
  update tonealarm state
 */
void ToneAlarm::update(const struct sitl_input &input)
{
    // currently all of the simulated buzzer logic is within AP_HAL_SITL
}
