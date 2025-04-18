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
  battery info support. This feature has a set of parameters which are
  controlled by a lua script on the FMU. The battery infomation node
  is attached to a specific battery and recorded the number of cycles,
  and total armed time
 */
#include "AP_Periph.h"

#if AP_PERIPH_BATTERY_INFO_ENABLED

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo BattInfo::var_info[] {
    // @Param: _NUM_CYCLES
    // @DisplayName: Number of cycles
    // @Description: Number of cycles the battery has been through
    AP_GROUPINFO("_NUM_CYCLES", 1, BattInfo, num_cycles, 0),

    // @Param: _ARM_HOURS
    // @DisplayName: Number of armed hours
    // @Description: Number of hours the battery has been armed
    AP_GROUPINFO("_ARM_HOURS", 2, BattInfo, num_hours, 0),
    
    AP_GROUPEND
};

BattInfo::BattInfo(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif  // AP_PERIPH_BATTERY_INFO_ENABLED
