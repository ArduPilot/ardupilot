/*
 * This program is free software: you can redistribute it and/or modify
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

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_BoardConfig.h"

#if HAL_WITH_UAVCAN
#include "AP_BoardConfig_CAN.h"
#include <AP_UAVCAN/AP_UAVCAN.h>

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_BoardConfig_CAN::CAN_if_var_info::var_info[] = {
    // @Param: DRIVER
    // @DisplayName: Index of virtual driver to be used with physical CAN interface
    // @Description: Enabling this option enables use of CAN buses.
    // @Values: 0:Disabled,1:First driver,2:Second driver
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("DRIVER", 1, AP_BoardConfig_CAN::CAN_if_var_info, _driver_number, HAL_CAN_DRIVER_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: BITRATE
    // @DisplayName: Bitrate of CAN interface
    // @Description: Bit rate can be set up to from 10000 to 1000000
    // @Range: 10000 1000000
    // @User: Advanced
    AP_GROUPINFO("BITRATE", 2, AP_BoardConfig_CAN::CAN_if_var_info, _can_bitrate, 1000000),

    // @Param: DEBUG
    // @DisplayName: Level of debug for CAN devices
    // @Description: Enabling this option will provide debug messages
    // @Values: 0:Disabled,1:Major messages,2:All messages
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 3, AP_BoardConfig_CAN::CAN_if_var_info, _can_debug, 2),

    AP_GROUPEND
};

#endif
