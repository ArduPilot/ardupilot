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

#if HAL_CANMANAGER_ENABLED
#include "AP_CANManager.h"

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_CANManager::CANIface_Params::var_info[] = {
    // @Param: DRIVER
    // @DisplayName: Index of virtual driver to be used with physical CAN interface
    // @Description: Enabling this option enables use of CAN buses.
    // @Values: 0:Disabled,1:First driver,2:Second driver,3:Third driver
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("DRIVER", 1, AP_CANManager::CANIface_Params, _driver_number, HAL_CAN_DRIVER_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: BITRATE
    // @DisplayName: Bitrate of CAN interface
    // @Description: Bit rate can be set up to from 10000 to 1000000
    // @Range: 10000 1000000
    // @User: Advanced
    AP_GROUPINFO("BITRATE", 2, AP_CANManager::CANIface_Params, _bitrate, 1000000),

#if HAL_CANFD_SUPPORTED
    // @Param: FDBITRATE
    // @DisplayName: Bitrate of CANFD interface
    // @Description: Bit rate can be set up to from 1000000 to 8000000
    // @Values: 1:1M, 2:2M, 4:4M, 5:5M, 8:8M
    // @User: Advanced
    AP_GROUPINFO("FDBITRATE", 3, AP_CANManager::CANIface_Params, _fdbitrate, HAL_CANFD_SUPPORTED),
#endif

    // @Param: OPTIONS
    // @DisplayName: CAN per-interface options
    // @Description: CAN per-interface options
    // @Bitmask: 0:LogAllFrames
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 4, AP_CANManager::CANIface_Params, _options, 0),

    // Index 3 occupied by Param: DEBUG
    AP_GROUPEND
};

#endif
