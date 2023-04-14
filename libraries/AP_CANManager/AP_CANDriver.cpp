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

#if HAL_MAX_CAN_PROTOCOL_DRIVERS && HAL_CANMANAGER_ENABLED
#include "AP_CANManager.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_PiccoloCAN/AP_PiccoloCAN.h>
#include <AP_KDECAN/AP_KDECAN.h>


// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_CANManager::CANDriver_Params::var_info[] = {

    // @Param: PROTOCOL
    // @DisplayName: Enable use of specific protocol over virtual driver
    // @Description: Enabling this option starts selected protocol that will use this virtual driver
    // @Values: 0:Disabled,1:DroneCAN,4:PiccoloCAN,6:EFI_NWPMU,7:USD1,8:KDECAN,10:Scripting,11:Benewake,12:Scripting2
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PROTOCOL", 1, AP_CANManager::CANDriver_Params, _driver_type, AP_CANManager::Driver_Type_DroneCAN),

#if HAL_ENABLE_DRONECAN_DRIVERS
    // @Group: UC_
    // @Path: ../AP_DroneCAN/AP_DroneCAN.cpp
    AP_SUBGROUPPTR(_uavcan, "UC_", 2, AP_CANManager::CANDriver_Params, AP_DroneCAN),
#endif

    // index 3 was KDECAN

    // index 4 was CANTester

#if HAL_PICCOLO_CAN_ENABLE
    // @Group: PC_
    // @Path: ../AP_PiccoloCAN/AP_PiccoloCAN.cpp
    AP_SUBGROUPPTR(_piccolocan, "PC_", 5, AP_CANManager::CANDriver_Params, AP_PiccoloCAN),
#endif

    AP_GROUPEND
};
#endif
