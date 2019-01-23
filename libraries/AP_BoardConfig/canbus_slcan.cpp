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

#if HAL_WITH_UAVCAN
#include "AP_BoardConfig_CAN.h"

const AP_Param::GroupInfo AP_BoardConfig_CAN::SLCAN_Interface::var_info[] = {
    // @Param: CPORT
    // @DisplayName: SLCAN Route
    // @Description: CAN Driver ID to be routed to SLCAN, 0 means no routing
    // @Values: 0:Disabled,1:First driver,2:Second driver
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CPORT", 1, AP_BoardConfig_CAN::SLCAN_Interface, _can_port, 1),

    // @Param: SERNUM
    // @DisplayName: SLCAN Serial Port
    // @Description: Serial Port ID to be used for temporary SLCAN iface, -1 means no temporary serial. This parameter is automatically reset on reboot or on timeout. See CAN_SLCAN_TIMOUT for timeout details
    // @Values: -1:Disabled,0:Serial0,1:Serial1,2:Serial2,3:Serial3,4:Serial4,5:Serial5,6:Serial6
    // @User: Standard
    AP_GROUPINFO("SERNUM", 2, AP_BoardConfig_CAN::SLCAN_Interface, _ser_port, -1),

    // @Param: TIMOUT
    // @DisplayName: SLCAN Timeout
    // @Description: Duration of inactivity after which SLCAN is switched back to original protocol in seconds.
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("TIMOUT", 3, AP_BoardConfig_CAN::SLCAN_Interface, _timeout, 0),

    AP_GROUPEND
};

#endif
