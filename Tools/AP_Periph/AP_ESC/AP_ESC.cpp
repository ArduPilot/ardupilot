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

#include "AP_ESC.h"
#include "AP_ESC_hwing_esc.h"
#include "AP_ESC_APDHVPro.h"

#include "../can.h"


// table of user settable parameters
const AP_Param::GroupInfo AP_ESC::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: ESC telemetry communication type
    // @Description: Type of ESC telemetry 
    // @Values: 0:None; 1:HobbyWing; 2:APD-HVPro200
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_ESC, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: serial_port_num
    // @DisplayName: ESC telemetry serial port number
    // @Description: Used to define ESC telemetry serial port number
    // @Range: { 0, 2, 3, 1, 4, 5, 6, 7, 8, 9 };
    // @User: Advanced
    AP_GROUPINFO("_PORT", 3, AP_ESC, serial_port_num, 0),

    // @Param: serial_port
    // @DisplayName: ESC telemetry serial port number
    // @Description: Used to define ESC telemetry serial port number
    // @Range: { 0, 2, 3, 1, 4, 5, 6, 7, 8, 9 };
    // @User: Advanced
    AP_GROUPINFO("_BAUD", 3, AP_ESC, serial_port_baud, 0),

    AP_GROUPEND
};


AP_ESC::AP_ESC() {
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_ESC::init(void) {
    can_printf("init - %d", (int)type.get());
    switch ((Type)type.get()){
        case Type::NONE:
        break;
        case Type::HobbyWing:
            //backend = new HWESC_Telem(*this);
        break;
        case Type::APD_HVPro200:
            backend = new AP_ESC_APDHVPro(*this);
        break;
    }
}