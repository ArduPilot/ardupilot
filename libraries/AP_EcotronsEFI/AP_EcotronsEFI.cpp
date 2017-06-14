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

#include "AP_EcotronsEFI.h"
#include "AP_EcotronsEFI_UAVCAN.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_EcotronsEFI::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: Ecotrons EFI usage enabled (1) or disabled (0)
    // @Description: Enables or disables use of the Ecotrons UAV EFI system
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_ENABLE", 0, AP_EcotronsEFI, _enabled, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: _TYPE
    // @DisplayName: EFI communication type
    // @Description: What method of communication is used for EFI #1
    // @Values: 0:None,1:UAVCAN,2:Serial
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_TYPE", 1, AP_EcotronsEFI, _type[0], 1),

    // @Param: _UC_NODE
    // @DisplayName: EFI #1 UAVCAN Node ID
    // @Description: The source node ID for this EFI (if using UAVCAN)
    // @Range: 1 250
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_UC_NODE", 2, AP_EcotronsEFI, _uavcan_node_id[0], 1),

#if EFI_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: EFI #2 communication type
    // @Description: What method of communication is used for EFI #2
    // @Values: 0:None,1:UAVCAN,2:Serial
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("2_TYPE", 3, AP_EcotronsEFI, _type[1], 0),

    // @Param: 2_UC_NODE
    // @DisplayName: EFI #2 UAVCAN Node ID
    // @Description: The source node ID for this EFI (if using UAVCAN)
    // @Range: 1 250
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("2_UC_NODE", 4, AP_EcotronsEFI, _uavcan_node_id[1], 1),

#endif

    AP_GROUPEND
};

// Initialize parameters
AP_EcotronsEFI::AP_EcotronsEFI()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Initialize backends based on existing params
void AP_EcotronsEFI::init()
{
    if (_backend_count > 0) {
        // Init called twice, perhaps
        return;
    }

    // Otherwise, initialize backends as required
    for (uint8_t i = 0; i < EFI_MAX_INSTANCES; i++) {
        if (_type[i] == EcotronsEFI_COMMUNICATION_TYPE_UAVCAN) {
            hal.console->printf("AP_EcotronsEFI: Starting UAVCAN backend\n");
            _backends[_backend_count] = new AP_EcotronsEFI_UAVCAN(_state[_backend_count], _uavcan_node_id[_backend_count]);
            _backend_count++;
        } else if (_type[i] == EcotronsEFI_COMMUNICATION_TYPE_SERIAL) {
            // TODO: not supported yet
        }
    }
}

// Ask all backends to update the frontend
void AP_EcotronsEFI::update()
{
    for (uint8_t i = 0; i < _backend_count; i++) {
        _backends[i]->update();
    }
}