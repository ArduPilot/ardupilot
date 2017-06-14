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

#include "AP_EcotronsEFI_UAVCAN.h"

extern const AP_HAL::HAL &hal;

#if HAL_WITH_UAVCAN

AP_EcotronsEFI_UAVCAN::AP_EcotronsEFI_UAVCAN(EFI_State& _efi_state, uint8_t efi_source_node_id)
    : AP_EcotronsEFI_Backend(_efi_state)
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            uint8_t listener_channel = ap_uavcan->register_efi_listener(this, efi_source_node_id);
            hal.console->printf("AP_EcotronsEFI_UAVCAN: EFI Listener registered to channel %d\n", listener_channel); 
        }
    }
}

AP_EcotronsEFI_UAVCAN::~AP_EcotronsEFI_UAVCAN()
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            ap_uavcan->remove_efi_listener(this);
        }
    }
}

void AP_EcotronsEFI_UAVCAN::update()
{
    // TODO: should I be using semaphores here? Should it be blocking?
    // Referred to compass/rangefinder/baro
    if (_sem->take_nonblocking()) {
        copy_to_frontend();
        _sem->give();
    }
}

void AP_EcotronsEFI_UAVCAN::handle_efi_msg(const EFI_State& message_efi_state)
{
    if (_sem->take_nonblocking()) {
        copy_state(message_efi_state, _internal_state);
        _sem->give();
    }
}

#endif