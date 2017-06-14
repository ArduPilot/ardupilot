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
#pragma once

#include "AP_EcotronsEFI.h"
#include "AP_EcotronsEFI_Backend.h"
#include <AP_UAVCAN/AP_UAVCAN.h>

class AP_EcotronsEFI_UAVCAN: public AP_EcotronsEFI_Backend {
public:
    // Constructor with initialization
    AP_EcotronsEFI_UAVCAN(EFI_State& _efi_state, uint8_t efi_source_node_id);

    // Destructor
    ~AP_EcotronsEFI_UAVCAN(void) override;

    // Update the state structure
    void update();

    // UAVCAN callback, called from AP_UAVCAN.cpp
    void handle_efi_msg(const EFI_State& message_efi_state);
};