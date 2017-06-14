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

#include "AP_EcotronsEFI_State.h"
#include <AP_HAL/AP_HAL.h>

class AP_EcotronsEFI; //forward declaration

class AP_EcotronsEFI_Backend {
public:    
    // Constructor with initialization
    AP_EcotronsEFI_Backend(EFI_State& _efi_state);

    // Virtual destructor that efi backends can override 
    virtual ~AP_EcotronsEFI_Backend(void) {}

    // Update the state structure
    virtual void update() = 0;

    // UAVCAN callback for backends that use UAVCAN
    virtual void handle_efi_msg(const EFI_State& message_efi_state) {};

protected:
    // Copies internal state to the frontend state
    void copy_to_frontend();

    // Copies state from one struct to another
    void copy_state(const EFI_State& src, EFI_State& dst);

    // Current frontend state of this EFI system
    EFI_State& efi_state;

    // Semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;    

    // Internal state for this driver (before copying to frontend)
    EFI_State _internal_state;
};

#include "AP_EcotronsEFI_UAVCAN.h"