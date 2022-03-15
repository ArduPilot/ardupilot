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

#include "AP_EFI.h"
#include "AP_EFI_State.h"
#include <AP_HAL/Semaphores.h>

class AP_EFI; //forward declaration

class AP_EFI_Backend {
public:    
    // Constructor with initialization
    AP_EFI_Backend(AP_EFI &_frontend);

    // Virtual destructor that efi backends can override 
    virtual ~AP_EFI_Backend(void) {}

    // Update the state structure
    virtual void update() = 0;

protected:
    // Copies internal state to the frontend state
    void copy_to_frontend();

    // Internal state for this driver (before copying to frontend)
    EFI_State internal_state;

    int8_t get_uavcan_node_id(void) const;
    float get_coef1(void) const;
    float get_coef2(void) const;
    float get_ecu_dn(void) const;

    HAL_Semaphore &get_sem(void);

private:
    AP_EFI &frontend;
};
