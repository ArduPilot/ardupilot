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

#include "AP_ESC.h"
#include <AP_HAL/Semaphores.h>

// forward declaration
class AP_ESC;

class AP_ESC_Backend {
public:
    // Constructor with initialization
    AP_ESC_Backend(AP_ESC &_frontend);

    // Virtaul destructor that ESC backend can override
    virtual ~AP_ESC_Backend(void);

    // update the state structure
    virtual bool update() = 0;

protected:
    // Copies internal state to the frontend state
    void copy_to_frontend();

    //  Semaphore for access to shared frontend data
    // HAL_Semaphore sem;

    // uint8_t get_uavcan_node_id(void) const;
    // AP_Enum<Type> get_esc_type(void) const;
    uint32_t get_esc_port(void) const;
    uint32_t get_esc_baud(void) const;

private:
    AP_ESC &frontend;
};
