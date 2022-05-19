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

/*
 * This library aims to read data from Electronic Speed Controller
 * 
 *
 * Authors: Pradeep CK
 * With direction from Andrew Tridgell
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "AP_ESC_Backend.h"

typedef struct {
    uint32_t counter;
    uint16_t throttle_req;
    uint16_t throttle;
    float rpm;
    float voltage;
    float phase_current;
    float current;
    uint8_t mos_temperature;
    uint8_t cap_temperature;
    uint16_t status;
    uint32_t error_count;
} esc_packet_t;



class AP_ESC
{
public:
    friend class AP_ESC_Backend;

    const AP_HAL::HAL &hal = AP_HAL::get_HAL();

    uint32_t serial_port_num;

    uint32_t serial_port_baud;

    enum class Type : uint8_t {
        NONE = 0,
        HobbyWing = 1,
        APD_HVPro200 = 2,
    };

    // Parameter info
    static const struct AP_Param::GroupInfo var_info[];

    // Initialize parameters
    AP_ESC();

    // Initialization
    void init(void);

    //  Refresh the data
    void update(void);

private:

    AP_Enum<Type> type;

    esc_packet_t esc_packet;

    AP_ESC_Backend *backend;
};
