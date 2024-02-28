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
/*
  Send and receive JSON backend data to alow a second AP instance to ride along
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_JSON_MASTER_ENABLED
#define HAL_SIM_JSON_MASTER_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if HAL_SIM_JSON_MASTER_ENABLED

#include "SITL_Input.h"
#include <AP_HAL/utility/Socket_native.h>
#include <AP_Math/AP_Math.h>

namespace SITL {

class JSON_Master {
public:
    JSON_Master() {};

    // setup sockets
    void init(const int32_t num_slaves);

    // Receive PWM outs from ride along controlers
    void receive(struct sitl_input &input);

    // send vehicle state to ride along controlers
    void send(const struct sitl_fdm &output, const Vector3d &position);

private:

    struct socket_list {
        SocketAPM_native sock_in{true};
        SocketAPM_native sock_out{true};
        uint8_t instance;
        bool connected;
        socket_list *next;
    } _list;

    char *json_out;

    bool initialized;

};

}

#endif  // HAL_SIM_JSON_MASTER_ENABLED
