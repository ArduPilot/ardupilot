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

#include "SIM_config.h"

#if AP_SIM_JSON_MASTER_ENABLED

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
        uint32_t seq;       // last consumed shared-memory channel seq
        socket_list *next;
    } _list;

    // true when the ride-along exchange runs over the cluster's shared
    // memory segment (--cluster given) instead of the UDP sockets
    bool shmem_transport(void) const;

    // shared-memory equivalent of the UDP receive loop: block until the
    // slave publishes a fresh servo packet in its slot's channel
    void receive_shmem(struct socket_list &list, void *pkt, uint32_t pkt_len);

    char *json_out;

    bool initialized;

    uint8_t slave_count;

    // sequence number of our published fdm channel (shmem transport)
    uint32_t fdm_seq;

    // wall-clock window for the periodic frames/s report
    uint64_t report_wall_us;
    uint32_t report_frames;

};

}

#endif  // AP_SIM_JSON_MASTER_ENABLED
