/*
  MAVLink TUNNEL handling for COBS ethernet frame tunneling
 */

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

#include "GCS_config.h"

#if AP_MAVLINK_MSG_TUNNEL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "GCS.h"
#include <AP_Networking/AP_Networking_Config.h>

#if AP_NETWORKING_BACKEND_SWITCHPORT_MAVLINK_COBS
#include <AP_Networking/AP_Networking_SwitchPort_MAVLink_COBS.h>
#endif

/**
   handle a TUNNEL message
 */
void GCS_MAVLINK::handle_tunnel(const mavlink_message_t &msg)
{
    mavlink_tunnel_t packet;
    mavlink_msg_tunnel_decode(&msg, &packet);

#if AP_NETWORKING_BACKEND_SWITCHPORT_MAVLINK_COBS
    // Check if this is a COBS tunnel message
    if (AP_Networking_SwitchPort_MAVLink_COBS::is_cobs_payload_type(packet.payload_type)) {
        // Get or create a port for this (channel, sysid, compid) combination
        auto *cobs_port = AP_Networking_SwitchPort_MAVLink_COBS::get_or_create_port(
            chan, msg.sysid, msg.compid);
        if (cobs_port != nullptr) {
            cobs_port->handle_tunnel(packet.payload_type,
                                     packet.payload, packet.payload_length);
        }
        return;
    }
#endif

    // Other tunnel payload types can be handled here in the future
}

#endif  // AP_MAVLINK_MSG_TUNNEL_ENABLED
