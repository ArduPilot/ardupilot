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

#include "AP_SwarmMesh_Backend.h"

#if AP_SWARMMESH_SITL_ENABLED

#include <AP_HAL/utility/Socket_native.h>
#include <SITL/SIM_config.h>

class AP_SwarmMesh_SITL : public AP_SwarmMesh_Backend
{
public:
    AP_SwarmMesh_SITL(AP_SwarmMesh &frontend);

protected:
    bool     transport_ready() const override;
    uint32_t transport_available() override;
    int16_t  transport_read() override;
    uint32_t transport_txspace() override;
    void     transport_write(const uint8_t *buf, uint16_t len) override;

private:
#if AP_SIM_SWARMMESH_LOSS_ENABLED
    // true if this datagram should be discarded, per SIM_SWARM_LOSS
    bool packet_lost() const;
#endif

    SocketAPM_native _sock{true};   // true = datagram (UDP)
    bool     _sock_ok = false;

    // datagram receive buffer: holds one complete SwarmMesh packet at a time
    uint8_t  _rx_buf[SWARMMESH_MSG_BUF_MAX];
    uint16_t _rx_buf_len = 0;
    uint16_t _rx_buf_pos = 0;
};

#endif  // AP_SWARMMESH_SITL_ENABLED
