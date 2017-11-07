/*
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <unistd.h>

#include <AP_HAL/utility/Socket.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "RCInput.h"

namespace Linux {

class RCInput_SoloLink : public RCInput
{
public:
    RCInput_SoloLink();

    void init();
    void _timer_tick();

private:
    static const unsigned int PACKET_LEN = 26;
    static const unsigned int PORT = 5005;

    union packet {
        struct PACKED {
            uint64_t timestamp_usec;
            le16_t seq;
            le16_t channel[8];
        };
        uint8_t buf[PACKET_LEN];
    };

    bool _check_hdr(ssize_t len);

    SocketAPM _socket{true};
    uint64_t _last_usec = 0;
    uint16_t _last_seq = 0;
    union packet _packet;
};

}
