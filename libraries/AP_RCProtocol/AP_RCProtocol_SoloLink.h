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

/*
 * Binds a port and waits for UDP packets.  Packets contain a 64-bit
 * timestamp, a sequence number and 8 16-bit channels.  For some
 * reason it swaps channel ordering from AETR to ETAR (ie. switches
 * around first 3 received channels).
 *
 * Note that this protocol isn't actually *used* by ArduPilot on Solo!
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_SOLOLINK_ENABLED

#include "AP_RCProtocol_Backend.h"

#include <AP_HAL/utility/Socket_native.h>

class AP_RCProtocol_SoloLink : public AP_RCProtocol_Backend
{
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    void update() override;

private:

    void init();

    void thread_main();

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

    SocketAPM_native _socket{true};
    uint64_t _last_usec = 0;
    uint16_t _last_seq = 0;
    union packet _packet;

    bool init_done;  // eg. whether we have created timer thread entry
};

#endif  // AP_RCPROTOCOL_SOLOLINK_ENABLED
