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
#include "RCInput_SoloLink.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

#define DEBUG 0
#if DEBUG
#define debug(fmt, args...) ::printf(fmt "\n", ##args)
#else
#define debug(fmt, args...)
#endif

extern const AP_HAL::HAL& hal;

using namespace Linux;

RCInput_SoloLink::RCInput_SoloLink()
{
    memset(&_packet, 0, sizeof(_packet));
}

void RCInput_SoloLink::init()
{
    if (!_socket.bind("0.0.0.0", PORT)) {
        AP_HAL::panic("failed to bind UDP socket");
    }

    // timeout is handled by poll() in SocketAPM
    _socket.set_blocking(true);

    return;
}

bool RCInput_SoloLink::_check_hdr(ssize_t len)
{
    if (len < (ssize_t) sizeof(_packet)) {
        hal.console->printf("RCInput: Packet too small (%zd), doesn't contain full frame\n",
                            len);
        return false;
    }

    uint64_t now_usec = AP_HAL::micros64();
    uint64_t delay = now_usec - _last_usec;

    if (_last_usec != 0 && delay > 40000) {
        debug("RCInput: no rc cmds received for %llu\n", (unsigned long long)delay);
    }
    _last_usec = now_usec;

    uint16_t seq = le16toh(_packet.seq);
    if (seq - _last_seq > 1) {
        debug("RCInput: gap in rc cmds : %u\n", seq - _last_seq);
    }
    _last_seq = seq;

    return true;
}

/* TODO: this should be a PollerThread or at least stop using a SchedThread */
void RCInput_SoloLink::_timer_tick(void)
{
    do {
        uint16_t channels[8];
        ssize_t r;

        r = _socket.recv(&_packet.buf, sizeof(_packet), 20);
        if (r < 0) {
            break;
        }

        if (!_check_hdr(r)) {
            break;
        }

        channels[0] = le16toh(_packet.channel[1]);
        channels[1] = le16toh(_packet.channel[2]);
        channels[2] = le16toh(_packet.channel[0]);

        for (unsigned int i = 3; i < 8; i++) {
            channels[i] = le16toh(_packet.channel[i]);
        }



        _update_periods(channels, 8);
    } while (true);
}
