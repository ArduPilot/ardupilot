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
#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_SOLOLINK_ENABLED

#include "AP_RCProtocol_SoloLink.h"

#include <AP_HAL/utility/Socket_native.h>
#include <AP_HAL/utility/sparse-endian.h>

#ifndef AP_SOCKET_NATIVE_ENABLED
#error "need native"
#endif

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>

#include <GCS_MAVLink/GCS.h>

#include <AP_HAL/AP_HAL.h>

#define DEBUG 0
#if DEBUG
#define debug(fmt, args...) ::printf(fmt "\n", ##args)
#else
#define debug(fmt, args...)
#endif

extern const AP_HAL::HAL& hal;

void AP_RCProtocol_SoloLink::init()
{
    if (!_socket.bind("0.0.0.0", PORT)) {
        AP_HAL::panic("failed to bind UDP socket");
    }

    // timeout is handled by poll() in SocketAPM
    _socket.set_blocking(true);

    return;
}

bool AP_RCProtocol_SoloLink::_check_hdr(ssize_t len)
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

void AP_RCProtocol_SoloLink::update()
{
    if (!init_done) {
        init_done = true;
        if (!hal.scheduler->thread_create(
                FUNCTOR_BIND_MEMBER(&AP_RCProtocol_SoloLink::thread_main, void),
                "RCSoloLink",
                512,
                AP_HAL::Scheduler::PRIORITY_RCIN,
                1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to create RC thread");
        }
    }
}

void AP_RCProtocol_SoloLink::thread_main(void)
{
    init();

    do {
        uint16_t channels[8];
        ssize_t r;

        r = _socket.recv(&_packet.buf, sizeof(_packet), 20);
        if (r < 0) {
            continue;
        }

        if (!_check_hdr(r)) {
            continue;
        }

        channels[0] = le16toh(_packet.channel[1]);
        channels[1] = le16toh(_packet.channel[2]);
        channels[2] = le16toh(_packet.channel[0]);

        for (unsigned int i = 3; i < 8; i++) {
            channels[i] = le16toh(_packet.channel[i]);
        }


        add_input(
            ARRAY_SIZE(channels),
            channels,
            false  // in_failsafe
         );
    } while (true);
}

#endif  // AP_RCPROTOCOL_SOLOLINK_ENABLED
