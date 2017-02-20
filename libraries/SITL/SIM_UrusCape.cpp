/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  simulator connector for ardupilot version of UrusCape
*/

#include "SIM_UrusCape.h"

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define DEBUG_URUS_CAPE 1

namespace SITL {

UrusCape::UrusCape(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    sock(true)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // UrusCape keeps sending us packets. Not strictly necessary but
    // useful for debugging
    sock.bind("0.0.0.0", 6270);

    sock.reuseaddress();
    sock.set_blocking(false);
    fprintf(stdout, "bind\n");
}

/*
  Only for debugging.
*/
void UrusCape::send_debug()
{
#if DEBUG_URUS_CAPE
    fdm_packet_debug pkt;
    pkt.dummy = 0;
    pkt.onoff = onoff;
    sock.sendto(&pkt, sizeof(pkt), "127.0.0.1", 6271);
#endif
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void UrusCape::recv_fdm()
{

#if DEBUG_URUS_CAPE
    fdm_packet_debug pkt;

    uint8_t size_pkt = sizeof(pkt);
    uint8_t len_rcv;
    len_rcv = sock.recv(&pkt, sizeof(pkt), 0);

    if ((pkt.dummy) && (size_pkt == len_rcv)) {
        ::printf("onoff: %u dummy: %u len: %u\n", pkt.onoff, pkt.dummy, len_rcv);
        onoff = pkt.onoff;
    }
#endif

}

/*
  update the UrusCape simulation by one time step
 */
void UrusCape::update(const struct sitl_input &input)
{
    recv_fdm();

    if (time_now_us - last_debug_us > 2e6f) {
        last_debug_us = time_now_us;

#if DEBUG_URUS_CAPE
        if (onoff) {
            printf("Urus cape running ok...\n");
        }
        send_debug();
#endif
    }

    update_position();
}

} // namespace SITL
