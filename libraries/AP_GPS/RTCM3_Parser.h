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
 RTCMv3 parser, used to support moving baseline RTK mode between two
 GPS modules
*/
#pragma once
#include <stdint.h>

// maximum packet length with MAVLink GPS_RTCM_DATA is 4*180 as we
// handle up to 4 fragments
#define RTCM3_MAX_PACKET_LEN 720

class RTCM3_Parser {
public:

    // returns 0 if there's no packet.
    // returns positive packet length if packet is found
    // returns a negative number containing number of bytes to discard if packet not found
    int16_t find_packet(const uint8_t *buffer, uint8_t buffer_len) const;

    // return ID of found packet
    uint16_t get_id(const uint8_t *pkt, uint8_t len) const;

private:
    const uint8_t RTCMv3_PREAMBLE = 0xD3;

};
