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

#include <string.h>
#include <AP_Math/AP_Math.h>
#include "RTCM3_Parser.h"

// return ID of found packet
uint16_t RTCM3_Parser::get_id(const uint8_t *pkt, uint8_t pkt_len) const
{
    if (pkt_len < 5) {
        // should not have been called!  This method should only be
        // called if find_packet returns true.
        return 0;
    }
    return (pkt[3]<<8 | pkt[4]) >> 4;
}

int16_t RTCM3_Parser::find_packet(const uint8_t *pkt, uint8_t buffer_len) const
{
    auto preamble_offset = offset_of_byte_in_buffer(RTCMv3_PREAMBLE, pkt, buffer_len);
    if (preamble_offset != 0) {
        return -buffer_len;
    }

    if (buffer_len < 3) {
        // might be a packet but we don't have a length
        return 0;
    }

    const uint16_t pkt_len = (pkt[1]<<8 | pkt[2]) & 0x3ff;
    if (pkt_len > 300) {
        // don't accept packets over 300 bytes long
        return -1;
    }

    // 1 byte preamble, 2 bytes length, n-bytes data, 2 bytes crc, 1-byte parity
    if (buffer_len < pkt_len + 6) {
        return 0;
    }

    // we have enough bytes for a complete packet; validate checksum:
    const uint8_t *parity = &pkt[pkt_len+3];
    uint32_t crc1 = (parity[0] << 16) | (parity[1] << 8) | parity[2];
    uint32_t crc2 = crc_crc24(pkt, pkt_len+3);
    if (crc1 != crc2) {
#if 1
        // checksum mis-match; search the buffer for the preamble to
        // allow many bytes to be discarded in case of corruption
        auto offset = offset_of_byte_in_buffer(RTCMv3_PREAMBLE, pkt, buffer_len);
        if (offset < 0) {
            return buffer_len;
        }
        return -offset;
#else
        // just discard 1 byte to save searching for the preamble
        return -1;
#endif
    }

    // checksum matches; 
    return pkt_len + 6;
}

#ifdef RTCM_MAIN_TEST
/*
  parsing test, taking a raw file captured from UART to u-blox F9
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

int main(int argc, const char *argv[])
{
    const char *fname = argv[1];
    int fd = ::open(fname, O_RDONLY);
    if (fd == -1) {
        perror(fname);
        ::exit(1);
    }
    RTCM3_Parser parser {};
    uint8_t b;
    while (::read(fd, &b, 1) == 1) {
        if (parser.read(b)) {
            const uint8_t *bytes;
            printf("packet len %u ID %u\n", parser.get_len(bytes), parser.get_id());
        }
    }
    return 0;
}
#endif
