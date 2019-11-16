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
#include "RTCM3_Parser.h"

// reset state
void RTCM3_Parser::reset(void)
{
    pkt_len = 0;
    pkt_bytes = 0;
    found_len = 0;
}

// clear previous packet
void RTCM3_Parser::clear_packet(void)
{
    if (found_len == 0) {
        return;
    }
    // clear previous packet
    if (pkt_bytes > found_len) {
        memmove(&pkt[0], &pkt[found_len], pkt_bytes-found_len);
        pkt_bytes -= found_len;
    } else {
        pkt_bytes = 0;
    }
    found_len = 0;
    pkt_len = 0;
}

// return length of found packet
uint16_t RTCM3_Parser::get_len(const uint8_t *&bytes) const
{
    if (found_len > 0) {
        bytes = &pkt[0];
    }
    return found_len;
}

// return ID of found packet
uint16_t RTCM3_Parser::get_id(void) const
{
    if (found_len == 0) {
        return 0;
    }
    return (pkt[3]<<8 | pkt[4]) >> 4;
}

// look for preamble to try to resync
void RTCM3_Parser::resync(void)
{
    const uint8_t *p = (const uint8_t *)memchr(&pkt[1], RTCMv3_PREAMBLE, pkt_bytes-1);
    if (p != nullptr) {
        const uint16_t idx = p - &pkt[0];
        memmove(&pkt[0], p, pkt_bytes - idx);
        pkt_bytes -= idx;
        if (pkt_bytes >= 3) {
            pkt_len = (pkt[1]<<8 | pkt[2]) & 0x3ff;
        } else {
            pkt_len = 0;
        }
    } else {
        reset();
    }
}

// parse packet
bool RTCM3_Parser::parse(void)
{
    const uint8_t *parity = &pkt[pkt_len+3];
    uint32_t crc1 = (parity[0] << 16) | (parity[1] << 8) | parity[2];
    uint32_t crc2 = crc24(pkt, pkt_len+3);
    if (crc1 != crc2) {
        resync();
        return false;
    }

    // we got a good packet
    found_len = pkt_len+6;
    return true;
}

// read in one byte, return true if a full packet is available
bool RTCM3_Parser::read(uint8_t byte)
{
    clear_packet();

    if (pkt_bytes > 0 && pkt[0] != RTCMv3_PREAMBLE) {
        resync();
    }

    if (pkt_bytes == 0 && byte != RTCMv3_PREAMBLE) {
        // discard
        return false;
    }

    if (pkt_bytes >= sizeof(pkt)) {
        // too long, resync
        resync();
    }

    pkt[pkt_bytes++] = byte;

    if (pkt_len == 0 && pkt_bytes >= 3) {
        pkt_len = (pkt[1]<<8 | pkt[2]) & 0x3ff;
        if (pkt_len == 0) {
            resync();
        }
    }

    if (pkt_len != 0 && pkt_bytes >= pkt_len + 6) {
        // got header, packet body and parity
        return parse();
    }

    // need more bytes
    return false;
}

/*
  calculate 24 bit RTCMv3 crc. We take an approach that saves memory
  and flash at the cost of higher CPU load. This makes it appropriate
  for use in the f103 AP_Periph nodes
  On a F765 this costs us approx 2ms of CPU per second of 5Hz all
  constellation RTCM data
*/
uint32_t RTCM3_Parser::crc24(const uint8_t *bytes, uint16_t len)
{
    uint32_t crc = 0;
    while (len--) {
        uint8_t b = *bytes++;
        const uint8_t idx = (crc>>16) ^ b;
        uint32_t crct = idx<<16;
        for (uint8_t j=0; j<8; j++) {
            crct <<= 1;
            if (crct & 0x1000000) {
                crct ^= POLYCRC24;
            }
        }
        crc = ((crc<<8)&0xFFFFFF) ^ crct;
    }
    return crc;
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
