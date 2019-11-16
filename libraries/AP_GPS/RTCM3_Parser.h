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

#include <stdint.h>

class RTCM3_Parser {
public:
    // process one byte, return true if packet found
    bool read(uint8_t b);

    // reset internal state
    void reset(void);

    // clear previous packet
    void clear_packet(void);
    
    // return length of found packet
    uint16_t get_len(const uint8_t *&bytes) const;

    // return ID of found packet
    uint16_t get_id(void) const;
    
private:
    const uint8_t RTCMv3_PREAMBLE = 0xD3;
    const uint32_t POLYCRC24 = 0x1864CFB;

    // raw packet, we shouldn't need over 300 bytes for the MB configs we use
    uint8_t pkt[300];

    // number of bytes in pkt[]
    uint16_t pkt_bytes;

    // length from header
    uint16_t pkt_len;

    // length of found packet
    uint16_t found_len;
    
    bool parse(void);
    void resync(void);
    uint32_t crc24(const uint8_t *bytes, uint16_t len);
};

