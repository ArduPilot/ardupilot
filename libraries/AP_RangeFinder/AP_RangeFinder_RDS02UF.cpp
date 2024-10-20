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

/**
 * RDS02UF Note:
 * Sensor range scope 1.5m~20.0m
 * Azimuth Coverage ±17°,Elevation Coverage ±3°
 * Frame Rate 20Hz
 */

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_RDS02UF_ENABLED

#include "AP_RangeFinder_RDS02UF.h"
#include <AP_HAL/AP_HAL.h>

#define RDS02_HEAD                  0x55
#define RDS02_END                   0xAA
#define RDS02UF_PRE_DATA_LEN        6
#define RDS02_TARGET_INFO_FC        0x070C
#define RDS02UF_IGNORE_ID_BYTE      0x0F0F
#define RDS02UF_UAV_PRODUCTS_ID     0x03FF
#define RDS02UF_IGNORE_CRC          0xFF
#define RDS02UF_NO_ERR              0x00

bool AP_RangeFinder_RDS02UF::get_reading(float &distance_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint32_t nbytes = uart->read(&u.parse_buffer[body_length],
                                 ARRAY_SIZE(u.parse_buffer)-body_length);
    if (nbytes == 0) {
        return false;
    }
    body_length += nbytes;

    move_header_in_buffer(0);

    // header byte 1 is correct.
    if (body_length < ARRAY_SIZE(u.parse_buffer)) {
        // need a full buffer to have a valid message...
        return false;
    }

    if (u.packet.headermagic2 != RDS02_HEAD) {
        move_header_in_buffer(1);
        return false;
    }

    const uint16_t read_len = u.packet.length_h << 8 | u.packet.length_l;
    if (read_len != RDS02UF_DATA_LEN) {
        // we can only accept the fixed length message
        move_header_in_buffer(1);
        return false;
    }

    // check for the footer signatures:
    if (u.packet.footermagic1 != RDS02_END) {
        move_header_in_buffer(1);
        return false;
    }
    if (u.packet.footermagic2 != RDS02_END) {
        move_header_in_buffer(1);
        return false;
    }

    // calculate checksum
    const uint8_t checksum = crc8_rds02uf(&u.parse_buffer[2], RDS02UF_PRE_DATA_LEN + RDS02UF_DATA_LEN);
    if (u.packet.checksum != checksum && u.packet.checksum != RDS02UF_IGNORE_CRC) {
        move_header_in_buffer(1);
        return false;
    }

    const uint16_t fc_code = (u.packet.fc_high << 8 | u.packet.fc_low);
    if (fc_code == RDS02UF_UAV_PRODUCTS_ID && u.packet.error_code == RDS02UF_NO_ERR) {
        // get target information
        const uint16_t read_info_fc = (u.packet.data[1] << 8 | u.packet.data[0]);
        if ((read_info_fc & RDS02UF_IGNORE_ID_BYTE) == RDS02_TARGET_INFO_FC) {
            // read_info_fc = 0x70C + ID * 0x10, ID: 0~0xF
            distance_m = (u.packet.data[6] * 256 + u.packet.data[5]) * 0.01f;
            state.last_reading_ms = AP_HAL::millis();
        }
    }

    // consume this message:
    move_header_in_buffer(ARRAY_SIZE(u.parse_buffer));

    return true;
}

// find a RDS02UF message in the buffer, starting at
// initial_offset.  If found, that message (or partial message) will
// be moved to the start of the buffer.
void AP_RangeFinder_RDS02UF::move_header_in_buffer(uint8_t initial_offset)
{
    uint8_t* header_ptr = (uint8_t*)memchr(&u.parse_buffer[initial_offset], RDS02_HEAD, body_length - initial_offset);
    if (header_ptr != nullptr) {
        size_t header_offset = header_ptr - &u.parse_buffer[0];
        if (header_offset != 0) {
            // header was found, but not at index 0; move it back to start of array
            memmove(u.parse_buffer, header_ptr, body_length - header_offset);
            body_length -= header_offset;
        }
    } else {
        // no header found; reset buffer
        body_length = 0;
    }
}

#endif  // AP_RANGEFINDER_RDS02UF_ENABLED

