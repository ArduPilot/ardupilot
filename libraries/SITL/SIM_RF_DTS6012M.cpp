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
  Simulator for the DTS6012M serial rangefinder
*/

#include "SIM_config.h"

#if AP_SIM_RF_DTS6012M_ENABLED

#include "SIM_RF_DTS6012M.h"
#include <AP_Math/crc.h>

using namespace SITL;

uint32_t RF_DTS6012M::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    const uint8_t PACKET_LEN = 23;

    if (buflen < PACKET_LEN) {
        abort();
    }

    // sensor sends 0xFFFF when out of range (confirmed on hardware)
    const uint16_t dist_mm = (alt_m * 1000 > 20000) ? 0xFFFF : (uint16_t)(alt_m * 1000);

    // 7-byte header
    buffer[0] = 0xA5;   // frame header
    buffer[1] = 0x03;   // device ID
    buffer[2] = 0x20;   // device type
    buffer[3] = 0x01;   // command echo (start stream)
    buffer[4] = 0x00;   // reserved
    buffer[5] = 0x00;   // data length high byte (14 = 0x000E, big-endian)
    buffer[6] = 0x0E;   // data length low byte

    // 14-byte data payload
    buffer[7]  = 0xFF;  // secondary target distance low byte (0xFFFF = invalid)
    buffer[8]  = 0xFF;  // secondary target distance high byte
    buffer[9]  = 0x00;  // secondary target correction low
    buffer[10] = 0x00;  // secondary target correction high
    buffer[11] = 0x00;  // secondary target intensity low
    buffer[12] = 0x00;  // secondary target intensity high
    buffer[13] = dist_mm & 0xFF;  // primary target distance low byte (little-endian, mm)
    buffer[14] = dist_mm >> 8;    // primary target distance high byte
    buffer[15] = 0x00;  // primary target correction low
    buffer[16] = 0x00;  // primary target correction high
    buffer[17] = 0x10;  // primary target intensity low byte (10000 = 0x2710 → 100% quality)
    buffer[18] = 0x27;  // primary target intensity high byte
    buffer[19] = 0x00;  // sunlight base low
    buffer[20] = 0x00;  // sunlight base high

    // CRC-16/MODBUS over bytes 0–20, stored big-endian (high byte first)
    const uint16_t crc = calc_crc_modbus(buffer, PACKET_LEN - 2);
    buffer[21] = crc >> 8;    // CRC high byte
    buffer[22] = crc & 0xFF;  // CRC low byte

    return PACKET_LEN;
}

#endif  // AP_SIM_RF_DTS6012M_ENABLED