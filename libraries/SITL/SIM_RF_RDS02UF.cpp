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
  Simulator for the RDS02UF rangefinder
*/

#include "SIM_RF_RDS02UF.h"

#include <stdio.h>
#include <AP_Math/crc.h>

using namespace SITL;

uint32_t RF_RDS02UF::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    const uint16_t fc_code = 0x3ff;  // NFI what this means
    const uint16_t data_fc = 0x70c;  // NFI what this means

    // bodgy fixed-length response to keep things simple:
    union response_t {
        struct {
            uint8_t header1;
            uint8_t header2;
            uint8_t address;
            uint8_t error_code;
            uint8_t fc_code_l;
            uint8_t fc_code_h;
            uint8_t length_l;
            uint8_t length_h;
            uint8_t data0;  // used
            uint8_t data1;  // used
            uint8_t data2;
            uint8_t data3;
            uint8_t data4;
            uint8_t data5;  // distance-low
            uint8_t data6;  // distance-high
            uint8_t data7;
            uint8_t data8;
            uint8_t data9;
            uint8_t crc8;
            uint8_t footer1;
            uint8_t footer2;
        };
        uint8_t buffer[21];
    } response{};

    response.header1 = 0x55;
    response.header2 = 0x55;
    response.fc_code_l = fc_code & 0xff;
    response.fc_code_h = fc_code >> 8;
    response.length_l = 10;
    response.length_h = 0;
    response.data0 = data_fc & 0xff;
    response.data1 = data_fc >> 8;
    response.data5 = alt_cm  & 0xff;
    response.data6 = alt_cm >> 8;
    response.crc8 = crc8_rds02uf(&response.buffer[2], 16);
    response.footer1 = 0xAA;
    response.footer2 = 0xAA;

    if (buflen < ARRAY_SIZE(response.buffer)) {
        AP_HAL::panic("Too short a buffer");
    }

    memcpy(buffer, response.buffer, ARRAY_SIZE(response.buffer));

    return ARRAY_SIZE(response.buffer);
}
