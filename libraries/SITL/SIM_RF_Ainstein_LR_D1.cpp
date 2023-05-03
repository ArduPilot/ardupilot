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
  Simulator for the Ainstein LR-D1 Serial rangefinder
*/

#include "SIM_RF_Ainstein_LR_D1.h"

#include <stdio.h>
#include <string.h>

using namespace SITL;

uint32_t RF_Ainstein_LR_D1::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    const uint8_t PACKET_LEN = 32;

    if (buflen < PACKET_LEN) {
        return 0;
    }

    uint8_t malfunction_alert = 0;

    uint8_t snr;
    if (alt_cm > 10000) {
        // out of range @100m
        snr = 0;
        alt_cm = 146;
    } else {
        snr = 100;
    }


    buffer[0] = 0xEB;  // packet header msb
    buffer[1] = 0x90;  // packet header lsb
    buffer[2] = 0;  // device_id
    buffer[3] = 28;  // length
    buffer[4] = malfunction_alert;
    buffer[5] = 1;  // object count
    buffer[6] = alt_cm >> 8;
    buffer[7] = alt_cm & 0xff;
    buffer[8] = snr;
    buffer[9] = 0;  // speed high
    buffer[10] = 0;  // speed low
    buffer[11] = 0xff;
    buffer[12] = 0xff;
    buffer[13] = 0xff;
    buffer[14] = 0xff;
    buffer[15] = 0xff;
    buffer[16] = 0xff;
    buffer[17] = 0xff;
    buffer[18] = 0xff;
    buffer[19] = 0xff;
    buffer[20] = 0xff;
    buffer[21] = 0xff;
    buffer[22] = 0xff;
    buffer[23] = 0xff;
    buffer[24] = 0xff;
    buffer[25] = 0xff;
    buffer[26] = 0xff;
    buffer[27] = 0xff;
    buffer[28] = 0xff;
    buffer[29] = 0xff;
    buffer[30] = 0xff;

    uint32_t sum = 0;
    for (uint8_t i=3; i<31; i++) {
        sum += buffer[i];
    }
    buffer[31] = sum & 0xff;

    return PACKET_LEN;
}
