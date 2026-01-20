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

using namespace SITL;

uint32_t RF_Ainstein_LR_D1::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    const uint8_t PACKET_LEN = 32;

    if (buflen < PACKET_LEN) {
        return 0;
    }

    uint8_t malfunction_alert = 0;

    uint16_t alt_cm = alt_m * 100;

    uint8_t snr = (alt_cm == 0xFFFF) ? 0 : 100;
    if (alt_m > 525) {
        // overheats at 525 metres:
        malfunction_alert |= 1U << 0;
    }
    if (alt_m > 500) {
        // out of range @500m
        snr = 0;
    }
    if (alt_m*100 > 65535) {
        malfunction_alert |= 1U << 7;  // AltitudeReading alert
    }

    buffer[0] = 0xEB;  // packet header msb
    buffer[1] = 0x90;  // packet header lsb
    buffer[2] = 0;  // device_id
    buffer[3] = 28;  // length
    buffer[4] = malfunction_alert;
    buffer[5] = 1;  // object count
    buffer[6] = HIGHBYTE(alt_cm);
    buffer[7] = LOWBYTE(alt_cm);
    buffer[8] = snr;
    buffer[9] = 0;  // speed high
    buffer[10] = 0;  // speed low
    memset(&buffer[11], 0xff, 20); // unused
    buffer[31] = crc_sum_of_bytes(&buffer[3], PACKET_LEN-4); // minus headerMSB, headerLSB, device_id and checksum

    return PACKET_LEN;
}
