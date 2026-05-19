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
  Base class for simulator for the NoopLoop TOFSense-F/P Serial RangeFinders
*/

#include "SIM_RF_JRE.h"

using namespace SITL;

uint32_t RF_JRE::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    uint16_t status = 0;
    if (alt_m > 500) {
        status |= 0x2;  // NTRK, whatever that means...
    }

    const uint16_t alt_cm = alt_m * 100;

    buffer[0] = 'R';
    buffer[1] = 'A';
    buffer[2] = 0x01;
    buffer[3] = frame_count++;
    buffer[4] = alt_cm >> 8;    // altitide low
    buffer[5] = alt_cm & 0xff;  // Altitude-High, LSB
    buffer[6] = 0x00;  // reserved
    buffer[7] = 0x00;  // reserved
    buffer[8] = 0x00;  // reserved
    buffer[9] = 0x00;  // reserved
    buffer[10] = 0x00;  // FFT
    buffer[11] = 0x00;  // FFT
    buffer[12] = status >> 8;  // FFT
    buffer[13] = status & 0xff;  // FFT

    const uint16_t crc = crc16_ccitt_r(&buffer[0], 14, 0xffff, 0xffff);
    buffer[14] = crc & 0xff;
    buffer[15] = crc >> 8;

    return 16;
}
