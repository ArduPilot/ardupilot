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

#include "SIM_RF_NoopLoop.h"

using namespace SITL;

uint32_t RF_Nooploop::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{

    int32_t alt_scaled = 2560*alt_cm;
    buffer[0] = 0x57;
    buffer[1] = 0x00;
    buffer[2] = 0xff;
    buffer[3] = 0x00;
    buffer[4] = 0x9e;
    buffer[5] = 0x8f;
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    buffer[8] = alt_scaled >> 8 & 0xff;
    buffer[9] = alt_scaled >> 16 & 0xff;
    buffer[10] = alt_scaled >> 24 & 0xff;
    buffer[11] = 0x00;
    buffer[12] = 0x03;
    buffer[13] = 0x00;
    buffer[14] = 0x06;

    // calculate checksum:
    buffer[15] = 0;
    for (uint8_t i=0; i<15; i++) {
        buffer[15] += buffer[i];
    }
    return 16;
}
