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
  Base class for simulator for the Benewake Serial RangeFinders
*/

#include "SIM_RF_Benewake.h"

using namespace SITL;

uint32_t RF_Benewake::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    buffer[0] = 0x59;
    buffer[1] = 0x59;
    buffer[3] = alt_cm >> 8;
    buffer[2] = alt_cm & 0xff;
    buffer[4] = byte4();
    buffer[5] = byte5();
    buffer[6] = byte6();
    buffer[7] = byte7();

    // calculate checksum:
    buffer[8] = 0;
    for (uint8_t i=0; i<8; i++) {
        buffer[8] += buffer[i];
    }
    return 9;
}
