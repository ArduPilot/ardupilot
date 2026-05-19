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
  Simulator for the GYUS42v2 Serial rangefinder
*/

#include "SIM_RF_GYUS42v2.h"

#include <stdio.h>
#include <string.h>

using namespace SITL;

uint32_t RF_GYUS42v2::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    const uint16_t alt_cm = alt_m * 100;

    if (buflen < 7) {
        return 0;
    }
    buffer[0] = 0x5A;
    buffer[1] = 0; // FIXME
    buffer[2] = 0; // FIXME
    buffer[3] = 0; // FIXME
    buffer[4] = alt_cm >> 8;
    buffer[5] = alt_cm & 0xFF;
    buffer[6] = 0;
    for (uint8_t i=0; i<6; i++) {
        buffer[6] += buffer[i];
    }

    return 7;
}
