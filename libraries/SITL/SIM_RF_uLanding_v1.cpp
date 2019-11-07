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
  Simulator for the uLanding v1 rangefinder
*/

#include "SIM_RF_uLanding_v1.h"

using namespace SITL;

uint32_t RF_uLanding_v1::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    buffer[0] = 0xFE;
    buffer[1] = 0; // unused?
    buffer[2] = alt_cm & 0xff;
    buffer[3] = alt_cm >> 8;
    buffer[4] = 0; // unused?
    // checksum:
    buffer[5] = buffer[1] + buffer[2] + buffer[3] + buffer[4];

    return 6;
}
