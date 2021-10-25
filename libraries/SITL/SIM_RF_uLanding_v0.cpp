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
  Simulator for the USD1 v0 rangefinder
*/

#include "SIM_RF_USD1_v0.h"

using namespace SITL;

uint32_t RF_USD1_v0::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    const uint16_t reading = alt_cm / 2.5f;
    buffer[0] = 0x48;
    buffer[1] = reading & 0x7f;
    buffer[2] = (reading >> 7) & 0xff;

    // the detection routine is crap, frankly.  Needs lots of bytes
    // *in one read* to work.
    buffer[3] = 0x48;
    buffer[4] = reading & 0x7f;
    buffer[5] = (reading >> 7) & 0xff;
    return 6;
}
