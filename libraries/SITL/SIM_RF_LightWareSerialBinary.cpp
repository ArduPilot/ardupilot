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
  Simulator for the serial LightWare rangefinder - binary mode
*/

#include "SIM_RF_LightWareSerialBinary.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>


using namespace SITL;

uint32_t RF_LightWareSerialBinary::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    const uint16_t alt_cm = alt_m * 100;

    // high byte is second 7 bits but high bit set
    buffer[0] = ((alt_cm >> 7) & 0x7f) | (1<<7);
    // low byte is just first 7 bits
    buffer[1] = alt_cm & 0x7f;
    return 2;
}
