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
  Simulator for the Lanbao rangefinder 
*/

#include "SIM_RF_Lanbao.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>


using namespace SITL;

uint32_t RF_Lanbao::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    const uint16_t alt_mm = alt_m * 1000;

    buffer[0] = 0xA5;
    buffer[1] = 0x5A;
    buffer[2] = alt_mm >> 8;
    buffer[3] = alt_mm & 0xff;

    const uint16_t crc = calc_crc_modbus(buffer, 4);
    buffer[4] = crc & 0xff;
    buffer[5] = crc >> 8;

    return 6;
}
