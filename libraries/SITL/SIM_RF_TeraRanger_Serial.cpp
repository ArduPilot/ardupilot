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
  Base class for simulator for the TeraRanger Serial RangeFinders
*/

#include "SIM_RF_TeraRanger_Serial.h"

using namespace SITL;

uint32_t RF_TeraRanger_Serial::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    uint16_t alt_mm = alt_cm * 10;
    buffer[0] = 0x54; //header byte
    buffer[1] = alt_mm >> 8; //MSB mm
    buffer[2] = alt_mm & 0xff; //LSB mm

    if (alt_cm > 3000) {
       buffer[3] = 0xC4; //full strength, out of range, no overtemp
    }
    else {
       buffer[3] = 0xC0; //full strength, no reading error, no overtemp
    }
    
    // calculate CRC8:
    buffer[4] = crc_crc8(buffer,4);;

    return 5;
}

