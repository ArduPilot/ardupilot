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
  Simulator for the Generic Chinese Sonar rangefinder : A02YYUW; ME007YS
*/

#include "SIM_RF_SinoSonar_Serial.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>


using namespace SITL;

uint32_t RF_SinoSonar_Serial::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    const uint16_t alt_mm = uint16_t(alt_m * 1000);

    buffer[0] = 0xFF;
    buffer[1] = alt_mm >> 8;
    buffer[2] = alt_mm & 0xff;
    buffer[3] = (buffer[0] + buffer[1] + buffer[2]) & 0x00FF;

    return 4;
}
