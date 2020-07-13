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
  Simulator for the NMEA Serial rangefinder
*/

#include "SIM_RF_NMEA.h"

#include <GCS_MAVLink/GCS.h>

#include <stdio.h>
#include <string.h>

using namespace SITL;

uint32_t RF_NMEA::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
// Format 2 DBT NMEA mode (e.g. $SMDBT,5.94,f,1.81,M,67)
// Format 3 DPT NMEA mode (e.g. $SMDPT,1.81,0.066)

    ssize_t ret = snprintf((char*)buffer, buflen, "$SMDPT,%f,%f", alt_cm/100.0f, 0.01f);
    uint8_t checksum = 0;
    for (uint8_t i=1; i<ret; i++) { // 1 because the initial $ is skipped
        checksum ^= buffer[i];
    }
    ret += snprintf((char*)&buffer[ret], buflen-ret, "*%02X\r\n", checksum);
    return ret;
}
