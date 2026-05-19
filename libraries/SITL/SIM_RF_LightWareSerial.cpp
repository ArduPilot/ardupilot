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
  Simulator for the serial LightWare rangefinder 
*/

#include "SIM_RF_LightWareSerial.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>


using namespace SITL;

bool RF_LightWareSerial::check_synced()
{
    if (!synced) {
        // just try to slurp a buffer in one hit:
        char buffer[12] {};
        ssize_t n = read_from_autopilot(buffer, ARRAY_SIZE(buffer) - 1);
        if (n > 0) {
            if (!strncmp(buffer, "www\r\n", ARRAY_SIZE(buffer))) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Slurped a sync thing");
                synced = true;
            }
        }
    }
    return synced;
}

void RF_LightWareSerial::update(float range)
{
    if (!check_synced()) {
        return;
    }
    return SerialRangeFinder::update(range);
}

uint32_t RF_LightWareSerial::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    uint16_t alt_cm;
    if (alt_m > 100) {
        alt_cm = 13000;  // from datasheet
    } else {
        alt_cm = alt_m * 100;
    }
    return snprintf((char*)buffer, buflen, "%0.2f\r", alt_cm * 0.01f); // note tragic lack of snprintf return checking
}
