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
  Base class for serial proximity sensors
*/

#include "SIM_SerialProximitySensor.h"

#if AP_SIM_SERIALPROXIMITYSENSOR_ENABLED

#include <AP_Math/AP_Math.h>

#include <stdio.h>

using namespace SITL;

void SerialProximitySensor::update(const Location &location)
{
    // just send a chunk of data at 5Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < reading_interval_ms()) {
        return;
    }
    last_sent_ms = now;

    uint8_t data[255];
    const uint32_t packetlen = packet_for_location(location,
                                                   data,
                                                   ARRAY_SIZE(data));

    write_to_autopilot((char*)data, packetlen);
}

#endif  // AP_SIM_SERIALPROXIMITYSENSOR_ENABLED
