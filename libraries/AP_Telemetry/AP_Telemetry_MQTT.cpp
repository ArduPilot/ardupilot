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

#include "AP_Telemetry_MQTT.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_Telemetry_MQTT::AP_Telemetry_MQTT(AP_Telemetry &frontend, AP_HAL::UARTDriver* uart) :
        AP_Telemetry_Backend(frontend, uart)
    {}

// update - provide an opportunity to read/send telemetry
void AP_Telemetry_MQTT::update()
{
    // exit immediately if no uart
    if (_uart == nullptr || _frontend._ahrs == nullptr) {
        return;
    }

    // send telemetry data once per second
    uint32_t now = AP_HAL::millis();
    if (_last_send_ms == 0 || (now - _last_send_ms) > 1000) {
        _last_send_ms = now;
        Location loc;
        if (_frontend._ahrs->get_position(loc)) {
            ::printf("lat:%ld lon:%ld alt:%ld\n",
                    (long)loc.lat,
                    (long)loc.lng,
                    (long)loc.alt);
        }
    }
}
