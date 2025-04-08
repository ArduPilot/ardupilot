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
  Base class for serial rangefinders
*/

#include "SIM_SerialRangeFinder.h"

using namespace SITL;

void SerialRangeFinder::update(float range)
{
    // just send a chunk of data at 5Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < reading_interval_ms()) {
        return;
    }
    last_sent_ms = now;

    const uint16_t range_cm = uint16_t(range*100);
    uint8_t data[255];
    const uint32_t packetlen = packet_for_alt(range_cm,
                                              data,
                                              ARRAY_SIZE(data));

    write_to_autopilot((char*)data, packetlen);

    if (has_temperature()) {
        send_temperature();
    }

}

void SerialRangeFinder::send_temperature()
{
    // Use the simple underwater model to get temperature
    float rho, delta, theta;
    AP_Baro::SimpleUnderWaterAtmosphere(-0.5 * 0.001, rho, delta, theta); // get simulated temperature for 0.5m depth
    const float temperature = Aircraft::rand_normal(KELVIN_TO_C(SSL_AIR_TEMPERATURE * theta), 1); // FIXME pick a stddev based on data sheet

    uint8_t data[255];
    const uint32_t packetlen = packet_for_temperature(temperature,
                                                        data,
                                                        ARRAY_SIZE(data));
    write_to_autopilot((char*)data, packetlen);
}
