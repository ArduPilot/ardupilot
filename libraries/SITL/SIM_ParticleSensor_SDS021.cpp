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
  simple particle sensor simulator class
*/

#include "SIM_ParticleSensor_SDS021.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

using namespace SITL;

ParticleSensor_SDS021::ParticleSensor_SDS021() :
    SerialDevice()
{
}

/*
  update particle sensor state
 */
void ParticleSensor_SDS021::update(const Location &loc)
{
    const uint64_t now = AP_HAL::millis();

    if (now - last_update_ms < 1000) {
        // we only provide a reading every second
        return;
    }
    last_update_ms = now;

    // push a reading into the socket...
    const float value10 = 8.7;
    const float value25 = 3.9;
    const bool should_corrupt = false;
    push_reading(value10, value25, should_corrupt);
}


bool ParticleSensor_SDS021::push_byte(const uint8_t byte)
{
    if (write_to_autopilot((char*)&byte, 1) != 1) {
        return false;
    }
    checksum += byte;
    return true;
}

void ParticleSensor_SDS021::push_reading(const float value10, const float value25, const bool should_corrupt)
{
    const char header_byte = 0xAA;
    if (!push_byte(header_byte)) {
        return;
    }

    const char commander_no = 0xC0;
    if (!push_byte(commander_no)) {
        return;
    }
    checksum = 0;
    const uint16_t pm25 = (uint16_t)(value25*10);
    const uint8_t pm25_low = (pm25 & 0xff);
    if (!push_byte(pm25_low)) {
        return;
    }
    const uint8_t pm25_high = ((pm25>>8) & 0xff);
    if (!push_byte(pm25_high)) {
        return;
    }
    const uint16_t pm10 = (uint16_t)(value10*10);
    const uint8_t pm10_low = (pm10 & 0xff);
    if (!push_byte(pm10_low)) {
        return;
    }
    const uint8_t pm10_high = ((pm10>>8) & 0xff);
    if (!push_byte(pm10_high)) {
        return;
    }
    const uint8_t id1 = 1;
    if (!push_byte(id1)) {
        return;
    }
    const uint8_t id2 = 0;
    if (!push_byte(id2)) {
        return;
    }

    if (!push_byte(checksum)) {
        return;
    }

    const char footer_byte = 0xAB; // Message Tail
    if (!push_byte(footer_byte)) {
        return;
    }
}
