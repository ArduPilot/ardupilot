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
  Simulator for the TeraRangerTower proximity sensor
*/

#include "SIM_config.h"

#if AP_SIM_PS_TERARANGERTOWER_ENABLED

#include "SIM_PS_TeraRangerTower.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include <errno.h>

using namespace SITL;

uint32_t PS_TeraRangerTower::packet_for_location(const Location &location,
                                           uint8_t *data,
                                           uint8_t buflen)
{
    return 0;
}

void PS_TeraRangerTower::update_output(const Location &location)
{
    const uint32_t now = AP_HAL::millis();
    if (last_output_time_ms == 0) {
        last_output_time_ms = now;
        return;
    }
    const uint32_t time_delta = now - last_output_time_ms;
    if (time_delta < 200) {  // 5Hz update
        return;
    }
    last_output_time_ms = now;

    struct PACKED  {
        uint8_t HEADER1{'T'};
        uint8_t HEADER2{'H'};
        uint16_t measurements[8];
        uint8_t checksum;
    } send_buffer;

    for (uint8_t i=0; i<8; i++) {
        const uint16_t bf_angle = (360 - (i * 45)) % 360;
        float distance = measure_distance_at_angle_bf(location, bf_angle);
        // ::fprintf(stderr, "SIM: %f=%fm\n", current_degrees_bf, distance);
        if (distance > MAX_RANGE) {
            send_buffer.measurements[i] = 0xffff;
        } else {
            send_buffer.measurements[i] = htobe16(distance * 1000);
        }
    }

    send_buffer.checksum = crc_crc8((uint8_t*)&send_buffer, 18);

    const ssize_t ret = write_to_autopilot((const char*)&send_buffer, sizeof(send_buffer));
    if (ret != sizeof(send_buffer)) {
        abort();
    }
}

void PS_TeraRangerTower::update(const Location &location)
{
    update_output(location);
}

#endif  // AP_SIM_PS_TERARANGERTOWER_ENABLED
