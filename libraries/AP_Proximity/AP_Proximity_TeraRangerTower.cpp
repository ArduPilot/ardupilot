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

#include "AP_Proximity_TeraRangerTower.h"

#if HAL_PROXIMITY_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// update the state of the sensor
void AP_Proximity_TeraRangerTower::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_TRTOWER_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_TeraRangerTower::distance_max() const
{
    return 4.5f;
}
float AP_Proximity_TeraRangerTower::distance_min() const
{
    return 0.20f;
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_TeraRangerTower::read_sensor_data()
{
    if (_uart == nullptr) {
        return false;
    }

    uint16_t message_count = 0;
    int16_t nbytes = _uart->available();

    while (nbytes-- > 0) {
        char c = _uart->read();
        if (c == 'T' ) {
            buffer_count = 0;
        }

        buffer[buffer_count++] = c;

        // we should always read 19 bytes THxxxxxxxxxxxxxxxxC
        if (buffer_count >= 19){
            buffer_count = 0;

            // check if message has right CRC
            if (crc_crc8(buffer, 18) == buffer[18]){
                update_sector_data(0,   UINT16_VALUE(buffer[2],  buffer[3]));   // d1
                update_sector_data(45,  UINT16_VALUE(buffer[16], buffer[17]));  // d8
                update_sector_data(90,  UINT16_VALUE(buffer[14], buffer[15]));  // d7
                update_sector_data(135, UINT16_VALUE(buffer[12], buffer[13]));  // d6
                update_sector_data(180, UINT16_VALUE(buffer[10], buffer[11]));  // d5
                update_sector_data(225, UINT16_VALUE(buffer[8],  buffer[9]));   // d4
                update_sector_data(270, UINT16_VALUE(buffer[6],  buffer[7]));   // d3
                update_sector_data(315, UINT16_VALUE(buffer[4],  buffer[5]));   // d2

                message_count++;
            }
        }
    }
    return (message_count > 0);
}

// process reply
void AP_Proximity_TeraRangerTower::update_sector_data(int16_t angle_deg, uint16_t distance_cm)
{   
    // Get location on 3-D boundary based on angle to the object
    const AP_Proximity_Boundary_3D::Face face = boundary.get_face(angle_deg);
    if ((distance_cm != 0xffff) && !check_obstacle_near_ground(angle_deg, distance_cm * 0.001f)) {
        boundary.set_face_attributes(face, angle_deg, ((float) distance_cm) / 1000);
        // update OA database
        database_push(angle_deg, ((float) distance_cm) / 1000);
    } else {
        boundary.reset_face(face);
    }
    _last_distance_received_ms = AP_HAL::millis();
}

#endif // HAL_PROXIMITY_ENABLED
