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
  Simulator for the MAVLink Serial rangefinder
*/

#include "SIM_RF_MAVLink.h"

#include <GCS_MAVLink/GCS.h>

#include <stdio.h>
#include <string.h>

using namespace SITL;

uint32_t RF_MAVLink::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    // we share channels with the ArduPilot binary!

    // we're swiping the Vicon's channel here.  If it causes issues we
    // may need to allocate an additional mavlink channel for the rangefinder
    const mavlink_channel_t mavlink_ch = (mavlink_channel_t)(MAVLINK_COMM_0+5);
    if (!valid_channel(mavlink_ch)) {
        AP_HAL::panic("Invalid mavlink channel");
    }

    mavlink_message_t msg;
    const uint8_t system_id = 32;
    const uint8_t component_id = 32;
    const mavlink_distance_sensor_t distance_sensor{
        time_boot_ms: AP_HAL::millis(),
        min_distance: 10, // cm
        max_distance: 10, // cm
        current_distance: alt_cm,
        type: 0,
        id: 72, // ID
        MAV_SENSOR_ROTATION_PITCH_270,
        255, // 255 is unknown covariance
        0, // 0 is unknown horizontal fov
        0, // 0 is unknown vertical fov
        {0,0,0,0} // unknown/unused quat
    };
    const uint16_t len = mavlink_msg_distance_sensor_encode(system_id,
                                                            component_id,
                                                            &msg,
                                                            &distance_sensor);
    if (len > buflen) {
        AP_HAL::panic("Insufficient buffer passed in");
    }
    const uint16_t retlen = mavlink_msg_to_send_buffer(buffer, &msg);
    return retlen;
}
