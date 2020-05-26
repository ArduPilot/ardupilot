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

#include "AP_VisualOdom_MAV.h"

#if HAL_VISUALODOM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_VisualOdom_MAV::AP_VisualOdom_MAV(AP_VisualOdom &frontend) :
    AP_VisualOdom_Backend(frontend)
{
}

// consume vision position estimate data and send to EKF. distances in meters
void AP_VisualOdom_MAV::handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, uint8_t reset_counter)
{
    const float scale_factor =  _frontend.get_pos_scale();
    Vector3f pos{x * scale_factor, y * scale_factor, z * scale_factor};

    // send attitude and position to EKF
    const float posErr = 0; // parameter required?
    const float angErr = 0; // parameter required?
    AP::ahrs().writeExtNavData(pos, attitude, posErr, angErr, time_ms, _frontend.get_delay_ms(), get_reset_timestamp_ms(reset_counter));

    // calculate euler orientation for logging
    float roll;
    float pitch;
    float yaw;
    attitude.to_euler(roll, pitch, yaw);

    // log sensor data
    AP::logger().Write_VisualPosition(remote_time_us, time_ms, pos.x, pos.y, pos.z, degrees(roll), degrees(pitch), degrees(yaw), reset_counter);

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();
}

#endif
