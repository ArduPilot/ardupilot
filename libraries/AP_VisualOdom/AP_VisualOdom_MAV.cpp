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

#include "AP_VisualOdom_config.h"

#if AP_VISUALODOM_MAV_ENABLED

#include "AP_VisualOdom_MAV.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

// consume vision pose estimate data and send to EKF. distances in meters
// quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
void AP_VisualOdom_MAV::handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter, int8_t quality)
{
    const float scale_factor =  _frontend.get_pos_scale();
    Vector3f pos{x * scale_factor, y * scale_factor, z * scale_factor};

    posErr = constrain_float(posErr, _frontend.get_pos_noise(), 100.0f);
    angErr = constrain_float(angErr, _frontend.get_yaw_noise(), 1.5f);

    // record quality
    _quality = quality;

    // send attitude and position to EKF if quality OK
    bool consume = (_quality >= _frontend.get_quality_min());
    if (consume) {
        AP::ahrs().writeExtNavData(pos, attitude, posErr, angErr, time_ms, _frontend.get_delay_ms(), get_reset_timestamp_ms(reset_counter));
    }

    // calculate euler orientation for logging
    float roll;
    float pitch;
    float yaw;
    attitude.to_euler(roll, pitch, yaw);

#if HAL_LOGGING_ENABLED
    // log sensor data
    Write_VisualPosition(remote_time_us, time_ms, pos.x, pos.y, pos.z, degrees(roll), degrees(pitch), degrees(yaw), posErr, angErr, reset_counter, !consume, _quality);
#endif

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();
}

// consume vision velocity estimate data and send to EKF, velocity in NED meters per second
// quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
void AP_VisualOdom_MAV::handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, int8_t quality)
{
    // record quality
    _quality = quality;

    // send velocity to EKF if quality OK
    bool consume = (_quality >= _frontend.get_quality_min());
    if (consume) {
        AP::ahrs().writeExtNavVelData(vel, _frontend.get_vel_noise(), time_ms, _frontend.get_delay_ms());
    }

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();

#if HAL_LOGGING_ENABLED
    Write_VisualVelocity(remote_time_us, time_ms, vel, reset_counter, !consume, _quality);
#endif
}

#endif  // AP_VISUALODOM_MAV_ENABLED
