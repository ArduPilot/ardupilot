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
#pragma once

#include "AP_VisualOdom.h"

#if HAL_VISUALODOM_ENABLED

class AP_VisualOdom_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_VisualOdom_Backend(AP_VisualOdom &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() const;

    // consume vision_position_delta mavlink messages
    void handle_vision_position_delta_msg(const mavlink_message_t &msg);

    // consume vision pose estimate data and send to EKF. distances in meters
    virtual void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter) = 0;

    // consume vision velocity estimate data and send to EKF, velocity in NED meters per second
    virtual void handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter) = 0;

    // request sensor's yaw be aligned with vehicle's AHRS/EKF attitude
    virtual void request_align_yaw_to_ahrs() {}

    // handle request to align position with AHRS
    virtual void align_position_to_ahrs(bool align_xy, bool align_z) {}

    // arming check - by default no checks performed
    virtual bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const { return true; }

protected:

    // returns the system time of the last reset if reset_counter has not changed
    // updates the reset timestamp to the current system time if the reset_counter has changed
    uint32_t get_reset_timestamp_ms(uint8_t reset_counter);

    AP_VisualOdom::VisualOdom_Type get_type(void) const {
        return _frontend.get_type();
    }

    // Logging Functions
    void Write_VisualOdom(float time_delta, const Vector3f &angle_delta, const Vector3f &position_delta, float confidence);
    void Write_VisualPosition(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, float pos_err, float ang_err, uint8_t reset_counter, bool ignored);
    void Write_VisualVelocity(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, bool ignored);

    AP_VisualOdom &_frontend;   // reference to frontend
    uint32_t _last_update_ms;   // system time of last update from sensor (used by health checks)

    // reset counter handling
    uint8_t _last_reset_counter;    // last sensor reset counter received
    uint32_t _reset_timestamp_ms;   // time reset counter was received
};

#endif  // HAL_VISUALODOM_ENABLED
