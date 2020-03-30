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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_VisualOdom.h"

class AP_VisualOdom_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_VisualOdom_Backend(AP_VisualOdom &frontend);

	// return true if sensor is basically healthy (we are receiving data)
	bool healthy() const;

    // consume VISION_POSITION_DELTA MAVLink message
	virtual void handle_msg(const mavlink_message_t &msg) {};

	// general purpose methods to consume position estimate data and send to EKF
	// distances in meters, roll, pitch and yaw are in radians
    void handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw);
    void handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude);

    // calibrate camera attitude to align with vehicle's AHRS/EKF attitude
    void align_sensor_to_vehicle() { _align_camera = true; }

    // arming check
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

protected:

    // apply rotation and correction to position
    void rotate_and_correct_position(Vector3f &position) const;

    // rotate attitude using _yaw_trim
    void rotate_attitude(Quaternion &attitude) const;

    // use sensor provided position and attitude to calculate rotation to align sensor with AHRS/EKF attitude
    bool align_sensor_to_vehicle(const Vector3f &position, const Quaternion &attitude);

    AP_VisualOdom &_frontend;                   // reference to frontend

    float _yaw_trim;                            // yaw angle trim (in radians) to align camera's yaw to ahrs/EKF's
    Quaternion _yaw_rotation;                   // earth-frame yaw rotation to align heading of sensor with vehicle.  use when _yaw_trim is non-zero
    Quaternion _att_rotation;                   // body-frame rotation corresponding to ORIENT parameter.  use when get_orientation != NONE
    Matrix3f _pos_rotation;                     // rotation to align position from sensor to earth frame.  use when _use_pos_rotation is true
    Vector3f _pos_correction;                   // position correction that should be added to position reported from sensor
    bool _use_att_rotation;                     // true if _att_rotation should be applied to sensor's attitude data
    bool _use_pos_rotation;                     // true if _pos_rotation should be applied to sensor's position data
    bool _align_camera = true;                  // true if camera should be aligned to AHRS/EKF
    bool _have_attitude;                        // true if we have received an attitude from the camera (used for arming checks)
    bool _error_orientation;                    // true if the orientation is not supported
    Quaternion _attitude_last;                  // last attitude received from camera (used for arming checks)
    uint32_t _last_update_ms;                   // system time of last update from sensor (used by health checks)
};
