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

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#ifndef HAL_VISUALODOM_ENABLED
#define HAL_VISUALODOM_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_VISUALODOM_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

class AP_VisualOdom_Backend;

#define AP_VISUALODOM_TIMEOUT_MS 300

class AP_VisualOdom
{
public:

    AP_VisualOdom();

    // get singleton instance
    static AP_VisualOdom *get_singleton() {
        return _singleton;
    }

    // external position backend types (used by _TYPE parameter)
    enum AP_VisualOdom_Type {
        AP_VisualOdom_Type_None         = 0,
        AP_VisualOdom_Type_MAV          = 1,
        AP_VisualOdom_Type_IntelT265    = 2
    };

    // detect and initialise any sensors
    void init();

    // return true if sensor is enabled
    bool enabled() const;

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() const;

    // get user defined orientation
    enum Rotation get_orientation() const { return (enum Rotation)_orientation.get(); }

    // get user defined scaling applied to position estimates
    float get_pos_scale() const { return _pos_scale; }

    // return a 3D vector defining the position offset of the camera in meters relative to the body frame origin
    const Vector3f &get_pos_offset(void) const { return _pos_offset; }

    // return the sensor delay in milliseconds (see _DELAY_MS parameter)
    uint16_t get_delay_ms() const { return MAX(0, _delay_ms); }

    // consume vision_position_delta mavlink messages
    void handle_vision_position_delta_msg(const mavlink_message_t &msg);

    // general purpose methods to consume position estimate data and send to EKF
    // distances in meters, roll, pitch and yaw are in radians
    void handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, uint8_t reset_counter);
    void handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, uint8_t reset_counter);

    // calibrate camera attitude to align with vehicle's AHRS/EKF attitude
    void align_sensor_to_vehicle();

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_VisualOdom *_singleton;

    // parameters
    AP_Int8 _type;              // sensor type
    AP_Vector3f _pos_offset;    // position offset of the camera in the body frame
    AP_Int8 _orientation;       // camera orientation on vehicle frame
    AP_Float _pos_scale;        // position scale factor applied to sensor values
    AP_Int16 _delay_ms;         // average delay relative to inertial measurements

    // reference to backends
    AP_VisualOdom_Backend *_driver;
};

namespace AP {
    AP_VisualOdom *visualodom();
};

#endif // HAL_VISUALODOM_ENABLED
