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

#include "AP_VisualOdom.h"

#if HAL_VISUALODOM_ENABLED

#include "AP_VisualOdom_Backend.h"
#include "AP_VisualOdom_MAV.h"
#include "AP_VisualOdom_IntelT265.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_VisualOdom::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Visual odometry camera connection type
    // @Description: Visual odometry camera connection type
    // @Values: 0:None,1:MAVLink,2:IntelT265,3:VOXL(ModalAI)
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 0, AP_VisualOdom, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _POS_X
    // @DisplayName: Visual odometry camera X position offset
    // @Description: X position of the camera in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Visual odometry camera Y position offset
    // @Description: Y position of the camera in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Visual odometry camera Z position offset
    // @Description: Z position of the camera in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_POS", 1, AP_VisualOdom, _pos_offset, 0.0f),

    // @Param: _ORIENT
    // @DisplayName: Visual odometery camera orientation
    // @Description: Visual odometery camera orientation
    // @Values: 0:Forward, 2:Right, 4:Back, 6:Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("_ORIENT", 2, AP_VisualOdom, _orientation, ROTATION_NONE),

    // @Param: _SCALE
    // @DisplayName: Visual odometry scaling factor
    // @Description: Visual odometry scaling factor applied to position estimates from sensor
    // @User: Advanced
    AP_GROUPINFO("_SCALE", 3, AP_VisualOdom, _pos_scale, 1.0f),

    // @Param: _DELAY_MS
    // @DisplayName: Visual odometry sensor delay
    // @Description: Visual odometry sensor delay relative to inertial measurements
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("_DELAY_MS", 4, AP_VisualOdom, _delay_ms, 10),

    // @Param: _VEL_M_NSE
    // @DisplayName: Visual odometry velocity measurement noise
    // @Description: Visual odometry velocity measurement noise in m/s
    // @Units: m/s
    // @Range: 0.05 5.0
    // @User: Advanced
    AP_GROUPINFO("_VEL_M_NSE", 5, AP_VisualOdom, _vel_noise, 0.1),

    // @Param: _POS_M_NSE
    // @DisplayName: Visual odometry position measurement noise 
    // @Description: Visual odometry position measurement noise minimum (meters). This value will be used if the sensor provides a lower noise value (or no noise value)
    // @Units: m
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("_POS_M_NSE", 6, AP_VisualOdom, _pos_noise, 0.2f),

    // @Param: _YAW_M_NSE
    // @DisplayName: Visual odometry yaw measurement noise
    // @Description: Visual odometry yaw measurement noise minimum (radians), This value will be used if the sensor provides a lower noise value (or no noise value)
    // @Units: rad
    // @Range: 0.05 1.0
    // @User: Advanced
    AP_GROUPINFO("_YAW_M_NSE", 7, AP_VisualOdom, _yaw_noise, 0.2f),

    AP_GROUPEND
};

AP_VisualOdom::AP_VisualOdom()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("must be singleton");
    }
#endif
    _singleton = this;
}

// detect and initialise any sensors
void AP_VisualOdom::init()
{
    // create backend
    switch (VisualOdom_Type(_type.get())) {
    case VisualOdom_Type::None:
        // do nothing
        break;
    case VisualOdom_Type::MAV:
        _driver = new AP_VisualOdom_MAV(*this);
        break;
    case VisualOdom_Type::IntelT265:
    case VisualOdom_Type::VOXL:
        _driver = new AP_VisualOdom_IntelT265(*this);
        break;
    }
}

// return true if sensor is enabled
bool AP_VisualOdom::enabled() const
{
    return ((_type != VisualOdom_Type::None));
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_VisualOdom::healthy() const
{
    if (!enabled()) {
        return false;
    }

    if (_driver == nullptr) {
        return false;
    }
    return _driver->healthy();
}

#if HAL_GCS_ENABLED
// consume vision_position_delta mavlink messages
void AP_VisualOdom::handle_vision_position_delta_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_vision_position_delta_msg(msg);
    }
}
#endif

// general purpose method to consume position estimate data and send to EKF
// distances in meters, roll, pitch and yaw are in radians
void AP_VisualOdom::handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, float posErr, float angErr, uint8_t reset_counter)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        // convert attitude to quaternion and call backend
        Quaternion attitude;
        attitude.from_euler(roll, pitch, yaw);
        _driver->handle_vision_position_estimate(remote_time_us, time_ms, x, y, z, attitude, posErr, angErr, reset_counter);
    }
}

// general purpose method to consume position estimate data and send to EKF
void AP_VisualOdom::handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_vision_position_estimate(remote_time_us, time_ms, x, y, z, attitude, posErr, angErr, reset_counter);
    }
}

void AP_VisualOdom::handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_vision_speed_estimate(remote_time_us, time_ms, vel, reset_counter);
    }
}

// request sensor's yaw be aligned with vehicle's AHRS/EKF attitude
void AP_VisualOdom::request_align_yaw_to_ahrs()
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->request_align_yaw_to_ahrs();
    }
}

// update position offsets to align to AHRS position.  Should only be called when this library is not being used as the position source
void AP_VisualOdom::align_position_to_ahrs(bool align_xy, bool align_z)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->align_position_to_ahrs(align_xy, align_z);
    }
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
bool AP_VisualOdom::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // exit immediately if not enabled
    if (!enabled()) {
        return true;
    }

    // check healthy
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "not healthy");
        return false;
    }

    // if no backend we must have failed to create because out of memory
    if (_driver == nullptr) {
        hal.util->snprintf(failure_msg, failure_msg_len, "out of memory");
        return false;
    }

    // call backend specific arming check
    return _driver->pre_arm_check(failure_msg, failure_msg_len);
}

// singleton instance
AP_VisualOdom *AP_VisualOdom::_singleton;

namespace AP {

AP_VisualOdom *visualodom()
{
    return AP_VisualOdom::get_singleton();
}

}

#endif
