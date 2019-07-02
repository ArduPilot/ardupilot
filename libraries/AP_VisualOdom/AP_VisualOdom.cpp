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
#include "AP_VisualOdom_Backend.h"
#include "AP_VisualOdom_MAV.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_VisualOdom::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Visual odometry camera connection type
    // @Description: Visual odometry camera connection type
    // @Values: 0:None,1:MAV
    // @User: Advanced
    AP_GROUPINFO("_TYPE", 0, AP_VisualOdom, _type, 0),

    // @Param: _POS_X
    // @DisplayName: Visual odometry camera X position offset
    // @Description: X position of the camera in body frame. Positive X is forward of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Visual odometry camera Y position offset
    // @Description: Y position of the camera in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Visual odometry camera Z position offset
    // @Description: Z position of the camera in body frame. Positive Z is down from the origin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_POS", 1, AP_VisualOdom, _pos_offset, 0.0f),

    // @Param: _ORIENT
    // @DisplayName: Visual odometery camera orientation
    // @Description: Visual odometery camera orientation
    // @Values: 0:Forward, 2:Right, 4:Back, 6:Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("_ORIENT", 2, AP_VisualOdom, _orientation, ROTATION_NONE),

    AP_GROUPEND
};

AP_VisualOdom::AP_VisualOdom()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("VisualOdom must be singleton");
    }
#endif
    _singleton = this;
}

// detect and initialise any sensors
void AP_VisualOdom::init()
{
    // create backend
    if (_type == AP_VisualOdom_Type_MAV) {
        _driver = new AP_VisualOdom_MAV(*this);
    }
}

// return true if sensor is enabled
bool AP_VisualOdom::enabled() const
{
    return ((_type != AP_VisualOdom_Type_None) && (_driver != nullptr));
}

// update visual odometry sensor
void AP_VisualOdom::update()
{
    if (!enabled()) {
        return;
    }

    // check for updates
    if (_state.last_processed_sensor_update_ms == _state.last_sensor_update_ms) {
        // This reading has already been processed
        return;
    }
    _state.last_processed_sensor_update_ms = _state.last_sensor_update_ms;

    const float time_delta_sec = get_time_delta_usec() / 1000000.0f;

    AP::ahrs_navekf().writeBodyFrameOdom(get_confidence(),
                                         get_position_delta(),
                                         get_angle_delta(),
                                         time_delta_sec,
                                         get_last_update_ms(),
                                         get_pos_offset());
    // log sensor data
    AP::logger().Write_VisualOdom(time_delta_sec,
                                  get_angle_delta(),
                                  get_position_delta(),
                                  get_confidence());
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_VisualOdom::healthy() const
{
    if (!enabled()) {
        return false;
    }

    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _state.last_sensor_update_ms) < AP_VISUALODOM_TIMEOUT_MS);
}

// consume VISION_POSITION_DELTA MAVLink message
void AP_VisualOdom::handle_msg(mavlink_message_t *msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_msg(msg);
    }
}

// singleton instance
AP_VisualOdom *AP_VisualOdom::_singleton;

namespace AP {

AP_VisualOdom *visualodom()
{
    return AP_VisualOdom::get_singleton();
}

}
