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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "WheelEncoder_SITL_Quadrature.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_WheelEncoder_SITL_Quadrature::update(void)
{
    const auto *_sitl = AP::sitl();

    // earth frame velocity of vehicle in vector form
    const Vector2f ef_velocity(_sitl->state.speedN, _sitl->state.speedE);
    // store current heading
    const double current_heading = _sitl->state.yawDeg;

    // transform ef_velocity vector to current_heading frame
    // helps to calculate direction of movement
    const Vector2f vf_velocity(ef_velocity.x*cos(radians(current_heading)) + ef_velocity.y*sin(radians(current_heading)),
                               -ef_velocity.x*sin(radians(current_heading)) + ef_velocity.y*cos(radians(current_heading)));

    // calculate dt
    const uint32_t time_now = AP_HAL::millis();
    const double dt = (time_now - _state.last_reading_ms)*0.001f;
    if (is_zero(dt)) { // sanity check
        return;
    }

    // get current speed and turn rate
    const double turn_rate = radians(_sitl->state.yawRate);
    double speed = norm(vf_velocity.x, vf_velocity.y);

    // assign direction to speed value
    if (vf_velocity.x < 0.0f) {
        speed *= -1;
    }

    // distance from center of wheel axis to each wheel
    const double half_wheelbase = ( fabsf(_frontend.get_pos_offset(0).y) + fabsf(_frontend.get_pos_offset(1).y) )/2.0f;
    if (is_zero(half_wheelbase)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WheelEncoder: wheel offset not set!");
    }

    if (_state.instance == 0) { 
        speed = speed - turn_rate * half_wheelbase; // for left wheel
    } else if (_state.instance == 1) {
        speed = speed + turn_rate * half_wheelbase; // for right wheel
    } else {
        AP_HAL::panic("Invalid wheel encoder instance");
        return; // invalid instance
    }

    const double radius = _frontend.get_wheel_radius(_state.instance);
    if (is_zero(radius)) { // avoid divide by zero
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WheelEncoder: wheel radius not set!");
        return; 
    }

    // calculate angle turned and corresponding encoder ticks from wheel angular rate
    const double angle_turned = ( speed/radius ) * dt;
    const int32_t ticks =  static_cast<int>( ( angle_turned/(2 * M_PI) ) * _frontend.get_counts_per_revolution(_state.instance) );

    // update distance count
    _distance_count += ticks;
    // update total count of encoder ticks
    _total_count += abs(ticks);

    // update previous state to current
    copy_state_to_frontend(_distance_count, _total_count, 0, time_now);
}

#endif
