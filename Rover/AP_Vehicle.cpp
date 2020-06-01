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
#include "Rover.h"

// set target location (for use by scripting)
bool Rover::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    return control_mode->set_desired_location(target_loc);
}

// set target velocity (for use by scripting)
bool Rover::set_target_velocity_NED(const Vector3f& vel_ned)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    // convert vector length into speed
    const float target_speed_m = safe_sqrt(sq(vel_ned.x) + sq(vel_ned.y));

    // convert vector direction to target yaw
    const float target_yaw_cd = degrees(atan2f(vel_ned.y, vel_ned.x)) * 100.0f;

    // send target heading and speed
    mode_guided.set_desired_heading_and_speed(target_yaw_cd, target_speed_m);

    return true;
}