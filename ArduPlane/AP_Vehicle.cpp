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
#include "Plane.h"

// set target location (for use by scripting)
bool Plane::set_target_location(const Location& target_loc)
{
    if (plane.control_mode != &plane.mode_guided) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    plane.guided_WP_loc = target_loc;
    // add home alt if needed
    if (plane.guided_WP_loc.relative_alt) {
        plane.guided_WP_loc.alt += plane.home.alt;
        plane.guided_WP_loc.relative_alt = 0;
    }
    plane.set_guided_WP();
    return true;
}

// set target location (for use by scripting)
bool Plane::get_target_location(Location& target_loc)
{
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::AUTO:
    case Mode::Number::LOITER:
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
        target_loc = next_WP_loc;
        return true;
        break;
    default:
        break;
    }
    return false;
}

AC_PID* Plane::get_AC_PID(AC_PID_TYPE type)
{
    AC_PID *ret = nullptr;

    if (plane.quadplane.enabled()) {
        switch(type) {
            case AC_PID_TYPE::RATE_ROLL:
                ret = &quadplane.attitude_control->get_rate_roll_pid();
                break;

            case AC_PID_TYPE::RATE_PITCH:
                ret = &quadplane.attitude_control->get_rate_pitch_pid();
                break;

            case AC_PID_TYPE::RATE_YAW:
                ret = &quadplane.attitude_control->get_rate_yaw_pid();
                break;

            default:
                break;
        }
    }

    return ret;
}
