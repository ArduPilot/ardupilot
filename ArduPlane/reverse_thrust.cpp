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
/*
  reverse thrust support functions
 */
#include "Plane.h"

/*
  see if reverse thrust should be allowed in the current flight state
 */
bool Plane::allow_reverse_thrust(void) const
{
    // check if we should allow reverse thrust
    bool allow = false;

    if (g.use_reverse_thrust == USE_REVERSE_THRUST_NEVER || !have_reverse_thrust()) {
        return false;
    }

    switch (control_mode->mode_number()) {
    case Mode::Number::AUTO:
        {
        uint16_t nav_cmd = mission.get_current_nav_cmd().id;

        // never allow reverse thrust during takeoff
        if (nav_cmd == MAV_CMD_NAV_TAKEOFF) {
            return false;
        }

        // always allow regardless of mission item
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_AUTO_ALWAYS);

        // landing
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_AUTO_LAND_APPROACH) &&
                (nav_cmd == MAV_CMD_NAV_LAND);

        // LOITER_TO_ALT
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_AUTO_LOITER_TO_ALT) &&
                (nav_cmd == MAV_CMD_NAV_LOITER_TO_ALT);

        // any Loiter (including LOITER_TO_ALT)
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_AUTO_LOITER_ALL) &&
                    (nav_cmd == MAV_CMD_NAV_LOITER_TIME ||
                     nav_cmd == MAV_CMD_NAV_LOITER_TO_ALT ||
                     nav_cmd == MAV_CMD_NAV_LOITER_TURNS ||
                     nav_cmd == MAV_CMD_NAV_LOITER_UNLIM);

        // waypoints
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_AUTO_WAYPOINT) &&
                    (nav_cmd == MAV_CMD_NAV_WAYPOINT ||
                     nav_cmd == MAV_CMD_NAV_SPLINE_WAYPOINT);

        // we are on a landing pattern
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_AUTO_LANDING_PATTERN) &&
                mission.get_in_landing_sequence_flag();
        }
        break;

    case Mode::Number::LOITER:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_LOITER);
        break;
    case Mode::Number::RTL:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_RTL);
        break;
    case Mode::Number::CIRCLE:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_CIRCLE);
        break;
    case Mode::Number::CRUISE:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_CRUISE);
        break;
    case Mode::Number::FLY_BY_WIRE_B:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_FBWB);
        break;
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_GUIDED);
        break;
    case Mode::Number::TAKEOFF:
        allow = false;
        break;
case Mode::Number::FLY_BY_WIRE_A:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_FBWA);
        break;
case Mode::Number::ACRO:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_ACRO);
        break;
case Mode::Number::STABILIZE:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_STABILIZE);
        break;
case Mode::Number::THERMAL:
        allow |= (g.use_reverse_thrust & USE_REVERSE_THRUST_THERMAL);
        break;
    default:
        // all other control_modes allow independent of mask(MANUAL)
        allow = true;
        break;
    }

    // cope with bitwise ops above
    return allow != false;
}

/*
  return true if we are configured to support reverse thrust
 */
bool Plane::have_reverse_thrust(void) const
{
    return aparm.throttle_min < 0;
}

/*
  return control in from the radio throttle channel.
 */
int16_t Plane::get_throttle_input(bool no_deadzone) const
{
    int16_t ret;
    if (no_deadzone) {
        ret = channel_throttle->get_control_in_zero_dz();
    } else {
        ret = channel_throttle->get_control_in();
    }
    if (reversed_throttle) {
        // RC option for reverse throttle has been set
        ret = -ret;
    }
    return ret;
}
