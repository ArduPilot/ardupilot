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

// Check USE_REV_THRUST paramater for the given reverse thrust option
bool Plane::reverse_thrust_enabled(UseReverseThrust use_reverse_thrust_option) const
{
    return (g.use_reverse_thrust.get() & int32_t(use_reverse_thrust_option)) != 0;
}

/*
  see if reverse thrust should be allowed in the current flight state
 */
bool Plane::allow_reverse_thrust(void) const
{
    // Reverse thrust is not configured, so it should never be allowed
    if (!have_reverse_thrust()) {
        return false;
    }

    // check if we should allow reverse thrust
    bool allow = false;

    switch (control_mode->mode_number()) {
    case Mode::Number::AUTO:
        {
        uint16_t nav_cmd = mission.get_current_nav_cmd().id;

        // never allow reverse thrust during takeoff
        if (nav_cmd == MAV_CMD_NAV_TAKEOFF) {
            return false;
        }

        // always allow regardless of mission item
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_ALWAYS);

        // landing
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_LAND_APPROACH) &&
                (nav_cmd == MAV_CMD_NAV_LAND);

        // LOITER_TO_ALT
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_LOITER_TO_ALT) &&
                (nav_cmd == MAV_CMD_NAV_LOITER_TO_ALT);

        // any Loiter (including LOITER_TO_ALT)
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_LOITER_ALL) &&
                    (nav_cmd == MAV_CMD_NAV_LOITER_TIME ||
                     nav_cmd == MAV_CMD_NAV_LOITER_TO_ALT ||
                     nav_cmd == MAV_CMD_NAV_LOITER_TURNS ||
                     nav_cmd == MAV_CMD_NAV_LOITER_UNLIM);

        // waypoints
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_WAYPOINT) &&
                    (nav_cmd == MAV_CMD_NAV_WAYPOINT ||
                     nav_cmd == MAV_CMD_NAV_SPLINE_WAYPOINT);

        // we are on a landing pattern
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_LANDING_PATTERN) &&
                mission.get_in_landing_sequence_flag();
        }
        break;

    case Mode::Number::LOITER:
        allow |= reverse_thrust_enabled(UseReverseThrust::LOITER);
        break;
    case Mode::Number::RTL:
        allow |= reverse_thrust_enabled(UseReverseThrust::RTL);
        break;
    case Mode::Number::CIRCLE:
        allow |= reverse_thrust_enabled(UseReverseThrust::CIRCLE);
        break;
    case Mode::Number::CRUISE:
        allow |= reverse_thrust_enabled(UseReverseThrust::CRUISE);
        break;
    case Mode::Number::FLY_BY_WIRE_B:
        allow |= reverse_thrust_enabled(UseReverseThrust::FBWB);
        break;
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
        allow |= reverse_thrust_enabled(UseReverseThrust::GUIDED);
        break;
    case Mode::Number::TAKEOFF:
        allow = false;
        break;
    case Mode::Number::FLY_BY_WIRE_A:
        allow |= reverse_thrust_enabled(UseReverseThrust::FBWA);
        break;
    case Mode::Number::ACRO:
        allow |= reverse_thrust_enabled(UseReverseThrust::ACRO);
        break;
    case Mode::Number::STABILIZE:
        allow |= reverse_thrust_enabled(UseReverseThrust::STABILIZE);
        break;
    case Mode::Number::THERMAL:
        allow |= reverse_thrust_enabled(UseReverseThrust::THERMAL);
        break;
    default:
        // all other control_modes allow independent of mask(MANUAL)
        allow = true;
        break;
    }

    return allow;
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
float Plane::get_throttle_input(bool no_deadzone) const
{
    if (!rc().has_valid_input()) {
        // Return 0 if there is no valid input
        return 0.0;
    }
    float ret;
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

/*
  return control in from the radio throttle channel with curve giving mid-stick equal to TRIM_THROTTLE.
 */
float Plane::get_adjusted_throttle_input(bool no_deadzone) const
{
    if (!rc().has_valid_input()) {
        // Return 0 if there is no valid input
        return 0.0;
    }
    if ((plane.channel_throttle->get_type() != RC_Channel::ControlType::RANGE) ||
        (flight_option_enabled(FlightOptions::CENTER_THROTTLE_TRIM)) == 0) {
       return  get_throttle_input(no_deadzone);
    }
    float ret = channel_throttle->get_range() * throttle_curve(aparm.throttle_cruise * 0.01, 0, 0.5 + 0.5*channel_throttle->norm_input());
    if (reversed_throttle) {
        // RC option for reverse throttle has been set
        return -ret;
    }
    return ret;
}
