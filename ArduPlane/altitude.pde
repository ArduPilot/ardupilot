// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  altitude handling routines
 */

/*
  adjust altitude target depending on mode
 */
static void adjust_altitude_target()
{
    if (control_mode == FLY_BY_WIRE_B ||
        control_mode == CRUISE) {
        return;
    }
    if (nav_controller->reached_loiter_target() || (wp_distance <= 30) || (wp_totalDistance<=30)) {
        // once we reach a loiter target then lock to the final
        // altitude target
        set_target_altitude_location(next_WP_loc);
    } else if (target_altitude.offset_cm != 0) {
        // control climb/descent rate
        set_target_altitude_proportion(next_WP_loc, (float)(wp_distance-30) / (float)(wp_totalDistance-30));

        // stay within the range of the start and end locations in altitude
        constrain_target_altitude_location(next_WP_loc, prev_WP_loc);
    } else if (mission.get_current_do_cmd().id != MAV_CMD_CONDITION_CHANGE_ALT) {
        set_target_altitude_location(next_WP_loc);
    }

    altitude_error_cm = calc_altitude_error_cm();
}

/*
  setup for a gradual glide slope to the next waypoint, if appropriate
 */
static void setup_glide_slope(void)
{
    // establish the distance we are travelling to the next waypoint,
    // for calculating out rate of change of altitude
    wp_totalDistance        = get_distance(current_loc, next_WP_loc);
    wp_distance             = wp_totalDistance;

    /*
      work out if we will gradually change altitude, or try to get to
      the new altitude as quickly as possible.
     */
    switch (control_mode) {
    case RTL:
    case GUIDED:
        /* glide down slowly if above target altitude, but ascend more
           rapidly if below it. See
           https://github.com/diydrones/ardupilot/issues/39
        */
        if (above_location(next_WP_loc)) {
            set_offset_altitude_location(next_WP_loc);
        } else {
            reset_offset_altitude();
        }
        break;

    case AUTO:
        // we only do glide slide handling in AUTO when above 40m or
        // when descending. The 40 meter threshold is arbitrary, and
        // is basically to prevent situations where we try to slowly
        // gain height at low altitudes, potentially hitting
        // obstacles.
        if (relative_altitude() > 40 || !above_location(next_WP_loc)) {
            set_offset_altitude_location(next_WP_loc);
        } else {
            reset_offset_altitude();
        }
        break;
    default:
        reset_offset_altitude();
        break;
    }
}

/*
  return RTL altitude as AMSL altitude
 */
static int32_t get_RTL_altitude()
{
    if (g.RTL_altitude_cm < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude_cm + home.alt;
}


/*
  return relative altitude in meters (relative to home)
 */
static float relative_altitude(void)
{
    return (current_loc.alt - home.alt) * 0.01f;
}

/*
  return relative altitude in centimeters, absolute value
 */
static int32_t relative_altitude_abs_cm(void)
{
    return labs(current_loc.alt - home.alt);
}


/*
  set the target altitude to the current altitude. This is used when 
  setting up for altitude hold, such as when releasing elevator in
  CRUISE mode.
 */
static void set_target_altitude_current(void)
{
    target_altitude.amsl_cm = current_loc.alt;
}

/*
  set the target altitude to the current altitude, with ALT_OFFSET adjustment
 */
static void set_target_altitude_current_adjusted(void)
{
    target_altitude.amsl_cm = adjusted_altitude_cm();
}

/*
  set target altitude based on a location structure
 */
static void set_target_altitude_location(const Location &loc)
{
    target_altitude.amsl_cm = loc.alt;
}

/*
  return relative to home target altitude in centimeters. Used for
  altitude control libraries
 */
static int32_t relative_target_altitude_cm(void)
{
    return target_altitude.amsl_cm - home.alt + (int32_t(g.alt_offset)*100);
}

/*
  change the current target altitude by an amount in centimeters. Used
  to cope with changes due to elevator in CRUISE or FBWB
 */
static void change_target_altitude(int32_t change_cm)
{
    target_altitude.amsl_cm += change_cm;
}

/*
  change target altitude by a proportion of the target altitude offset
  (difference in height to next WP from previous WP). proportion
  should be between 0 and 1. 

  When proportion is zero we have reached the destination. When
  proportion is 1 we are at the starting waypoint.

  Note that target_altitude is setup initially based on the
  destination waypoint
 */
static void set_target_altitude_proportion(const Location &loc, float proportion)
{
    set_target_altitude_location(loc);
    proportion = constrain_float(proportion, 0.0f, 1.0f);
    change_target_altitude(-target_altitude.offset_cm*proportion);
}

/*
  constrain target altitude to be between two locations. Used to
  ensure we stay within two waypoints in altitude
 */
static void constrain_target_altitude_location(const Location &loc1, const Location &loc2)
{
    if (loc1.alt > loc2.alt) {
        target_altitude.amsl_cm = constrain_int32(target_altitude.amsl_cm, loc2.alt, loc1.alt);
    } else {
        target_altitude.amsl_cm = constrain_int32(target_altitude.amsl_cm, loc1.alt, loc2.alt);
    }
}

/*
  return error between target altitude and current altitude
 */
static int32_t calc_altitude_error_cm(void)
{
    return target_altitude.amsl_cm - adjusted_altitude_cm();
}

/*
  check for FBWB_min_altitude_cm violation
 */
static void check_minimum_altitude(void)
{
    if (g.FBWB_min_altitude_cm != 0 && target_altitude.amsl_cm < home.alt + g.FBWB_min_altitude_cm) {
        target_altitude.amsl_cm = home.alt + g.FBWB_min_altitude_cm;
    }
}

/*
  reset the altitude offset used for glide slopes
 */
static void reset_offset_altitude(void)
{
    target_altitude.offset_cm = 0;
}

/*
  reset the altitude offset used for glide slopes, based on difference
  between altitude at a destination and current altitude. If
  destination is above the current altitude then the result is
  positive.
 */
static void set_offset_altitude_location(const Location &loc)
{
    target_altitude.offset_cm = loc.alt - current_loc.alt;
}

/*
  are we above the altitude given by a location?
 */
static bool above_location(const Location &loc)
{
    return current_loc.alt > loc.alt;
}
