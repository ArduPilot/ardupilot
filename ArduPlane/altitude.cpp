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

#include "Plane.h"

/*
  altitude handling routines. These cope with both barometric control
  and terrain following control
 */

/*
  adjust altitude target depending on mode
 */
void Plane::adjust_altitude_target()
{
    if (control_mode == FLY_BY_WIRE_B ||
        control_mode == CRUISE) {
        return;
    }
    if (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
        // in land final TECS uses TECS_LAND_SINK as a target sink
        // rate, and ignores the target altitude
        set_target_altitude_location(next_WP_loc);
    } else if (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH ||
            flight_stage == AP_SpdHgtControl::FLIGHT_LAND_PREFLARE) {
        setup_landing_glide_slope();
    } else if (nav_controller->reached_loiter_target()) {
        // once we reach a loiter target then lock to the final
        // altitude target
        set_target_altitude_location(next_WP_loc);
    } else if (target_altitude.offset_cm != 0 && 
               !location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) {
        // control climb/descent rate
        set_target_altitude_proportion(next_WP_loc, 1.0f-auto_state.wp_proportion);

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
void Plane::setup_glide_slope(void)
{
    // establish the distance we are travelling to the next waypoint,
    // for calculating out rate of change of altitude
    auto_state.wp_distance = get_distance(current_loc, next_WP_loc);
    auto_state.wp_proportion = location_path_proportion(current_loc, 
                                                        prev_WP_loc, next_WP_loc);
    SpdHgt_Controller->set_path_proportion(auto_state.wp_proportion);

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
        if (above_location_current(next_WP_loc)) {
            set_offset_altitude_location(next_WP_loc);
        } else {
            reset_offset_altitude();
        }
        break;

    case AUTO:
        // we only do glide slide handling in AUTO when above 20m or
        // when descending. The 20 meter threshold is arbitrary, and
        // is basically to prevent situations where we try to slowly
        // gain height at low altitudes, potentially hitting
        // obstacles.
        if (adjusted_relative_altitude_cm() > 2000 || above_location_current(next_WP_loc)) {
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
int32_t Plane::get_RTL_altitude()
{
    if (g.RTL_altitude_cm < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude_cm + home.alt;
}


/*
  return relative altitude in meters (relative to home)
 */
float Plane::relative_altitude(void)
{
    return (current_loc.alt - home.alt) * 0.01f;
}

/*
  return relative altitude in centimeters, absolute value
 */
int32_t Plane::relative_altitude_abs_cm(void)
{
    return labs(current_loc.alt - home.alt);
}


/*
  set the target altitude to the current altitude. This is used when 
  setting up for altitude hold, such as when releasing elevator in
  CRUISE mode.
 */
void Plane::set_target_altitude_current(void)
{
    // record altitude above sea level at the current time as our
    // target altitude
    target_altitude.amsl_cm = current_loc.alt;

    // reset any glide slope offset
    reset_offset_altitude();

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (g.terrain_follow && terrain.height_above_terrain(terrain_altitude, true)) {
        target_altitude.terrain_following = true;
        target_altitude.terrain_alt_cm = terrain_altitude*100;
    } else {
        // if terrain following is disabled, or we don't know our
        // terrain altitude when we set the altitude then don't
        // terrain follow
        target_altitude.terrain_following = false;        
    }
#endif
}

/*
  set the target altitude to the current altitude, with ALT_OFFSET adjustment
 */
void Plane::set_target_altitude_current_adjusted(void)
{
    set_target_altitude_current();

    // use adjusted_altitude_cm() to take account of ALTITUDE_OFFSET
    target_altitude.amsl_cm = adjusted_altitude_cm();
}

/*
  set target altitude based on a location structure
 */
void Plane::set_target_altitude_location(const Location &loc)
{
    target_altitude.amsl_cm = loc.alt;
    if (loc.flags.relative_alt) {
        target_altitude.amsl_cm += home.alt;
    }
#if AP_TERRAIN_AVAILABLE
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (loc.flags.terrain_alt && terrain.height_above_terrain(height, true)) {
        target_altitude.terrain_following = true;
        target_altitude.terrain_alt_cm = loc.alt;
        if (!loc.flags.relative_alt) {
            // it has home added, remove it
            target_altitude.terrain_alt_cm -= home.alt;
        }
    } else {
        target_altitude.terrain_following = false;
    }
#endif
}

/*
  return relative to home target altitude in centimeters. Used for
  altitude control libraries
 */
int32_t Plane::relative_target_altitude_cm(void)
{
#if AP_TERRAIN_AVAILABLE
    float relative_home_height;
    if (target_altitude.terrain_following && 
        terrain.height_relative_home_equivalent(target_altitude.terrain_alt_cm*0.01f,
                                                relative_home_height, true)) {
        // add lookahead adjustment the target altitude
        target_altitude.lookahead = lookahead_adjustment();
        relative_home_height += target_altitude.lookahead;

        // correct for rangefinder data
        relative_home_height += rangefinder_correction();

        // we are following terrain, and have terrain data for the
        // current location. Use it.
        return relative_home_height*100;
    }
#endif
    int32_t relative_alt = target_altitude.amsl_cm - home.alt;
    relative_alt += int32_t(g.alt_offset)*100;
    relative_alt += rangefinder_correction() * 100;
    return relative_alt;
}

/*
  change the current target altitude by an amount in centimeters. Used
  to cope with changes due to elevator in CRUISE or FBWB
 */
void Plane::change_target_altitude(int32_t change_cm)
{
    target_altitude.amsl_cm += change_cm;
#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following) {
        target_altitude.terrain_alt_cm += change_cm;
    }
#endif
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
void Plane::set_target_altitude_proportion(const Location &loc, float proportion)
{
    set_target_altitude_location(loc);
    proportion = constrain_float(proportion, 0.0f, 1.0f);
    change_target_altitude(-target_altitude.offset_cm*proportion);
    //rebuild the glide slope if we are above it and supposed to be climbing
    if(g.glide_slope_threshold > 0) {
        if(target_altitude.offset_cm > 0 && calc_altitude_error_cm() < -100 * g.glide_slope_threshold) {
            set_target_altitude_location(loc);
            set_offset_altitude_location(loc);
            change_target_altitude(-target_altitude.offset_cm*proportion);
            //adjust the new target offset altitude to reflect that we are partially already done
            if(proportion > 0.0f)
                target_altitude.offset_cm = ((float)target_altitude.offset_cm)/proportion;
        }
    }
}

/*
  constrain target altitude to be between two locations. Used to
  ensure we stay within two waypoints in altitude
 */
void Plane::constrain_target_altitude_location(const Location &loc1, const Location &loc2)
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
int32_t Plane::calc_altitude_error_cm(void)
{
#if AP_TERRAIN_AVAILABLE
    float terrain_height;
    if (target_altitude.terrain_following && 
        terrain.height_above_terrain(terrain_height, true)) {
        return target_altitude.lookahead*100 + target_altitude.terrain_alt_cm - (terrain_height*100);
    }
#endif
    return target_altitude.amsl_cm - adjusted_altitude_cm();
}

/*
  check for FBWB_min_altitude_cm violation
 */
void Plane::check_minimum_altitude(void)
{
    if (g.FBWB_min_altitude_cm == 0) {
        return;
    }

#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following) {
            // set our target terrain height to be at least the min set
            if (target_altitude.terrain_alt_cm < g.FBWB_min_altitude_cm) {
                target_altitude.terrain_alt_cm = g.FBWB_min_altitude_cm;
            }
            return;
    }
#endif

    if (target_altitude.amsl_cm < home.alt + g.FBWB_min_altitude_cm) {
        target_altitude.amsl_cm = home.alt + g.FBWB_min_altitude_cm;
    }
}

/*
  reset the altitude offset used for glide slopes
 */
void Plane::reset_offset_altitude(void)
{
    target_altitude.offset_cm = 0;
}


/*
  reset the altitude offset used for glide slopes, based on difference
  between altitude at a destination and current altitude. If
  destination is above the current altitude then the result is
  positive.
 */
void Plane::set_offset_altitude_location(const Location &loc)
{
    target_altitude.offset_cm = loc.alt - current_loc.alt;

#if AP_TERRAIN_AVAILABLE
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (loc.flags.terrain_alt && 
        target_altitude.terrain_following &&
        terrain.height_above_terrain(height, true)) {
        target_altitude.offset_cm = target_altitude.terrain_alt_cm - (height * 100);
    }
#endif

    if (flight_stage != AP_SpdHgtControl::FLIGHT_LAND_PREFLARE &&
        flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH &&
        flight_stage != AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
        // if we are within GLIDE_SLOPE_MIN meters of the target altitude
        // then reset the offset to not use a glide slope. This allows for
        // more accurate flight of missions where the aircraft may lose or
        // gain a bit of altitude near waypoint turn points due to local
        // terrain changes
        if (g.glide_slope_min <= 0 ||
            labs(target_altitude.offset_cm)*0.01f < g.glide_slope_min) {
            target_altitude.offset_cm = 0;
        }
    }
}

/*
  return true if current_loc is above loc. Used for glide slope
  calculations.

  "above" is simple if we are not terrain following, as it just means
  the pressure altitude of one is above the other.

  When in terrain following mode "above" means the over-the-terrain
  current altitude is above the over-the-terrain alt of loc. It is
  quite possible for current_loc to be "above" loc when it is at a
  lower pressure altitude, if current_loc is in a low part of the
  terrain
 */
bool Plane::above_location_current(const Location &loc)
{
#if AP_TERRAIN_AVAILABLE
    float terrain_alt;
    if (loc.flags.terrain_alt && 
        terrain.height_above_terrain(terrain_alt, true)) {
        float loc_alt = loc.alt*0.01f;
        if (!loc.flags.relative_alt) {
            loc_alt -= home.alt*0.01f;
        }
        return terrain_alt > loc_alt;
    }
#endif

    float loc_alt_cm = loc.alt;
    if (loc.flags.relative_alt) {
        loc_alt_cm += home.alt;
    }
    return current_loc.alt > loc_alt_cm;
}

/*
  modify a destination to be setup for terrain following if
  TERRAIN_FOLLOW is enabled
 */
void Plane::setup_terrain_target_alt(Location &loc)
{
#if AP_TERRAIN_AVAILABLE
    if (g.terrain_follow) {
        loc.flags.terrain_alt = true;
    }
#endif
}

/*
  return current_loc.alt adjusted for ALT_OFFSET
  This is useful during long flights to account for barometer changes
  from the GCS, or to adjust the flying height of a long mission
 */
int32_t Plane::adjusted_altitude_cm(void)
{
    return current_loc.alt - (g.alt_offset*100);
}

/*
  return home-relative altitude adjusted for ALT_OFFSET This is useful
  during long flights to account for barometer changes from the GCS,
  or to adjust the flying height of a long mission
 */
int32_t Plane::adjusted_relative_altitude_cm(void)
{
    return adjusted_altitude_cm() - home.alt;
}

/*
  return the height in meters above the next_WP_loc altitude
 */
float Plane::height_above_target(void)
{
    float target_alt = next_WP_loc.alt*0.01;
    if (!next_WP_loc.flags.relative_alt) {
        target_alt -= ahrs.get_home().alt*0.01f;
    }

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (next_WP_loc.flags.terrain_alt && 
        terrain.height_above_terrain(terrain_altitude, true)) {
        return terrain_altitude - target_alt;
    }
#endif

    return (adjusted_altitude_cm()*0.01f - ahrs.get_home().alt*0.01f) - target_alt;
}

/*
  work out target altitude adjustment from terrain lookahead
 */
float Plane::lookahead_adjustment(void)
{
#if AP_TERRAIN_AVAILABLE
    int32_t bearing_cd;
    int16_t distance;
    // work out distance and bearing to target
    if (control_mode == FLY_BY_WIRE_B) {
        // there is no target waypoint in FBWB, so use yaw as an approximation
        bearing_cd = ahrs.yaw_sensor;
        distance = g.terrain_lookahead;
    } else if (!nav_controller->reached_loiter_target()) {
        bearing_cd = nav_controller->target_bearing_cd();
        distance = constrain_float(auto_state.wp_distance, 0, g.terrain_lookahead);
    } else {
        // no lookahead when loitering
        bearing_cd = 0;
        distance = 0;
    }
    if (distance <= 0) {
        // no lookahead
        return 0;
    }

    
    float groundspeed = ahrs.groundspeed();
    if (groundspeed < 1) {
        // we're not moving
        return 0;
    }
    // we need to know the climb ratio. We use 50% of the maximum
    // climb rate so we are not constantly at 100% throttle and to
    // give a bit more margin on terrain
    float climb_ratio = 0.5f * SpdHgt_Controller->get_max_climbrate() / groundspeed;

    if (climb_ratio <= 0) {
        // lookahead makes no sense for negative climb rates
        return 0;
    }
    
    // ask the terrain code for the lookahead altitude change
    float lookahead = terrain.lookahead(bearing_cd*0.01f, distance, climb_ratio);
    
    if (target_altitude.offset_cm < 0) {
        // we are heading down to the waypoint, so we don't need to
        // climb as much
        lookahead += target_altitude.offset_cm*0.01f;
    }

    // constrain lookahead to a reasonable limit
    return constrain_float(lookahead, 0, 1000.0f);
#else
    return 0;
#endif
}


/*
  correct target altitude using rangefinder data. Returns offset in
  meters to correct target altitude. A positive number means we need
  to ask the speed/height controller to fly higher
 */
float Plane::rangefinder_correction(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    if (millis() - rangefinder_state.last_correction_time_ms > 5000) {
        // we haven't had any rangefinder data for 5s - don't use it
        return 0;
    }

    // for now we only support the rangefinder for landing 
    bool using_rangefinder = (g.rangefinder_landing &&
                              control_mode == AUTO && 
                              (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH ||
                               flight_stage == AP_SpdHgtControl::FLIGHT_LAND_PREFLARE ||
                               flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL));
    if (!using_rangefinder) {
        return 0;
    }

    return rangefinder_state.correction;
#else
    return 0;
#endif
}

#if RANGEFINDER_ENABLED == ENABLED
/*
  update the offset between rangefinder height and terrain height
 */
void Plane::rangefinder_height_update(void)
{
    float distance = rangefinder.distance_cm()*0.01f;
    float height_estimate = 0;
    if ((rangefinder.status() == RangeFinder::RangeFinder_Good) && home_is_set != HOME_UNSET) {
        if (!rangefinder_state.have_initial_reading) {
            rangefinder_state.have_initial_reading = true;
            rangefinder_state.initial_range = distance;
        }
        // correct the range for attitude (multiply by DCM.c.z, which
        // is cos(roll)*cos(pitch))
        height_estimate = distance * ahrs.get_rotation_body_to_ned().c.z;

        // we consider ourselves to be fully in range when we have 10
        // good samples (0.2s) that are different by 5% of the maximum
        // range from the initial range we see. The 5% change is to
        // catch Lidars that are giving a constant range, either due
        // to misconfiguration or a faulty sensor
        if (rangefinder_state.in_range_count < 10) {
            if (fabsf(rangefinder_state.initial_range - distance) > 0.05f * rangefinder.max_distance_cm()*0.01f) {
                rangefinder_state.in_range_count++;
            }
        } else {
            rangefinder_state.in_range = true;
            if (!rangefinder_state.in_use &&
                (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH ||
                flight_stage == AP_SpdHgtControl::FLIGHT_LAND_PREFLARE ||
                flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) &&
                g.rangefinder_landing) {
                rangefinder_state.in_use = true;
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Rangefinder engaged at %.2fm", (double)height_estimate);
            }
        }
    } else {
        rangefinder_state.in_range_count = 0;
        rangefinder_state.in_range = false;
    }

    if (rangefinder_state.in_range) {
        // base correction is the difference between baro altitude and
        // rangefinder estimate
        float correction = relative_altitude() - height_estimate;

#if AP_TERRAIN_AVAILABLE
        // if we are terrain following then correction is based on terrain data
        float terrain_altitude;
        if ((target_altitude.terrain_following || g.terrain_follow) && 
            terrain.height_above_terrain(terrain_altitude, true)) {
            correction = terrain_altitude - height_estimate;
        }
#endif    

        // remember the last correction. Use a low pass filter unless
        // the old data is more than 5 seconds old
        if (millis() - rangefinder_state.last_correction_time_ms > 5000) {
            rangefinder_state.correction = correction;
            rangefinder_state.initial_correction = correction;
        } else {
            rangefinder_state.correction = 0.8f*rangefinder_state.correction + 0.2f*correction;
            if (fabsf(rangefinder_state.correction - rangefinder_state.initial_correction) > 30) {
                // the correction has changed by more than 30m, reset use of Lidar. We may have a bad lidar
                if (rangefinder_state.in_use) {
                    gcs_send_text_fmt(MAV_SEVERITY_INFO, "Rangefinder disengaged at %.2fm", (double)height_estimate);
                }
                memset(&rangefinder_state, 0, sizeof(rangefinder_state));
            }
        }
        rangefinder_state.last_correction_time_ms = millis();    
    }
}
#endif
