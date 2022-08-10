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
#include <AP_AltitudePlanner/AP_AltitudePlanner.h>

/*
  altitude handling routines. These cope with both barometric control
  and terrain following control
 */

/*
  adjust altitude target depending on mode
 */
void Plane::adjust_altitude_target()
{
#if AP_TERRAIN_AVAILABLE
    // Let the altitude planner know if terrain is disabled.
    altitudePlanner.set_terrain_enabled(terrain_enabled_in_current_mode() && !terrain_disabled());
#endif

    // Set the mission alt offset.
    float alt_offset = mission_alt_offset();
    altitudePlanner.set_mission_alt_offset(alt_offset);
    
    Location target_location;

    if (control_mode == &mode_fbwb ||
        control_mode == &mode_cruise) {
        return;
    }
    if ((control_mode == &mode_loiter) && plane.stick_mixing_enabled() && (plane.g2.flight_options & FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
       return;
    }
#if OFFBOARD_GUIDED == ENABLED
    if (control_mode == &mode_guided && ((guided_state.target_alt_time_ms != 0) || guided_state.target_alt > -0.001 )) { // target_alt now defaults to -1, and _time_ms defaults to zero.
        // offboard altitude demanded
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - guided_state.target_alt_time_ms);
        guided_state.target_alt_time_ms = now;
        // determine delta accurately as a float
        float delta_amt_f = delta * guided_state.target_alt_accel;
        // then scale x100 to match last_target_alt and convert to a signed int32_t as it may be negative
        int32_t delta_amt_i = (int32_t)(100.0 * delta_amt_f); 
        Location temp {};
        temp.alt = guided_state.last_target_alt + delta_amt_i; // ...to avoid floats here, 
        if (is_positive(guided_state.target_alt_accel)) {
            temp.alt = MIN(guided_state.target_alt, temp.alt);
        } else {
            temp.alt = MAX(guided_state.target_alt, temp.alt);
        }
        guided_state.last_target_alt = temp.alt;
        altitudePlanner.set_target_altitude_location(temp);
    } else 
#endif // OFFBOARD_GUIDED == ENABLED
      if (control_mode->update_target_altitude()) {
          // handled in mode specific code
    } else if (landing.is_flaring()) {
        // during a landing flare, use TECS_LAND_SINK as a target sink
        // rate, and ignores the target altitude
        altitudePlanner.set_target_altitude_location(next_WP_loc);
    } else if (landing.is_on_approach()) {
        landing.setup_landing_glide_slope(prev_WP_loc, next_WP_loc, current_loc);
        landing.adjust_landing_slope_for_rangefinder_bump(rangefinder_state, prev_WP_loc, next_WP_loc, current_loc, auto_state.wp_distance);
    } else if (landing.get_target_altitude_location(target_location)) {
       altitudePlanner.set_target_altitude_location(target_location);
#if HAL_SOARING_ENABLED
    } else if (g2.soaring_controller.is_active() && g2.soaring_controller.get_throttle_suppressed()) {
       // Reset target alt to current alt, to prevent large altitude errors when gliding.
       altitudePlanner.set_target_altitude_location(current_loc);
       altitudePlanner.reset_offset_altitude();
#endif
    } else if (reached_loiter_target()) {
        // once we reach a loiter target then lock to the final
        // altitude target
        altitudePlanner.set_target_altitude_location(next_WP_loc);
    } else if (altitudePlanner.get_target_offset_cm() != 0 &&
               !current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        // control climb/descent rate
        bool suppress_glideslope = flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND;
        altitudePlanner.set_target_altitude_proportion(prev_WP_loc, next_WP_loc, 1.0f-auto_state.wp_proportion, suppress_glideslope);

        // stay within the range of the start and end locations in altitude
        altitudePlanner.constrain_target_altitude_location(next_WP_loc, prev_WP_loc);
    } else {
        altitudePlanner.set_target_altitude_location(next_WP_loc);
    }

    altitude_error_cm = altitudePlanner.calc_altitude_error_cm();
}

/*
  setup for a gradual glide slope to the next waypoint, if appropriate
 */
void Plane::setup_glide_slope(void)
{
    // establish the distance we are travelling to the next waypoint,
    // for calculating out rate of change of altitude
    auto_state.wp_distance = current_loc.get_distance(next_WP_loc);
    auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    TECS_controller.set_path_proportion(auto_state.wp_proportion);
    update_flight_stage();

    /*
      work out if we will gradually change altitude, or try to get to
      the new altitude as quickly as possible.
     */
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
        /* glide down slowly if above target altitude, but ascend more
           rapidly if below it. See
           https://github.com/ArduPilot/ardupilot/issues/39
        */
        altitudePlanner.setup_glide_slope(prev_WP_loc, next_WP_loc, true, adjusted_relative_altitude_cm(), flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND);
        break;

    case Mode::Number::AUTO:
        // we only do glide slide handling in AUTO when above 20m or
        // when descending. The 20 meter threshold is arbitrary, and
        // is basically to prevent situations where we try to slowly
        // gain height at low altitudes, potentially hitting
        // obstacles.
        altitudePlanner.setup_glide_slope(prev_WP_loc, next_WP_loc, false, adjusted_relative_altitude_cm(), flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND);
        break;
    default:
        altitudePlanner.reset_offset_altitude();
        break;
    }
}


void AP_AltitudePlanner::setup_glide_slope(const Location &start_loc, const Location &target_loc, bool apply_gradual_climb, int32_t start_adj_rel_alt, bool force_glideslope)
{
    if (apply_gradual_climb) {
        // RTL, AVOID_ADSB or GUIDED modes
        if (above_location(start_loc, target_loc)) {
            set_offset_altitude_location(start_loc, target_loc, force_glideslope);
        } else {
            reset_offset_altitude();
        }
    } else {
        // AUTO mode
        if (start_adj_rel_alt > 2000 || above_location(start_loc, target_loc)) {
            set_offset_altitude_location(start_loc, target_loc, force_glideslope);
        } else {
            reset_offset_altitude();
        }
    }
}

/*
  return RTL altitude as AMSL cm
 */
int32_t Plane::get_RTL_altitude_cm() const
{
    if (g.RTL_altitude_cm < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude_cm + home.alt;
}

/*
  return relative altitude in meters (relative to terrain, if available,
  or home otherwise)
 */
float Plane::relative_ground_altitude(bool use_rangefinder_if_available)
{
   if (use_rangefinder_if_available && rangefinder_state.in_range) {
        return rangefinder_state.height_estimate;
   }

#if HAL_QUADPLANE_ENABLED
   if (use_rangefinder_if_available && quadplane.in_vtol_land_final() &&
       rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::OutOfRangeLow) {
       // a special case for quadplane landing when rangefinder goes
       // below minimum. Consider our height above ground to be zero
       return 0;
   }
#endif

#if AP_TERRAIN_AVAILABLE
    float altitude;
    if (altitudePlanner.is_terrain_following() &&
        terrain.status() == AP_Terrain::TerrainStatusOK &&
        terrain.height_above_terrain(altitude, true)) {
        return altitude;
    }
#endif

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_descent() &&
        !quadplane.option_is_set(QuadPlane::OPTION::MISSION_LAND_FW_APPROACH)) {
        // when doing a VTOL landing we can use the waypoint height as
        // ground height. We can't do this if using the
        // LAND_FW_APPROACH as that uses the wp height as the approach
        // height
        return altitudePlanner.height_above_target(current_loc, next_WP_loc);
    }
#endif

    return relative_altitude;
}

/*
  set the target altitude to the current altitude. This is used when 
  setting up for altitude hold, such as when releasing elevator in
  CRUISE mode.
 */
void AP_AltitudePlanner::set_target_altitude_current()
{
    Location current_loc;
    
    if (!AP::ahrs().get_location(current_loc)) {
      return;
    }

    // record altitude above sea level at the current time as our
    // target altitude
    _target_amsl_cm = current_loc.alt;

    // reset any glide slope offset
    reset_offset_altitude();

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (_terrain_enabled && AP::terrain()->height_above_terrain(terrain_altitude, true)) {
        _target_terrain_following = true;
        _target_terrain_alt_cm = terrain_altitude*100;
    } else {
        // if terrain following is disabled, or we don't know our
        // terrain altitude when we set the altitude then don't
        // terrain follow
        _target_terrain_following = false;
    }
#endif
}

/*
  set the target altitude to the current altitude, with ALT_OFFSET adjustment
 */
void AP_AltitudePlanner::set_target_altitude_current_adjusted()
{
    set_target_altitude_current();

    // use adjusted_altitude_cm() to take account of ALTITUDE_OFFSET
    float position_D = 0.0f;
    AP::ahrs().get_relative_position_D_home(position_D);
    _target_amsl_cm = -position_D - 100*_mission_alt_offset;
}

/*
  set target altitude based on a location structure
 */
void AP_AltitudePlanner::set_target_altitude_location(const Location &loc)
{
    _target_amsl_cm = loc.alt;

    if (loc.relative_alt) {
        _target_amsl_cm += AP::ahrs().get_home().alt;
    }
#if AP_TERRAIN_AVAILABLE
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (loc.terrain_alt && AP::terrain()->height_above_terrain(height, true)) {
        _target_terrain_following = true;
        _target_terrain_alt_cm = loc.alt;
        if (!loc.relative_alt) {
            // it has home added, remove it
            _target_terrain_alt_cm -= AP::ahrs().get_home().alt;
        }
    } else {
        _target_terrain_following = false;
    }
#endif
}

/*
  return relative to home target altitude in centimeters. Used for
  altitude control libraries
 */
int32_t AP_AltitudePlanner::relative_target_altitude_cm(float lookahead_adjustment, float rangefinder_correction)
{
#if AP_TERRAIN_AVAILABLE
    float relative_home_height;
    if (_target_terrain_following &&
        AP::terrain()->height_relative_home_equivalent(_target_terrain_alt_cm*0.01f,
                                                relative_home_height, true)) {
        // add lookahead adjustment the target altitude
        _target_lookahead = lookahead_adjustment;
        relative_home_height += _target_lookahead;

        // correct for rangefinder data
        relative_home_height += rangefinder_correction;

        // we are following terrain, and have terrain data for the
        // current location. Use it.
        return relative_home_height*100;
    }
#endif
    int32_t relative_alt = _target_amsl_cm - AP::ahrs().get_home().alt;
    relative_alt += _mission_alt_offset*100;
    relative_alt += rangefinder_correction * 100;
    return relative_alt;
}

/*
  change the current target altitude by an amount in centimeters. Used
  to cope with changes due to elevator in CRUISE or FBWB
 */
void AP_AltitudePlanner::change_target_altitude(int32_t change_cm)
{
    _target_amsl_cm += change_cm;
#if AP_TERRAIN_AVAILABLE
    if (_terrain_enabled && _target_terrain_following) {
        _target_terrain_alt_cm += change_cm;
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
void AP_AltitudePlanner::set_target_altitude_proportion(const Location &start_loc, const Location &destination_loc, float proportion, bool suppress_glideslope)
{
    set_target_altitude_location(destination_loc);
    proportion = constrain_float(proportion, 0.0f, 1.0f);
    change_target_altitude(-_target_offset_cm*proportion);
    //rebuild the glide slope if we are above it and supposed to be climbing
    if(_aparm.glide_slope_threshold > 0) {
        if(_target_offset_cm > 0 && calc_altitude_error_cm() < -100 * _aparm.glide_slope_threshold) {
            Location current_loc;
            if (!AP::ahrs().get_location(current_loc)) {
              return;
            }

            set_target_altitude_location(destination_loc);
            set_offset_altitude_location(current_loc, destination_loc, suppress_glideslope);
            change_target_altitude(-_target_offset_cm*proportion);
            //adjust the new target offset altitude to reflect that we are partially already done
            if(proportion > 0.0f)
                _target_offset_cm = ((float)_target_offset_cm)/proportion;
        }
    }
}

/*
  constrain target altitude to be between two locations. Used to
  ensure we stay within two waypoints in altitude
 */
void AP_AltitudePlanner::constrain_target_altitude_location(const Location &loc1, const Location &loc2)
{
    if (loc1.alt > loc2.alt) {
        _target_amsl_cm = constrain_int32(_target_amsl_cm, loc2.alt, loc1.alt);
    } else {
        _target_amsl_cm = constrain_int32(_target_amsl_cm, loc1.alt, loc2.alt);
    }
}

/*
  return error between target altitude and current altitude
 */
int32_t AP_AltitudePlanner::calc_altitude_error_cm()
{
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
      return 0;
    }

    AP::logger().Write("ALTP", "TimeUS,tgt,off,terr", "Qfff",
                               AP_HAL::micros64(),
                               (double)_target_amsl_cm,
                               (double)_target_offset_cm,
                               (double)_terrain_enabled);

#if AP_TERRAIN_AVAILABLE
    float terrain_height;
    if (is_terrain_following() && 
        AP::terrain()->height_above_terrain(terrain_height, true)) {
        return _target_lookahead*100 + _target_terrain_alt_cm - (terrain_height*100);
    }
#endif
    return _target_amsl_cm - (current_loc.alt - _mission_alt_offset*100);
}

/*
  check for FBWB_min_altitude_cm violation
 */
void AP_AltitudePlanner::check_fbwb_minimum_altitude(int32_t min_alt_cm)
{
    if (min_alt_cm == 0) {
        return;
    }

#if AP_TERRAIN_AVAILABLE
    if (is_terrain_following()) {
            // set our target terrain height to be at least the min set
            if (_target_terrain_alt_cm < min_alt_cm) {
                _target_terrain_alt_cm = min_alt_cm;
            }
            return;
    }
#endif

    if (_target_amsl_cm < AP::ahrs().get_home().alt + min_alt_cm) {
        _target_amsl_cm = AP::ahrs().get_home().alt + min_alt_cm;
    }
}

/*
  reset the altitude offset used for glide slopes
 */
void AP_AltitudePlanner::reset_offset_altitude(void)
{
    _target_offset_cm = 0;
}


/*
  reset the altitude offset used for glide slopes, based on difference
  between altitude at a destination and a specified start altitude. If
  destination is above the starting altitude then the result is
  positive.
 */
void AP_AltitudePlanner::set_offset_altitude_location(const Location &start_loc, const Location &destination_loc, bool force_glideslope)
{
    _target_offset_cm = destination_loc.alt - start_loc.alt;

#if AP_TERRAIN_AVAILABLE
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (destination_loc.terrain_alt && 
        is_terrain_following() &&
        AP::terrain()->height_above_terrain(height, true)) {
        _target_offset_cm = _target_terrain_alt_cm - (height * 100);
    }
#endif

    if (!force_glideslope) {
        // if we are within GLIDE_SLOPE_MIN meters of the target altitude
        // then reset the offset to not use a glide slope. This allows for
        // more accurate flight of missions where the aircraft may lose or
        // gain a bit of altitude near waypoint turn points due to local
        // terrain changes
        if (_aparm.glide_slope_min <= 0 ||
            labs(_target_offset_cm)*0.01f < _aparm.glide_slope_min) {
            _target_offset_cm = 0;
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
bool AP_AltitudePlanner::above_location(const Location &loc1, const Location &loc2)
{
#if AP_TERRAIN_AVAILABLE
    float terrain_alt;
    if (loc2.terrain_alt && 
        AP::terrain()->height_above_terrain(terrain_alt, true)) {
        float loc2_alt = loc2.alt*0.01f;
        if (!loc2.relative_alt) {
            loc2_alt -= AP::ahrs().get_home().alt*0.01f;
        }
        return terrain_alt > loc2_alt;
    }
#endif

    float loc2_alt_cm = loc2.alt;
    if (loc2.relative_alt) {
        loc2_alt_cm += AP::ahrs().get_home().alt;
    }
    return loc1.alt > loc2_alt_cm;
}

/*
  modify a destination to be setup for terrain following if
  TERRAIN_FOLLOW is enabled
 */
void Plane::setup_terrain_target_alt(Location &loc) const
{
#if AP_TERRAIN_AVAILABLE
    if (terrain_enabled_in_current_mode()) {
        loc.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN);
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
    return current_loc.alt - (mission_alt_offset()*100);
}

/*
  return home-relative altitude adjusted for ALT_OFFSET This is useful
  during long flights to account for barometer changes from the GCS,
  or to adjust the flying height of a long mission
 */
int32_t Plane::adjusted_relative_altitude_cm(void)
{
    return (relative_altitude - mission_alt_offset())*100;
}


/*
  return the mission altitude offset. This raises or lowers all
  mission items. It is primarily set using the ALT_OFFSET parameter,
  but can also be adjusted by the rangefinder landing code for a
  NAV_LAND command if we have aborted a steep landing
 */
float Plane::mission_alt_offset(void)
{
    float ret = g.alt_offset;
    if (control_mode == &mode_auto &&
            (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND || auto_state.wp_is_land_approach)) {
        // when landing after an aborted landing due to too high glide
        // slope we use an offset from the last landing attempt
        ret += landing.alt_offset;
    }
    return ret;
}

float AP_AltitudePlanner::height_above_target(const Location& loc, const Location& target_loc) const
{
    float target_alt = target_loc.alt*0.01;
    if (!target_loc.relative_alt) {
        target_alt -= AP::ahrs().get_home().alt*0.01f;
    }

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (target_loc.terrain_alt &&
        AP::terrain()->height_above_terrain(terrain_altitude, true)) {
        return terrain_altitude - target_alt;
    }
#endif

    return (loc.alt*0.01f - _mission_alt_offset - AP::ahrs().get_home().alt*0.01f) - target_alt;
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
    if (control_mode == &mode_fbwb) {
        // there is no target waypoint in FBWB, so use yaw as an approximation
        bearing_cd = ahrs.yaw_sensor;
        distance = g.terrain_lookahead;
    } else if (!reached_loiter_target()) {
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
    float climb_ratio = 0.5f * TECS_controller.get_max_climbrate() / groundspeed;

    if (climb_ratio <= 0) {
        // lookahead makes no sense for negative climb rates
        return 0;
    }
    
    // ask the terrain code for the lookahead altitude change
    float lookahead = terrain.lookahead(bearing_cd*0.01f, distance, climb_ratio);
    
    int32_t offset_cm = altitudePlanner.get_target_altitude_offset_cm();
    if (offset_cm < 0) {
        // we are heading down to the waypoint, so we don't need to
        // climb as much
        lookahead += offset_cm*0.01f;
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
    if (millis() - rangefinder_state.last_correction_time_ms > 5000) {
        // we haven't had any rangefinder data for 5s - don't use it
        return 0;
    }

    // for now we only support the rangefinder for landing 
    bool using_rangefinder = (g.rangefinder_landing && flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND);
    if (!using_rangefinder) {
        return 0;
    }

    return rangefinder_state.correction;
}

/*
  correct rangefinder data for terrain height difference between
  NAV_LAND point and current location
 */
void Plane::rangefinder_terrain_correction(float &height)
{
#if AP_TERRAIN_AVAILABLE
    if (!g.rangefinder_landing ||
        flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND ||
        !terrain_enabled_in_current_mode()) {
        return;
    }
    float terrain_amsl1, terrain_amsl2;
    if (!terrain.height_amsl(current_loc, terrain_amsl1) ||
        !terrain.height_amsl(next_WP_loc, terrain_amsl2)) {
        return;
    }
    float correction = (terrain_amsl1 - terrain_amsl2);
    height += correction;
    auto_state.terrain_correction = correction;
#endif
}

/*
  update the offset between rangefinder height and terrain height
 */
void Plane::rangefinder_height_update(void)
{
    float distance = rangefinder.distance_orient(ROTATION_PITCH_270);
    
    if ((rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) && ahrs.home_is_set()) {
        if (!rangefinder_state.have_initial_reading) {
            rangefinder_state.have_initial_reading = true;
            rangefinder_state.initial_range = distance;
        }
        // correct the range for attitude (multiply by DCM.c.z, which
        // is cos(roll)*cos(pitch))
        rangefinder_state.height_estimate = distance * ahrs.get_rotation_body_to_ned().c.z;

        rangefinder_terrain_correction(rangefinder_state.height_estimate);

        // we consider ourselves to be fully in range when we have 10
        // good samples (0.2s) that are different by 5% of the maximum
        // range from the initial range we see. The 5% change is to
        // catch Lidars that are giving a constant range, either due
        // to misconfiguration or a faulty sensor
        if (rangefinder_state.in_range_count < 10) {
            if (!is_equal(distance, rangefinder_state.last_distance) &&
                fabsf(rangefinder_state.initial_range - distance) > 0.05f * rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)*0.01f) {
                rangefinder_state.in_range_count++;
            }
            if (fabsf(rangefinder_state.last_distance - distance) > rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)*0.01*0.2) {
                // changes by more than 20% of full range will reset counter
                rangefinder_state.in_range_count = 0;
            }
        } else {
            rangefinder_state.in_range = true;
            bool flightstage_good_for_rangefinder_landing = false;
            if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
                flightstage_good_for_rangefinder_landing = true;
            }
#if HAL_QUADPLANE_ENABLED
            if (control_mode == &mode_qland ||
                control_mode == &mode_qrtl ||
                (control_mode == &mode_auto && quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id))) {
                flightstage_good_for_rangefinder_landing = true;
            }
#endif
            if (!rangefinder_state.in_use &&
                flightstage_good_for_rangefinder_landing &&
                g.rangefinder_landing) {
                rangefinder_state.in_use = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder engaged at %.2fm", (double)rangefinder_state.height_estimate);
            }
        }
        rangefinder_state.last_distance = distance;
    } else {
        rangefinder_state.in_range_count = 0;
        rangefinder_state.in_range = false;
    }

    if (rangefinder_state.in_range) {
        // base correction is the difference between baro altitude and
        // rangefinder estimate
        float correction = adjusted_relative_altitude_cm()*0.01 - rangefinder_state.height_estimate;

#if AP_TERRAIN_AVAILABLE
        // if we are terrain following then correction is based on terrain data
        float terrain_altitude;
        if ((altitudePlanner.is_terrain_following() || terrain_enabled_in_current_mode()) &&
            terrain.height_above_terrain(terrain_altitude, true)) {
            correction = terrain_altitude - rangefinder_state.height_estimate;
        }
#endif    

        // remember the last correction. Use a low pass filter unless
        // the old data is more than 5 seconds old
        uint32_t now = millis();
        if (now - rangefinder_state.last_correction_time_ms > 5000) {
            rangefinder_state.correction = correction;
            rangefinder_state.initial_correction = correction;
            if (g.rangefinder_landing) {
                landing.set_initial_slope();
            }
            rangefinder_state.last_correction_time_ms = now;
        } else {
            rangefinder_state.correction = 0.8f*rangefinder_state.correction + 0.2f*correction;
            rangefinder_state.last_correction_time_ms = now;
            if (fabsf(rangefinder_state.correction - rangefinder_state.initial_correction) > 30) {
                // the correction has changed by more than 30m, reset use of Lidar. We may have a bad lidar
                if (rangefinder_state.in_use) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder disengaged at %.2fm", (double)rangefinder_state.height_estimate);
                }
                memset(&rangefinder_state, 0, sizeof(rangefinder_state));
            }
        }
        
    }
}

/*
  determine if Non Auto Terrain Disable is active and allowed in present control mode
 */
bool Plane::terrain_disabled()
{
    return control_mode->allows_terrain_disable() && non_auto_terrain_disable;
}


/*
  Check if terrain following is enabled for the current mode
 */
#if AP_TERRAIN_AVAILABLE
const Plane::TerrainLookupTable Plane::Terrain_lookup[] = {
    {Mode::Number::FLY_BY_WIRE_B, terrain_bitmask::FLY_BY_WIRE_B},
    {Mode::Number::CRUISE, terrain_bitmask::CRUISE},
    {Mode::Number::AUTO, terrain_bitmask::AUTO},
    {Mode::Number::RTL, terrain_bitmask::RTL},
    {Mode::Number::AVOID_ADSB, terrain_bitmask::AVOID_ADSB},
    {Mode::Number::GUIDED, terrain_bitmask::GUIDED},
    {Mode::Number::LOITER, terrain_bitmask::LOITER},
    {Mode::Number::CIRCLE, terrain_bitmask::CIRCLE},
#if HAL_QUADPLANE_ENABLED
    {Mode::Number::QRTL, terrain_bitmask::QRTL},
    {Mode::Number::QLAND, terrain_bitmask::QLAND},
    {Mode::Number::QLOITER, terrain_bitmask::QLOITER},
#endif
};

bool Plane::terrain_enabled_in_current_mode() const
{
    return terrain_enabled_in_mode(control_mode->mode_number());
}

bool Plane::terrain_enabled_in_mode(Mode::Number num) const
{
    // Global enable
    if ((g.terrain_follow.get() & int32_t(terrain_bitmask::ALL)) != 0) {
        return true;
    }

    // Specific enable
    for (const struct TerrainLookupTable entry : Terrain_lookup) {
        if (entry.mode_num == num) {
            if ((g.terrain_follow.get() & int32_t(entry.bitmask)) != 0) {
                return true;
            }
            break;
        }
    }

    return false;
}
#endif
