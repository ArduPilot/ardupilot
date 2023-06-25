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
 *   AP_Landing_Slope.cpp - Landing logic handler for ArduPlane for STANDARD_GLIDE_SLOPE
 */

#include "AP_Landing.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_LandingGear/AP_LandingGear.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Logger/AP_Logger.h>

#if defined(APM_BUILD_TYPE)
//  - this is just here to encourage the build system to supply the "legacy build defines".  The actual dependecy is in the AP_LandingGear.h and AP_LandingGear_config.h headers
#endif

void AP_Landing::type_slope_do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude)
{
    initial_slope = 0;
    slope = 0;

    // once landed, post some landing statistics to the GCS
    type_slope_flags.post_stats = false;

    type_slope_stage = SlopeStage::NORMAL;
    gcs().send_text(MAV_SEVERITY_INFO, "Landing approach start at %.1fm", (double)relative_altitude);
}

void AP_Landing::type_slope_verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, bool &throttle_suppressed)
{
    // when aborting a landing, mimic the verify_takeoff with steering hold. Once
    // the altitude has been reached, restart the landing sequence
    throttle_suppressed = false;
    nav_controller->update_heading_hold(prev_WP_loc.get_bearing_to(next_WP_loc));
}

/*
  update navigation for landing. Called when on landing approach or
  final flare
 */
bool AP_Landing::type_slope_verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
        const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms, const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range)
{
    // we don't 'verify' landing in the sense that it never completes,
    // so we don't verify command completion. Instead we use this to
    // adjust final landing parameters

    // determine stage
    if (type_slope_stage == SlopeStage::NORMAL) {
        const bool heading_lined_up = abs(nav_controller->bearing_error_cd()) < 1000 && !nav_controller->data_is_stale();
        const bool on_flight_line = fabsf(nav_controller->crosstrack_error()) < 5.0f && !nav_controller->data_is_stale();
        const bool below_prev_WP = current_loc.alt < prev_WP_loc.alt;
        if ((mission.get_prev_nav_cmd_id() == MAV_CMD_NAV_LOITER_TO_ALT) ||
            (wp_proportion >= 0 && heading_lined_up && on_flight_line) ||
            (wp_proportion > 0.15f && heading_lined_up && below_prev_WP) ||
            (wp_proportion > 0.5f)) {
            type_slope_stage = SlopeStage::APPROACH;
        }
    }

    /* Set land_complete (which starts the flare) under 3 conditions:
       1) we are within LAND_FLARE_ALT meters of the landing altitude
       2) we are within LAND_FLARE_SEC of the landing point vertically
          by the calculated sink rate (if LAND_FLARE_SEC != 0)
       3) we have gone past the landing point and don't have
          rangefinder data (to prevent us keeping throttle on
          after landing if we've had positive baro drift)
    */

    // flare check:
    // 1) below flare alt/sec requires approach stage check because if sec/alt are set too
    //    large, and we're on a hard turn to line up for approach, we'll prematurely flare by
    //    skipping approach phase and the extreme roll limits will make it hard to line up with runway
    // 2) passed land point and don't have an accurate AGL
    // 3) probably crashed (ensures motor gets turned off)

    const bool on_approach_stage = type_slope_is_on_approach();
    const bool below_flare_alt = (height <= flare_alt);
    const bool below_flare_sec = (flare_sec > 0 && height <= sink_rate * flare_sec);
    const bool probably_crashed = (aparm.crash_detection_enable && fabsf(sink_rate) < 0.2f && !is_flying);

    height_flare_log = height;

    const AP_GPS &gps = AP::gps();

    if ((on_approach_stage && below_flare_alt) ||
        (on_approach_stage && below_flare_sec && (wp_proportion > 0.5)) ||
        (!rangefinder_state_in_range && wp_proportion >= 1) ||
        probably_crashed) {

        if (type_slope_stage != SlopeStage::FINAL) {
            type_slope_flags.post_stats = true;
            if (is_flying && (AP_HAL::millis()-last_flying_ms) > 3000) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Flare crash detected: speed=%.1f", (double)gps.ground_speed());
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Flare %.1fm sink=%.2f speed=%.1f dist=%.1f",
                                  (double)height, (double)sink_rate,
                                  (double)gps.ground_speed(),
                                  (double)current_loc.get_distance(next_WP_loc));
            }
            
            type_slope_stage = SlopeStage::FINAL;

#if AP_LANDINGGEAR_ENABLED
            // Check if the landing gear was deployed before landing
            // If not - go around
            AP_LandingGear *LG_inst = AP_LandingGear::get_singleton();
            if (LG_inst != nullptr && !LG_inst->check_before_land()) {
                type_slope_request_go_around();
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Landing gear was not deployed");
            }
#endif
        }

        if (gps.ground_speed() < 3) {
            // reload any airspeed or groundspeed parameters that may have
            // been set for landing. We don't do this till ground
            // speed drops below 3.0 m/s as otherwise we will change
            // target speeds too early.
            aparm.airspeed_cruise_cm.load();
            aparm.min_gndspeed_cm.load();
            aparm.throttle_cruise.load();
        }
    } else if (type_slope_stage == SlopeStage::APPROACH && pre_flare_airspeed > 0) {
        bool reached_pre_flare_alt = pre_flare_alt > 0 && (height <= pre_flare_alt);
        bool reached_pre_flare_sec = pre_flare_sec > 0 && (height <= sink_rate * pre_flare_sec);
        if (reached_pre_flare_alt || reached_pre_flare_sec) {
            type_slope_stage = SlopeStage::PREFLARE;
        }
    }

    /*
      when landing we keep the L1 navigation waypoint 200m ahead. This
      prevents sudden turns if we overshoot the landing point
     */
    Location land_WP_loc = next_WP_loc;

    int32_t land_bearing_cd = prev_WP_loc.get_bearing_to(next_WP_loc);
    land_WP_loc.offset_bearing(land_bearing_cd * 0.01f, prev_WP_loc.get_distance(current_loc) + 200);
    nav_controller->update_waypoint(prev_WP_loc, land_WP_loc);

    // once landed and stationary, post some statistics
    // this is done before disarm_if_autoland_complete() so that it happens on the next loop after the disarm
    if (type_slope_flags.post_stats && !is_armed) {
        type_slope_flags.post_stats = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Distance from LAND point=%.2fm", (double)current_loc.get_distance(next_WP_loc));
    }

    // check if we should auto-disarm after a confirmed landing
    if (type_slope_stage == SlopeStage::FINAL) {
        disarm_if_autoland_complete_fn();
    }

    if (mission.continue_after_land() &&
        type_slope_stage == SlopeStage::FINAL &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D &&
        gps.ground_speed() < 1) {
        /*
          user has requested to continue with mission after a
          landing. Return true to allow for continue
         */
        return true;
    }

    /*
      we return false as a landing mission item never completes

      we stay on this waypoint unless the GCS commands us to change
      mission item, reset the mission, command a go-around or finish
      a land_abort procedure.
     */
    return false;
}

void AP_Landing::type_slope_adjust_landing_slope_for_rangefinder_bump(AP_FixedWing::Rangefinder_State &rangefinder_state, Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc, const float wp_distance, int32_t &target_altitude_offset_cm)
{
    // check the rangefinder correction for a large change. When found, recalculate the glide slope. This is done by
    // determining the slope from your current location to the land point then following that back up to the approach
    // altitude and moving the prev_wp to that location. From there
    float correction_delta = fabsf(rangefinder_state.last_stable_correction) - fabsf(rangefinder_state.correction);

    if (slope_recalc_shallow_threshold <= 0 ||
            fabsf(correction_delta) < slope_recalc_shallow_threshold) {
        return;
    }

    rangefinder_state.last_stable_correction = rangefinder_state.correction;

    float corrected_alt_m = (adjusted_altitude_cm_fn() - next_WP_loc.alt)*0.01f - rangefinder_state.correction;
    float total_distance_m = prev_WP_loc.get_distance(next_WP_loc);
    float top_of_glide_slope_alt_m = total_distance_m * corrected_alt_m / wp_distance;
    prev_WP_loc.alt = top_of_glide_slope_alt_m*100 + next_WP_loc.alt;

    // re-calculate auto_state.land_slope with updated prev_WP_loc
    setup_landing_glide_slope(prev_WP_loc, next_WP_loc, current_loc, target_altitude_offset_cm);

    if (rangefinder_state.correction >= 0) { // we're too low or object is below us
        // correction positive means we're too low so we should continue on with
        // the newly computed shallower slope instead of pitching/throttling up

    } else if (slope_recalc_steep_threshold_to_abort > 0 && !type_slope_flags.has_aborted_due_to_slope_recalc) {
        // correction negative means we're too high and need to point down (and speed up) to re-align
        // to land on target. A large negative correction means we would have to dive down a lot and will
        // generating way too much speed that we can not bleed off in time. It is better to remember
        // the large baro altitude offset and abort the landing to come around again with the correct altitude
        // offset and "perfect" slope.

        // calculate projected slope with projected alt
        float new_slope_deg = degrees(atanf(slope));
        float initial_slope_deg = degrees(atanf(initial_slope));

        // is projected slope too steep?
        if (new_slope_deg - initial_slope_deg > slope_recalc_steep_threshold_to_abort) {
            gcs().send_text(MAV_SEVERITY_INFO, "Landing slope too steep, aborting (%.0fm %.1fdeg)",
                                             (double)rangefinder_state.correction, (double)(new_slope_deg - initial_slope_deg));
            alt_offset = rangefinder_state.correction;
            flags.commanded_go_around = true;
            type_slope_flags.has_aborted_due_to_slope_recalc = true; // only allow this once.
            Log();
        }
    }
}

bool AP_Landing::type_slope_request_go_around(void)
{
    flags.commanded_go_around = true;
    return true;
}

/*
  a special glide slope calculation for the landing approach

  During the land approach use a linear glide slope to a point
  projected through the landing point. We don't use the landing point
  itself as that leads to discontinuities close to the landing point,
  which can lead to erratic pitch control
 */
void AP_Landing::type_slope_setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc, const Location &current_loc, int32_t &target_altitude_offset_cm)
{
    float total_distance = prev_WP_loc.get_distance(next_WP_loc);

    // If someone mistakenly puts all 0's in their LAND command then total_distance
    // will be calculated as 0 and cause a divide by 0 error below.  Lets avoid that.
    if (total_distance < 1) {
        total_distance = 1;
    }

    // height we need to sink for this WP
    float sink_height = (prev_WP_loc.alt - next_WP_loc.alt)*0.01f;

    // current ground speed
    float groundspeed = ahrs.groundspeed();
    if (groundspeed < 0.5f) {
        groundspeed = 0.5f;
    }

    // calculate time to lose the needed altitude
    float sink_time = total_distance / groundspeed;
    if (sink_time < 0.5f) {
        sink_time = 0.5f;
    }

    // find the sink rate needed for the target location
    float sink_rate = sink_height / sink_time;

    // the height we aim for is the one to give us the right flare point
    float aim_height = flare_sec * sink_rate;
    if (aim_height <= 0) {
        aim_height = flare_alt;
    }

    // don't allow the aim height to be too far above LAND_FLARE_ALT
    if (flare_alt > 0 && aim_height > flare_alt*2) {
        aim_height = flare_alt*2;
    }

    // calculate time spent in flare assuming the sink rate reduces over time from sink_rate at aim_height
    // to tecs_controller->get_land_sinkrate() at touchdown
    const float weight = constrain_float(0.01f*(float)flare_effectivness_pct, 0.0f, 1.0f);
    const float flare_sink_rate_avg = MAX(weight * tecs_Controller->get_land_sinkrate() + (1.0f - weight) * sink_rate, 0.1f);
    const float flare_time = aim_height / flare_sink_rate_avg;

    // distance to flare is based on ground speed, adjusted as we
    // get closer. This takes into account the wind
    float flare_distance = groundspeed * flare_time;

    // don't allow the flare before half way along the final leg
    if (flare_distance > total_distance*0.5f) {
        flare_distance = total_distance*0.5f;
    }

    // project a point 500 meters past the landing point, passing
    // through the landing point
    const float land_projection = 500;
    int32_t land_bearing_cd = prev_WP_loc.get_bearing_to(next_WP_loc);

    // now calculate our aim point, which is before the landing
    // point and above it
    Location loc = next_WP_loc;
    loc.offset_bearing(land_bearing_cd * 0.01f, -flare_distance);
    loc.alt += aim_height*100;

    // calculate slope to landing point
    bool is_first_calc = is_zero(slope);
    slope = (sink_height - aim_height) / (total_distance - flare_distance);
    if (is_first_calc) {
        gcs().send_text(MAV_SEVERITY_INFO, "Landing glide slope %.1f degrees", (double)degrees(atanf(slope)));
    }

    // calculate point along that slope 500m ahead
    loc.offset_bearing(land_bearing_cd * 0.01f, land_projection);
    loc.alt -= slope * land_projection * 100;

    // setup the offset_cm for set_target_altitude_proportion()
    target_altitude_offset_cm = loc.alt - prev_WP_loc.alt;

    // calculate the proportion we are to the target
    float land_proportion = current_loc.line_path_proportion(prev_WP_loc, loc);

    // now setup the glide slope for landing
    set_target_altitude_proportion_fn(loc, 1.0f - land_proportion);

    // stay within the range of the start and end locations in altitude
    constrain_target_altitude_location_fn(loc, prev_WP_loc);
}

int32_t AP_Landing::type_slope_get_target_airspeed_cm(void)
{
    // we're landing, check for custom approach and
    // pre-flare airspeeds. Also increase for head-winds

    const float land_airspeed = tecs_Controller->get_land_airspeed();
    int32_t target_airspeed_cm = aparm.airspeed_cruise_cm;
    if (land_airspeed >= 0) {
        target_airspeed_cm = land_airspeed * 100;
    } else {
        target_airspeed_cm = 0.5 * (aparm.airspeed_cruise_cm * 0.01 + aparm.airspeed_min);
    }
    switch (type_slope_stage) {
    case SlopeStage::NORMAL:
        target_airspeed_cm = aparm.airspeed_cruise_cm;
        break;
    case SlopeStage::APPROACH:
        break;
    case SlopeStage::PREFLARE:
    case SlopeStage::FINAL:
        if (pre_flare_airspeed > 0) {
            // if we just preflared then continue using the pre-flare airspeed during final flare
            target_airspeed_cm = pre_flare_airspeed * 100;
        }
        break;
    }

    // when landing, add half of head-wind.
    const float head_wind_comp = constrain_float(wind_comp, 0.0f, 100.0f)*0.01;
    const int32_t head_wind_compensation_cm = head_wind() * head_wind_comp * 100;

    const uint32_t max_airspeed_cm = AP_Landing::allow_max_airspeed_on_land() ? aparm.airspeed_max*100 : aparm.airspeed_cruise_cm;
    
    return constrain_int32(target_airspeed_cm + head_wind_compensation_cm, target_airspeed_cm, max_airspeed_cm);
    
}

int32_t AP_Landing::type_slope_constrain_roll(const int32_t desired_roll_cd, const int32_t level_roll_limit_cd)
{
    if (type_slope_stage == SlopeStage::FINAL) {
        return constrain_int32(desired_roll_cd, level_roll_limit_cd * -1, level_roll_limit_cd);
    } else {
        return desired_roll_cd;
    }
}

bool AP_Landing::type_slope_is_flaring(void) const
{
    return (type_slope_stage == SlopeStage::FINAL);
}

bool AP_Landing::type_slope_is_on_approach(void) const
{
    return (type_slope_stage == SlopeStage::APPROACH ||
            type_slope_stage == SlopeStage::PREFLARE);
}

bool AP_Landing::type_slope_is_expecting_impact(void) const
{
    return (type_slope_stage == SlopeStage::PREFLARE ||
            type_slope_stage == SlopeStage::FINAL);
}

bool AP_Landing::type_slope_is_complete(void) const
{
    return (type_slope_stage == SlopeStage::FINAL);
}

void AP_Landing::type_slope_log(void) const
{
// @LoggerMessage: LAND
// @Description: Slope Landing data
// @Field: TimeUS: Time since system startup
// @Field: stage: progress through landing sequence
// @Field: f1: Landing flags
// @Field: f2: Slope-specific landing flags
// @Field: slope: Slope to landing point
// @Field: slopeInit: Initial slope to landing point
// @Field: altO: Rangefinder correction
// @Field: fh: Height for flare timing.
    AP::logger().WriteStreaming("LAND", "TimeUS,stage,f1,f2,slope,slopeInit,altO,fh", "QBBBffff",
                                            AP_HAL::micros64(),
                                            type_slope_stage,
                                            flags,
                                            type_slope_flags,
                                            (double)slope,
                                            (double)initial_slope,
                                            (double)alt_offset,
                                            (double)height_flare_log);
}

bool AP_Landing::type_slope_is_throttle_suppressed(void) const
{
    return type_slope_stage == SlopeStage::FINAL;
}
