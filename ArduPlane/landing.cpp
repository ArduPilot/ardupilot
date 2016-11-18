#include "Plane.h"

/*
  landing logic
 */

/*
  update navigation for landing. Called when on landing approach or
  final flare
 */
bool Plane::verify_land()
{
    // we don't 'verify' landing in the sense that it never completes,
    // so we don't verify command completion. Instead we use this to
    // adjust final landing parameters

    // when aborting a landing, mimic the verify_takeoff with steering hold. Once
    // the altitude has been reached, restart the landing sequence
    if (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_ABORT) {

        throttle_suppressed = false;
        landing.complete = false;
        landing.pre_flare = false;
        nav_controller->update_heading_hold(get_bearing_cd(prev_WP_loc, next_WP_loc));

        // see if we have reached abort altitude
        if (adjusted_relative_altitude_cm() > auto_state.takeoff_altitude_rel_cm) {
            next_WP_loc = current_loc;
            mission.stop();
            bool success = landing.restart_landing_sequence();
            mission.resume();
            if (!success) {
                // on a restart failure lets RTL or else the plane may fly away with nowhere to go!
                set_mode(RTL, MODE_REASON_MISSION_END);
            }
            // make sure to return false so it leaves the mission index alone
        }
        return false;
    }

    float height = height_above_target();

    // use rangefinder to correct if possible
    height -= rangefinder_correction();

    /* Set land_complete (which starts the flare) under 3 conditions:
       1) we are within LAND_FLARE_ALT meters of the landing altitude
       2) we are within LAND_FLARE_SEC of the landing point vertically
          by the calculated sink rate (if LAND_FLARE_SEC != 0)
       3) we have gone past the landing point and don't have
          rangefinder data (to prevent us keeping throttle on
          after landing if we've had positive baro drift)
    */
    bool rangefinder_in_range = rangefinder_state.in_range;

    // flare check:
    // 1) below flare alt/sec requires approach stage check because if sec/alt are set too
    //    large, and we're on a hard turn to line up for approach, we'll prematurely flare by
    //    skipping approach phase and the extreme roll limits will make it hard to line up with runway
    // 2) passed land point and don't have an accurate AGL
    // 3) probably crashed (ensures motor gets turned off)

    bool on_approach_stage = (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH ||
                              flight_stage == AP_SpdHgtControl::FLIGHT_LAND_PREFLARE);
    bool below_flare_alt = (height <= aparm.land_flare_alt);
    bool below_flare_sec = (aparm.land_flare_sec > 0 && height <= auto_state.sink_rate * aparm.land_flare_sec);
    bool probably_crashed = (aparm.crash_detection_enable && fabsf(auto_state.sink_rate) < 0.2f && !is_flying());

    if ((on_approach_stage && below_flare_alt) ||
        (on_approach_stage && below_flare_sec && (auto_state.wp_proportion > 0.5)) ||
        (!rangefinder_in_range && auto_state.wp_proportion >= 1) ||
        probably_crashed) {

        if (!landing.complete) {
            landing.post_stats = true;
            if (!is_flying() && (millis()-auto_state.last_flying_ms) > 3000) {
                gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "Flare crash detected: speed=%.1f", (double)gps.ground_speed());
            } else {
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Flare %.1fm sink=%.2f speed=%.1f dist=%.1f",
                                  (double)height, (double)auto_state.sink_rate,
                                  (double)gps.ground_speed(),
                                  (double)get_distance(current_loc, next_WP_loc));
            }
            landing.complete = true;
            update_flight_stage();
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
    } else if (!landing.complete && !landing.pre_flare && aparm.land_pre_flare_airspeed > 0) {
        bool reached_pre_flare_alt = aparm.land_pre_flare_alt > 0 && (height <= aparm.land_pre_flare_alt);
        bool reached_pre_flare_sec = aparm.land_pre_flare_sec > 0 && (height <= auto_state.sink_rate * aparm.land_pre_flare_sec);
        if (reached_pre_flare_alt || reached_pre_flare_sec) {
            landing.pre_flare = true;
            update_flight_stage();
        }
    }

    /*
      when landing we keep the L1 navigation waypoint 200m ahead. This
      prevents sudden turns if we overshoot the landing point
     */
    struct Location land_WP_loc = next_WP_loc;
    int32_t land_bearing_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);
    location_update(land_WP_loc,
                    land_bearing_cd*0.01f,
                    get_distance(prev_WP_loc, current_loc) + 200);
    nav_controller->update_waypoint(prev_WP_loc, land_WP_loc);

    // once landed and stationary, post some statistics
    // this is done before disarm_if_autoland_complete() so that it happens on the next loop after the disarm
    if (landing.post_stats && !arming.is_armed()) {
        landing.post_stats = false;
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Distance from LAND point=%.2fm", (double)get_distance(current_loc, next_WP_loc));
    }

    // check if we should auto-disarm after a confirmed landing
    disarm_if_autoland_complete();

    /*
      we return false as a landing mission item never completes

      we stay on this waypoint unless the GCS commands us to change
      mission item, reset the mission, command a go-around or finish
      a land_abort procedure.
     */
    return false;
}

void Plane::adjust_landing_slope_for_rangefinder_bump(void)
{
    // check the rangefinder correction for a large change. When found, recalculate the glide slope. This is done by
    // determining the slope from your current location to the land point then following that back up to the approach
    // altitude and moving the prev_wp to that location. From there
    float correction_delta = fabsf(rangefinder_state.last_stable_correction) - fabsf(rangefinder_state.correction);

    if (aparm.land_slope_recalc_shallow_threshold <= 0 ||
            fabsf(correction_delta) < aparm.land_slope_recalc_shallow_threshold) {
        return;
    }

    rangefinder_state.last_stable_correction = rangefinder_state.correction;

    float corrected_alt_m = (adjusted_altitude_cm() - next_WP_loc.alt)*0.01f - rangefinder_state.correction;
    float total_distance_m = get_distance(prev_WP_loc, next_WP_loc);
    float top_of_glide_slope_alt_m = total_distance_m * corrected_alt_m / auto_state.wp_distance;
    prev_WP_loc.alt = top_of_glide_slope_alt_m*100 + next_WP_loc.alt;

    // re-calculate auto_state.land_slope with updated prev_WP_loc
    setup_landing_glide_slope();

    landing.check_if_need_to_abort(rangefinder_state);
}

/*
  a special glide slope calculation for the landing approach

  During the land approach use a linear glide slope to a point
  projected through the landing point. We don't use the landing point
  itself as that leads to discontinuities close to the landing point,
  which can lead to erratic pitch control
 */
void Plane::setup_landing_glide_slope(void)
{
    Location loc = landing.setup_landing_glide_slope(prev_WP_loc, next_WP_loc);

    // setup the offset_cm for set_target_altitude_proportion()
    target_altitude.offset_cm = loc.alt - prev_WP_loc.alt;

    // calculate the proportion we are to the target
    float land_proportion = location_path_proportion(current_loc, prev_WP_loc, loc);

    // now setup the glide slope for landing
    set_target_altitude_proportion(loc, 1.0f - land_proportion);

    // stay within the range of the start and end locations in altitude
    constrain_target_altitude_location(loc, prev_WP_loc);
}



