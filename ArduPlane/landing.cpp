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
    bool below_flare_alt = (height <= g.land_flare_alt);
    bool below_flare_sec = (aparm.land_flare_sec > 0 && height <= auto_state.sink_rate * aparm.land_flare_sec);
    bool probably_crashed = (g.crash_detection_enable && fabsf(auto_state.sink_rate) < 0.2f && !is_flying());

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
            g.airspeed_cruise_cm.load();
            g.min_gndspeed_cm.load();
            aparm.throttle_cruise.load();
        }
    } else if (!landing.complete && !landing.pre_flare && aparm.land_pre_flare_airspeed > 0) {
        bool reached_pre_flare_alt = g.land_pre_flare_alt > 0 && (height <= g.land_pre_flare_alt);
        bool reached_pre_flare_sec = g.land_pre_flare_sec > 0 && (height <= auto_state.sink_rate * g.land_pre_flare_sec);
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

    if (g.land_slope_recalc_shallow_threshold <= 0 ||
            fabsf(correction_delta) < g.land_slope_recalc_shallow_threshold) {
        return;
    }

    rangefinder_state.last_stable_correction = rangefinder_state.correction;

    float corrected_alt_m = (adjusted_altitude_cm() - next_WP_loc.alt)*0.01f - rangefinder_state.correction;
    float total_distance_m = get_distance(prev_WP_loc, next_WP_loc);
    float top_of_glide_slope_alt_m = total_distance_m * corrected_alt_m / auto_state.wp_distance;
    prev_WP_loc.alt = top_of_glide_slope_alt_m*100 + next_WP_loc.alt;

    // re-calculate auto_state.land_slope with updated prev_WP_loc
    setup_landing_glide_slope();

    if (rangefinder_state.correction >= 0) { // we're too low or object is below us
        // correction positive means we're too low so we should continue on with
        // the newly computed shallower slope instead of pitching/throttling up

    } else if (g.land_slope_recalc_steep_threshold_to_abort > 0) {
        // correction negative means we're too high and need to point down (and speed up) to re-align
        // to land on target. A large negative correction means we would have to dive down a lot and will
        // generating way too much speed that we can not bleed off in time. It is better to remember
        // the large baro altitude offset and abort the landing to come around again with the correct altitude
        // offset and "perfect" slope.

        // calculate projected slope with projected alt
        float new_slope_deg = degrees(atan(landing.slope));
        float initial_slope_deg = degrees(atan(landing.initial_slope));

        // is projected slope too steep?
        if (new_slope_deg - initial_slope_deg > g.land_slope_recalc_steep_threshold_to_abort) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Steep landing slope (%.0fm %.1fdeg)",
                                             (double)rangefinder_state.correction, (double)(new_slope_deg - initial_slope_deg));
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "aborting landing!");
            landing.alt_offset = rangefinder_state.correction;
            auto_state.commanded_go_around = 1;
            g.land_slope_recalc_steep_threshold_to_abort = 0; // disable this feature so we only perform it once
        }
    }
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
        float total_distance = get_distance(prev_WP_loc, next_WP_loc);

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
        float aim_height = aparm.land_flare_sec * sink_rate;
        if (aim_height <= 0) {
            aim_height = g.land_flare_alt;
        }

        // don't allow the aim height to be too far above LAND_FLARE_ALT
        if (g.land_flare_alt > 0 && aim_height > g.land_flare_alt*2) {
            aim_height = g.land_flare_alt*2;
        }

        // calculate slope to landing point
        bool is_first_calc = is_zero(landing.slope);
        landing.slope = (sink_height - aim_height) / total_distance;
        if (is_first_calc) {
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Landing glide slope %.1f degrees", (double)degrees(atanf(landing.slope)));
        }


        // time before landing that we will flare
        float flare_time = aim_height / SpdHgt_Controller->get_land_sinkrate();

        // distance to flare is based on ground speed, adjusted as we
        // get closer. This takes into account the wind
        float flare_distance = groundspeed * flare_time;

        // don't allow the flare before half way along the final leg
        if (flare_distance > total_distance/2) {
            flare_distance = total_distance/2;
        }

        // project a point 500 meters past the landing point, passing
        // through the landing point
        const float land_projection = 500;
        int32_t land_bearing_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);

        // now calculate our aim point, which is before the landing
        // point and above it
        Location loc = next_WP_loc;
        location_update(loc, land_bearing_cd*0.01f, -flare_distance);
        loc.alt += aim_height*100;

        // calculate point along that slope 500m ahead
        location_update(loc, land_bearing_cd*0.01f, land_projection);
        loc.alt -= landing.slope * land_projection * 100;

        // setup the offset_cm for set_target_altitude_proportion()
        target_altitude.offset_cm = loc.alt - prev_WP_loc.alt;

        // calculate the proportion we are to the target
        float land_proportion = location_path_proportion(current_loc, prev_WP_loc, loc);

        // now setup the glide slope for landing
        set_target_altitude_proportion(loc, 1.0f - land_proportion);

        // stay within the range of the start and end locations in altitude
        constrain_target_altitude_location(loc, prev_WP_loc);
}



