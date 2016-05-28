/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  landing logic
 */

/*
  update navigation for landing. Called when on landing approach or
  final flare
 */
static bool verify_land()
{
    // we don't 'verify' landing in the sense that it never completes,
    // so we don't verify command completion. Instead we use this to
    // adjust final landing parameters

    float height = height_above_target();

    // calculate the sink rate.
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    auto_state.land_sink_rate = 0.8f * auto_state.land_sink_rate + 0.2f*sink_rate;
    
    /* Set land_complete (which starts the flare) under 2 conditions:
       1) we are within LAND_FLARE_ALT meters of the landing altitude
       2) we are within LAND_FLARE_SEC of the landing point vertically
          by the calculated sink rate
    */
    if (height <= g.land_flare_alt ||
        height <= -auto_state.land_sink_rate * g.land_flare_sec) {

        if (!auto_state.land_complete) {
            gcs_send_text_fmt(PSTR("Flare %.1fm sink=%.2f speed=%.1f"), 
                              height, auto_state.land_sink_rate, gps.ground_speed());
        }
        auto_state.land_complete = true;

        if (gps.ground_speed() < 3) {
            // reload any airspeed or groundspeed parameters that may have
            // been set for landing. We don't do this till ground
            // speed drops below 3.0 m/s as otherwise we will change
            // target speeds too early.
            g.airspeed_cruise_cm.load();
            g.min_gndspeed_cm.load();
            aparm.throttle_cruise.load();
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

    /*
      we always return false as a landing mission item never
      completes - we stay on this waypoint unless the GCS commands us
      to change mission item or reset the mission
     */
    return false;
}

