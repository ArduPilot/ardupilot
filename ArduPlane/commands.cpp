/*
 *  logic for dealing with the current command in the mission and home location
 */

#include "Plane.h"

/*
 *  set_next_WP - sets the target location the vehicle should fly to
 */
void Plane::set_next_WP(const Location &loc)
{
    if (auto_state.next_wp_crosstrack) {
        // copy the current WP into the OldWP slot
        prev_WP_loc = next_WP_loc;
        auto_state.crosstrack = true;
    } else {
        // we should not try to cross-track for this waypoint
        prev_WP_loc = current_loc;
        // use cross-track for the next waypoint
        auto_state.next_wp_crosstrack = true;
        auto_state.crosstrack = false;
    }

    // Load the next_WP slot
    // ---------------------
    next_WP_loc = loc;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        next_WP_loc.lat = current_loc.lat;
        next_WP_loc.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP_loc.alt_is_zero()) {
            next_WP_loc.copy_alt_from(current_loc);
        }
    }

    fix_terrain_WP(next_WP_loc, __LINE__);

    // convert relative alt to absolute alt
    if (!next_WP_loc.terrain_alt) {
        next_WP_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        prev_WP_loc = current_loc;
    }

    // zero out our loiter vals to watch for missed waypoints
    loiter_angle_reset();

    setup_alt_slope();
    setup_turn_angle();

    // update plane.target_altitude straight away, or if we are too
    // close to out loiter point we may decide we are at the correct
    // altitude before updating it (this is based on scheduler table
    // ordering, where we navigate() before we
    // adjust_altitude_target(), and navigate() uses values updated in
    // adjust_altitude_target()
    adjust_altitude_target();
}

void Plane::set_guided_WP(const Location &loc)
{
    if (aparm.loiter_radius < 0 || loc.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP_loc = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP_loc = loc;

    fix_terrain_WP(next_WP_loc, __LINE__);

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    set_target_altitude_current();

    setup_alt_slope();
    setup_turn_angle();

    // disable crosstrack, head directly to the point
    auto_state.crosstrack = false;

    // reset loiter start time.
    loiter.start_time_ms = 0;

    // start in non-VTOL mode
    auto_state.vtol_loiter = false;
    
    loiter_angle_reset();

#if HAL_QUADPLANE_ENABLED
    // cancel pending takeoff
    quadplane.guided_takeoff = false;
#endif
}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed

  returns true if home is changed
*/
bool Plane::update_home()
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if ((g2.home_reset_threshold == -1) ||
        ((g2.home_reset_threshold > 0) &&
         (fabsf(barometer.get_altitude()) > g2.home_reset_threshold))) {
        // don't auto-update if we have changed barometer altitude
        // significantly. This allows us to cope with slow baro drift
        // but not re-do home and the baro if we have changed height
        // significantly
        return false;
    }
    bool ret = false;
    if (ahrs.home_is_set() && !ahrs.home_is_locked() && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        Location loc;
        if (ahrs.get_location(loc)) {
            // we take the altitude directly from the GPS as we are
            // about to reset the baro calibration. We can't use AHRS
            // altitude or we can end up perpetuating a bias in
            // altitude, as AHRS alt depends on home alt, which means
            // we would have a circular dependency
            loc.set_alt_cm(gps.location().alt,
                           Location::AltFrame::ABSOLUTE);
            ret = AP::ahrs().set_home(loc);
        }
    }

    // even if home is not updated we do a baro reset to stop baro
    // drift errors while disarmed
    barometer.update_calibration();
    ahrs.resetHeightDatum();

    update_current_loc();

    return ret;
}

bool Plane::set_home_persistently(const Location &loc)
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }

    return true;
}
