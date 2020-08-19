#include "mode.h"
#include "Plane.h"

bool ModeCruise::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = false;
    plane.cruise_state.locked_heading = false;
    plane.cruise_state.lock_timer_ms = 0;

#if SOARING_ENABLED == ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

void ModeCruise::update()
{
    /*
      in CRUISE mode we use the navigation code to control
      roll when heading is locked. Heading becomes unlocked on
      any aileron or rudder input
    */
    if (plane.channel_roll->get_control_in() != 0 || plane.channel_rudder->get_control_in() != 0) {
        plane.cruise_state.locked_heading = false;
        plane.cruise_state.lock_timer_ms = 0;
    }

    if (!plane.cruise_state.locked_heading) {
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
    } else {
        plane.calc_nav_roll();
    }
    plane.update_fbwb_speed_height();
}

/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
void ModeCruise::navigate()
{
    if (!plane.cruise_state.locked_heading &&
        plane.channel_roll->get_control_in() == 0 &&
        plane.rudder_input() == 0 &&
        plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        plane.gps.ground_speed() >= 3 &&
        plane.cruise_state.lock_timer_ms == 0) {
        // user wants to lock the heading - start the timer
        plane.cruise_state.lock_timer_ms = millis();
    }
    if (plane.cruise_state.lock_timer_ms != 0 &&
        (millis() - plane.cruise_state.lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        plane.cruise_state.locked_heading = true;
        plane.cruise_state.lock_timer_ms = 0;
        plane.cruise_state.locked_heading_cd = plane.gps.ground_course_cd();
        plane.prev_WP_loc = plane.current_loc;
    }
    if (plane.cruise_state.locked_heading) {
        plane.next_WP_loc = plane.prev_WP_loc;
        // always look 1km ahead
        plane.next_WP_loc.offset_bearing(plane.cruise_state.locked_heading_cd*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    }
}

