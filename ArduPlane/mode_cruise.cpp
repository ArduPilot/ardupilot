#include "mode.h"
#include "Plane.h"

bool ModeCruise::_enter()
{
    locked_heading = false;
    lock_timer_ms = 0;

#if HAL_SOARING_ENABLED
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
        locked_heading = false;
        lock_timer_ms = 0;
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // while a trick is running unlock heading and zero altitude offset
        locked_heading = false;
        lock_timer_ms = 0;
        plane.set_target_altitude_current();
    }
#endif
    
    if (!locked_heading) {
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
#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // don't try to navigate while running trick
        return;
    }
#endif

    // check if we are moving in the direction of the front of the vehicle
    const int32_t ground_course_cd = plane.gps.ground_course_cd();
    const bool moving_forwards = fabsf(wrap_PI(cd_to_rad(ground_course_cd) - plane.ahrs.get_yaw_rad())) < M_PI_2;

    if (!locked_heading &&
        plane.channel_roll->get_control_in() == 0 &&
        plane.rudder_input() == 0 &&
        plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        plane.gps.ground_speed() >= GPS_GND_CRS_MIN_SPD &&
        moving_forwards &&
        lock_timer_ms == 0) {
        // user wants to lock the heading - start the timer
        lock_timer_ms = millis();
    }
    if (lock_timer_ms != 0 &&
        (millis() - lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        locked_heading = true;
        lock_timer_ms = 0;
        locked_heading_cd = ground_course_cd;
        plane.prev_WP_loc = plane.current_loc;
    }
    if (locked_heading) {
        plane.next_WP_loc = plane.prev_WP_loc;
        // always look 1km ahead
        plane.next_WP_loc.offset_bearing(locked_heading_cd*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    }
}

bool ModeCruise::get_target_heading_cd(int32_t &target_heading) const
{
    target_heading = locked_heading_cd;
    return locked_heading;
}
