#include "mode.h"
#include "Plane.h"

bool ModeCruise::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = false;
    plane.cruise_state.locked_heading = false;
    plane.cruise_state.lock_timer_ms = 0;

    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();

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
        plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
        plane.update_load_factor();
    } else {
        plane.calc_nav_roll();
    }
    plane.update_fbwb_speed_height();
}

