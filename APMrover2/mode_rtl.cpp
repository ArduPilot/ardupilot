#include "mode.h"
#include "Rover.h"

bool ModeRTL::_enter()
{
    rover.prev_WP = rover.current_loc;
    rover.next_WP = rover.home;
    g2.motors.slew_limit_throttle(true);
    return true;
}

void ModeRTL::update()
{
    if (!rover.in_auto_reverse) {
        rover.set_reverse(false);
    }
    calc_lateral_acceleration();
    calc_nav_steer();
    calc_throttle(g.speed_cruise);
}

void ModeRTL::update_navigation()
{
    // no loitering around the wp with the rover, goes direct to the wp position
    if (rover.verify_RTL()) {
        rover.set_mode(rover.mode_hold);
    }
}
