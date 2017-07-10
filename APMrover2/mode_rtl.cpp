#include "mode.h"
#include "Rover.h"

bool ModeRTL::_enter()
{
    rover.do_RTL();
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
    if (rover.verify_nav_wp()) {
        g2.motors.set_throttle(g.throttle_min.get());
        rover.set_mode(rover.mode_hold);
    } else {
        calc_lateral_acceleration();
        calc_nav_steer();
    }
}
