#include "mode.h"
#include "Plane.h"

bool ModeLoiter::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.do_loiter_at_location();

    if (plane.g2.soaring_controller.is_active() && plane.g2.soaring_controller.suppress_throttle()) {
        plane.g2.soaring_controller.init_thermalling();
        plane.g2.soaring_controller.get_target(plane.next_WP_loc); // ahead on flight path
    }

    return true;
}

void ModeLoiter::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

