#include "mode.h"
#include "Plane.h"

#if HAL_SOARING_ENABLED

bool ModeThermal::_enter()
{
    if (!plane.g2.soaring_controller.is_active()) {
        return false;
    }

    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.do_loiter_at_location();
    plane.loiter_angle_reset();

    plane.g2.soaring_controller.init_thermalling();
    plane.g2.soaring_controller.get_target(plane.next_WP_loc); // ahead on flight path

    return true;
}

void ModeThermal::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeThermal::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

#endif
