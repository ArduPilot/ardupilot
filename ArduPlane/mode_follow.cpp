#include "mode.h"
#include "Plane.h"

#include <AP_Follow/AP_Follow.h>

bool ModeFollow::_enter()
{
    follow_sysid = 0;

    plane.guided_WP_loc = plane.current_loc;
    plane.set_guided_WP();

    return true;
}

void ModeFollow::update()
{
//    gcs().send_text(MAV_SEVERITY_INFO, "follow update %u", millis());
    Location loc;
    Vector3f vel_ned;
    static uint32_t last_notify;
    if (plane.g2.follow.get_target_location_and_velocity(loc, vel_ned)) {
        plane.guided_WP_loc = loc;
        const uint32_t now = AP_HAL::millis();
        if (now - last_notify > 1000) {
            last_notify = now;
            gcs().send_text(MAV_SEVERITY_INFO, "%u %u\n", loc.lat, loc.lng);
        }
        plane.set_guided_WP();
    }

    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeFollow::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}
