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
    Location loc;
    Vector3f vel_ned;
    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // lead vehicle velocity
    static uint32_t last_notify;
    if (plane.g2.follow.get_target_location_and_velocity(loc, vel_ned)) {
        plane.guided_WP_loc = loc;
        if (plane.g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {
        // convert dist_vec_offs to cm in NEU
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);
        const uint32_t now = AP_HAL::millis();
        if (now - last_notify > 1000) {
            last_notify = now;
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
    plane.update_loiter(3);    // Loiter radius of 3 prevents weaving and position error from target path
}
