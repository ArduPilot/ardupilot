#include "Plane.h"

#if MODE_FOLLOW_ENABLED == ENABLED

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id or gimbal target
 *
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AC_AVOID_ENABLED is true because we rely on it velocity limiting functions
 */

// initialise follow mode
bool ModeFollow::_enter()
{
    if (!plane.g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

    // re-use guided mode
    if (!plane.mode_guided._enter()) {
       return false;
    }

    update_follow_wp();

    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::_exit()
{
    plane.g2.follow.clear_offsets_if_required();
}

void ModeFollow::update()
{
    plane.mode_guided.update();
    update_follow_wp();
}

void ModeFollow::update_follow_wp()
{
    Location loc;
    Vector3f vel_ned;
    // this will get us the waypoint with the offset and radius applied
    if (plane.g2.follow.get_target_location_and_velocity(loc, vel_ned)) {
        plane.guided_WP_loc.lat = loc.lat;
        plane.guided_WP_loc.lng = loc.lng;
        plane.guided_WP_loc.alt = plane.current_loc.alt;
    } else {
        plane.guided_WP_loc = plane.current_loc;
    }

    plane.set_guided_WP();
}


uint32_t ModeFollow::wp_distance_cm() const
{
    return plane.g2.follow.get_distance_to_target() * 100;
}

int32_t ModeFollow::wp_bearing_cd() const
{
    return plane.g2.follow.get_bearing_to_target() * 100;
}

/*
  get target position for mavlink reporting
 */
bool ModeFollow::get_wp(Location &loc)
{
    float dist = plane.g2.follow.get_distance_to_target();
    float bearing = plane.g2.follow.get_bearing_to_target();
    loc = plane.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_FOLLOW_ENABLED == ENABLED
