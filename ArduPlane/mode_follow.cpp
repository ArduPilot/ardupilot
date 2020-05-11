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

    last_wp_change_ms = AP_HAL::millis();

    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::_exit()
{
    plane.g2.follow.clear_offsets_if_required();
}

void ModeFollow::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_wp_change_ms > 1000) {

        // if we change wp too fast the nav controller gets angry
        Location loc_next;
        if (get_follow_wp(loc_next) && plane.guided_WP_loc.same_latlngalt_as(loc_next)) {
            plane.guided_WP_loc = loc_next;
            gcs().send_text(MAV_SEVERITY_DEBUG, "ModeFollow::new (%.5f, %.5f, %d)", (double)plane.guided_WP_loc.lat*1e-7f, (double)plane.guided_WP_loc.lng*1e-7f, static_cast<int>(plane.guided_WP_loc.alt * 0.01f));

            last_wp_change_ms = now_ms;
            plane.set_guided_WP();
        }
    }

    plane.mode_guided.update();
}

// returns true if
bool ModeFollow::get_follow_wp(Location &loc_next)
{
    Location loc;
    Vector3f vel_ned;
    // this will get us the waypoint with the offset and radius applied
    if (plane.g2.follow.get_target_location_and_velocity(loc, vel_ned)) {
        loc_next.lat = loc.lat;
        loc_next.lng = loc.lng;
        loc_next.alt = plane.current_loc.alt;
        return true;
    }

    loc_next = plane.current_loc;
    return false;
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
