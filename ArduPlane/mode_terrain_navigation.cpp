#include "mode.h"
#include "Plane.h"

#define ENABLE_NPFG_CONTROLLER 0

bool ModeTerrainNavigation::_enter()
{
#if ENABLE_NPFG_CONTROLLER
    // switch to NPFG nav controller
    plane.nav_controller = &plane.NPFG_controller;
#else
#endif

    return true;
}

void ModeTerrainNavigation::_exit()
{
#if ENABLE_NPFG_CONTROLLER
    // restore default nav controller
    plane.nav_controller = &plane.L1_controller;
#else
#endif
}

void ModeTerrainNavigation::update()
{
    plane.calc_nav_roll();

    plane.calc_nav_pitch();

    plane.calc_throttle();
}

void ModeTerrainNavigation::navigate()
{
#if ENABLE_NPFG_CONTROLLER
    plane.NPFG_controller.set_path_tangent(_unit_path_tangent);
    plane.NPFG_controller.update_loiter(plane.next_WP_loc, _radius_m,
        plane.loiter.direction);
#else
    if (_radius_m > 0.0) {
        // moving along arc of circle - loiter about wp located at
        // centre of curvature.
        auto center_wp = plane.next_WP_loc;
        Vector3p tangent_ned(_unit_path_tangent.x, _unit_path_tangent.y, 0.0);
        Vector3p dn_ned(0.0, 0.0, 1.0); 
        auto ofs_ned = dn_ned.cross(tangent_ned)
            * _radius_m * plane.loiter.direction;
        center_wp.offset(ofs_ned);

        plane.nav_controller->update_loiter(center_wp, _radius_m,
            plane.loiter.direction);

        // notify when we switch mode
        if (_nav_mode == NAV_WAYPOINT) {
          gcs().send_text(MAV_SEVERITY_INFO,
              "NAV_LOITER: radius: %f, direction: %d", _radius_m,
              plane.loiter.direction);
        }
        _nav_mode = NAV_LOITER;
    } else {
        // moving along a line segment - navigate to wp ahead of closest point
        // in direction of path tangent 
        float ofs_m = 50.0;
        Vector3p ofs_ned(ofs_m * _unit_path_tangent.x,
            ofs_m * _unit_path_tangent.y, 0.0);
        auto prev_wp = plane.next_WP_loc;
        auto next_wp = plane.next_WP_loc;
        next_wp.offset(ofs_ned);

        plane.nav_controller->update_waypoint(prev_wp, next_wp);

        // notify when we switch mode
        if (_nav_mode == NAV_LOITER) {
          gcs().send_text(MAV_SEVERITY_INFO, "NAV_WAYPOINT: offset: %f", ofs_m);
        }
        _nav_mode = NAV_WAYPOINT;
    }

#endif
}

bool ModeTerrainNavigation::handle_guided_request(Location target_loc)
{
    // add home alt if needed
    if (target_loc.relative_alt) {
        target_loc.alt += plane.home.alt;
        target_loc.relative_alt = 0;
    }

    // copy the current location into the OldWP slot
    plane.prev_WP_loc = plane.current_loc;

    // Load the next_WP slot
    plane.next_WP_loc = target_loc;

    return true;
}

void ModeTerrainNavigation::set_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    _radius_m = radius;
    plane.loiter.direction = direction_is_ccw ? -1 : 1;
}

void ModeTerrainNavigation::set_path_tangent(Vector2f unit_path_tangent) {
    _unit_path_tangent = unit_path_tangent;
}

void ModeTerrainNavigation::update_target_altitude()
{
    // gcs().send_text(MAV_SEVERITY_DEBUG, "ModeTerrainNavigation::update_target_altitude");
    ModeGuided::update_target_altitude();
}
