#include "mode.h"
#include "Plane.h"

//! @note using ModeCruise as template 

bool ModeTerrainNavigation::_enter()
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "ModeTerrainNavigation::_enter");
    // return ModeGuided::_enter();

    // switch to NPFG nav controller
    plane.nav_controller = &plane.NPFG_controller;

    return true;
}

void ModeTerrainNavigation::_exit()
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "ModeTerrainNavigation::_exit");

    // restore default nav controller
    plane.nav_controller = &plane.L1_controller;
}

void ModeTerrainNavigation::update()
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "ModeTerrainNavigation::update");
    // ModeGuided::update();

    plane.calc_nav_roll();

    plane.calc_nav_pitch();

    // Throttle output

    // TECS control
    plane.calc_throttle();
}

void ModeTerrainNavigation::navigate()
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "ModeTerrainNavigation::navigate");
    // ModeGuided::navigate();

    plane.NPFG_controller.update_path_tangent(_unit_path_tangent);
    plane.nav_controller->update_loiter(plane.next_WP_loc, _radius_m,
        plane.loiter.direction);
}

bool ModeTerrainNavigation::handle_guided_request(Location target_loc)
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "ModeTerrainNavigation::handle_guided_request");
    // return ModeGuided::handle_guided_request(target_loc);

    // add home alt if needed
    if (target_loc.relative_alt) {
        target_loc.alt += plane.home.alt;
        target_loc.relative_alt = 0;
    }

    // see: Plane::set_guided_WP

    // copy the current location into the OldWP slot
    plane.prev_WP_loc = plane.current_loc;

    // Load the next_WP slot
    plane.next_WP_loc = target_loc;

    return true;
}

void ModeTerrainNavigation::set_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    // gcs().send_text(MAV_SEVERITY_DEBUG, "ModeTerrainNavigation::set_radius_and_direction");
    // ModeGuided::set_radius_and_direction(radius, direction_is_ccw);

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
