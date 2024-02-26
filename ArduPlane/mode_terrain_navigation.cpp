#include "mode.h"
#include "Plane.h"

#define ENABLE_NPFG_CONTROLLER 0

bool ModeTerrainNavigation::_enter()
{
#if ENABLE_NPFG_CONTROLLER
    // switch to NPFG nav controller
    plane.nav_controller = &plane.NPFG_controller;
#endif
    return ModeGuided::_enter();
}

void ModeTerrainNavigation::_exit()
{
#if ENABLE_NPFG_CONTROLLER
    // restore default nav controller
    plane.nav_controller = &plane.L1_controller;
#endif
    ModeGuided::_exit();
}

void ModeTerrainNavigation::update()
{
    ModeGuided::update();
}

void ModeTerrainNavigation::navigate()
{
    plane.nav_controller->update_path(plane.next_WP_loc, _unit_path_tangent,
        _curvature, plane.loiter.direction);
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
    _curvature = 0.0;
    if (!is_zero(radius)) {
        _curvature = 1.0 / radius;
    }
    ModeGuided::set_radius_and_direction(radius, direction_is_ccw);
}

void ModeTerrainNavigation::set_path_tangent(Vector2f unit_path_tangent) {
    _unit_path_tangent = unit_path_tangent;
}

void ModeTerrainNavigation::update_target_altitude()
{
    ModeGuided::update_target_altitude();
}
