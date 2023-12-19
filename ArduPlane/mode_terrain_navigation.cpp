#include "mode.h"
#include "Plane.h"

bool ModeTerrainNavigation::_enter()
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "Entering ModeTerrainNavigation");
    return ModeGuided::_enter();
}

void ModeTerrainNavigation::update()
{
    ModeGuided::update();
}

void ModeTerrainNavigation::navigate()
{
    ModeGuided::navigate();
}

bool ModeTerrainNavigation::handle_guided_request(Location target_loc)
{
    return ModeGuided::handle_guided_request(target_loc);
}

void ModeTerrainNavigation::set_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    ModeGuided::set_radius_and_direction(radius, direction_is_ccw);
}

void ModeTerrainNavigation::update_target_altitude()
{
    ModeGuided::update_target_altitude();
}
