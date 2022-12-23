#include "mode.h"
#include "Plane.h"

#if HAL_ADSB_ENABLED

bool ModeAvoidADSB::_enter()
{
    return plane.mode_guided.enter();
}

void ModeAvoidADSB::update()
{
    plane.mode_guided.update();
}

void ModeAvoidADSB::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

bool ModeAvoidADSB::use_glide_slope() const
{
    return plane.above_location_current(plane.next_WP_loc);
}

#endif // HAL_ADSB_ENABLED

