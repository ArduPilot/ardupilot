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

#endif // HAL_ADSB_ENABLED

