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
    plane.mode_guided.navigate();
}

#endif // HAL_ADSB_ENABLED

