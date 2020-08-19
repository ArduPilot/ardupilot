#include "mode.h"
#include "Plane.h"

bool ModeAvoidADSB::_enter()
{
    return plane.mode_guided.enter();
}

void ModeAvoidADSB::update()
{
    plane.mode_guided.update();
}

