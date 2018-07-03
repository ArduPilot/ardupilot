#include "mode.h"
#include "Plane.h"

bool ModeAvoidADSB::_enter()
{
    return guided_enter();
}

void ModeAvoidADSB::update()
{
    guided_update();
}

