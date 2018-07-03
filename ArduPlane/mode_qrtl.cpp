#include "mode.h"
#include "Plane.h"

bool ModeQRTL::_enter()
{
    return q_enter();
}

void ModeQRTL::update()
{
    q_update();
}

