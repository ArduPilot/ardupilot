#include "mode.h"
#include "Plane.h"

bool ModeQRTL::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQRTL::update()
{
    plane.mode_qstabilize.update();
}

