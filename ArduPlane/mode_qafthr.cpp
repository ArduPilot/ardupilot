#include "mode.h"
#include "Plane.h"

bool ModeQAFTHR::_enter()
{
    return plane.mode_qacro._enter();
}

void ModeQAFTHR::update()
{
    return plane.mode_qacro.update();
}
