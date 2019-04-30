#include "mode.h"
#include "Plane.h"

bool ModeQAcro::_enter()
{
    //return false;
    return plane.mode_qstabilize._enter();
}

void ModeQAcro::update()
{
    plane.mode_qstabilize.update();
}

