#include "mode.h"
#include "Plane.h"

bool ModeQLoiter::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQLoiter::update()
{
    plane.mode_qstabilize.update();
}


