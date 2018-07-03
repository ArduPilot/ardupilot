#include "mode.h"
#include "Plane.h"

bool ModeQLoiter::_enter()
{
    return q_enter();
}

void ModeQLoiter::update()
{
    q_update();
}


