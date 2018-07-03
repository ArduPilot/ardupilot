#include "mode.h"
#include "Plane.h"

bool ModeQHover::_enter()
{
    return q_enter();
}

void ModeQHover::update()
{
    q_update();
}


