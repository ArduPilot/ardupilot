#include "mode.h"
#include "Plane.h"

bool ModeQStabilize::_enter()
{
    return q_enter();
}

void ModeQStabilize::update()
{
    q_update();
}

