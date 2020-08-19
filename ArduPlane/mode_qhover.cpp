#include "mode.h"
#include "Plane.h"

bool ModeQHover::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQHover::update()
{
    plane.mode_qstabilize.update();
}


