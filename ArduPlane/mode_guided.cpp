#include "mode.h"
#include "Plane.h"

bool ModeGuided::_enter()
{
    return guided_enter();
}

void ModeGuided::update()
{
    guided_update();
}

