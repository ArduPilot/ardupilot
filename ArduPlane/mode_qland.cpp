#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLand::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQLand::update()
{
    plane.mode_qstabilize.update();
}

#endif
