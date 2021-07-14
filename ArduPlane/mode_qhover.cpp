#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQHover::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQHover::update()
{
    plane.mode_qstabilize.update();
}

#endif
