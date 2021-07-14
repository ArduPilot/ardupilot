#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLoiter::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQLoiter::update()
{
    plane.mode_qstabilize.update();
}

#endif // HAL_QUADPLANE_ENABLED
