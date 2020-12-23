#include "mode.h"
#include "Plane.h"

bool ModeInitializing::_enter()
{
    plane.auto_throttle_mode = true;

    return true;
}

