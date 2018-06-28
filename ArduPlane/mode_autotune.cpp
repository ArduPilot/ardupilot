#include "mode.h"
#include "Plane.h"

bool ModeAutoTune::_enter()
{
    return true;
}

void ModeAutoTune::_exit()
{
    // restore last gains
    plane.autotune_restore();
}
