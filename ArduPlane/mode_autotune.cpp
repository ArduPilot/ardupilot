#include "mode.h"
#include "Plane.h"

bool ModeAutoTune::_enter()
{
    plane.autotune_start();

    return true;
}


void ModeAutoTune::update()
{
    plane.mode_fbwa.update();
}

