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

void ModeAutoTune::run()
{
    // Run base class function and then output throttle
    Mode::run();

    output_pilot_throttle();
}
