#include "mode.h"
#include "Plane.h"

bool ModeQSFTHR::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQSFTHR::update()
{
    return plane.mode_qstabilize.update();
}
