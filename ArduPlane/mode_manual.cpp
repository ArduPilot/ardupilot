#include "mode.h"
#include "Plane.h"

bool ModeManual::_enter()
{
    return true;
}

void ModeManual::_exit()
{
    if (plane.g.auto_trim > 0) {
        plane.trim_radio();
    }
}
