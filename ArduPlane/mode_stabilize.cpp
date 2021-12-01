#include "mode.h"
#include "Plane.h"

void ModeStabilize::update()
{
    plane.nav_roll_cd = plane.ahrs.roll_sensor;
    plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
}

