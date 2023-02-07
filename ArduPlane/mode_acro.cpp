#include "mode.h"
#include "Plane.h"

bool ModeAcro::_enter()
{
    plane.acro_state.locked_roll = false;
    plane.acro_state.locked_pitch = false;
    IGNORE_RETURN(plane.ahrs.get_quaternion(plane.acro_state.q));
    return true;
}

void ModeAcro::update()
{
    // handle locked/unlocked control
    if (plane.acro_state.locked_roll) {
        plane.nav_roll_cd = plane.acro_state.locked_roll_err;
    } else {
        plane.nav_roll_cd = plane.ahrs.roll_sensor;
    }
    if (plane.acro_state.locked_pitch) {
        plane.nav_pitch_cd = plane.acro_state.locked_pitch_cd;
    } else {
        plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
    }
}

