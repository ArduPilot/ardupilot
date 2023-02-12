#include "mode.h"
#include "Plane.h"

bool ModeAcro::_enter()
{
    acro_state.locked_roll = false;
    acro_state.locked_pitch = false;
    IGNORE_RETURN(ahrs.get_quaternion(acro_state.q));
    return true;
}

void ModeAcro::update()
{
    // handle locked/unlocked control
    if (acro_state.locked_roll) {
        plane.nav_roll_cd = acro_state.locked_roll_err;
    } else {
        plane.nav_roll_cd = ahrs.roll_sensor;
    }
    if (acro_state.locked_pitch) {
        plane.nav_pitch_cd = acro_state.locked_pitch_cd;
    } else {
        plane.nav_pitch_cd = ahrs.pitch_sensor;
    }
}

