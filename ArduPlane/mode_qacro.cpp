#include "mode.h"
#include "Plane.h"

bool ModeQAcro::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQAcro::update()
{
    // get nav_roll and nav_pitch from multicopter attitude controller
    Vector3f att_target = plane.quadplane.attitude_control->get_att_target_euler_cd();
    plane.nav_pitch_cd = att_target.y;
    plane.nav_roll_cd = att_target.x;
    return;
}

