#include "mode.h"
#include "Plane.h"

bool ModeQAcro::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_navigation_mode = false;
    if (!plane.quadplane.init_mode() && plane.previous_mode != nullptr) {
        plane.control_mode = plane.previous_mode;
    } else {
        plane.auto_throttle_mode = false;
        plane.auto_state.vtol_mode = true;
    }

    return true;
}

void ModeQAcro::update()
{
    // get nav_roll and nav_pitch from multicopter attitude controller
    Vector3f att_target = plane.quadplane.attitude_control->get_att_target_euler_cd();
    plane.nav_pitch_cd = att_target.y;
    plane.nav_roll_cd = att_target.x;
    return;
}

