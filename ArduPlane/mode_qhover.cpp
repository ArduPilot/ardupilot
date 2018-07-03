#include "mode.h"
#include "Plane.h"

bool ModeQHover::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_navigation_mode = false;
    if (!plane.quadplane.init_mode() && plane.previous_mode != nullptr) {
        plane.control_mode = plane.previous_mode;
    } else {
        plane.auto_throttle_mode = false;
        plane.auto_state.vtol_mode = true;
    }

    return true;
}

void ModeQHover::update()
{
    // set nav_roll and nav_pitch using sticks
    int16_t roll_limit = MIN(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);
    plane.nav_roll_cd  = (plane.channel_roll->get_control_in() / 4500.0) * roll_limit;
    plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -roll_limit, roll_limit);
    float pitch_input = plane.channel_pitch->norm_input();
    // Scale from normalized input [-1,1] to centidegrees
    if (plane.quadplane.tailsitter_active()) {
        // For tailsitters, the pitch range is symmetrical: [-Q_ANGLE_MAX,Q_ANGLE_MAX]
        plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
    } else {
        // pitch is further constrained by LIM_PITCH_MIN/MAX which may impose
        // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
        if (pitch_input > 0) {
            plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max_cd, plane.quadplane.aparm.angle_max);
        } else {
            plane.nav_pitch_cd = pitch_input * MIN(-plane.pitch_limit_min_cd, plane.quadplane.aparm.angle_max);
        }
        plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
    }
}


