#include "mode.h"
#include "Plane.h"

bool ModeQStabilize::_enter()
{
    if (!plane.quadplane.init_mode() && plane.previous_mode != nullptr) {
        plane.control_mode = plane.previous_mode;
    } else {
        plane.auto_state.vtol_mode = true;
    }

    return true;
}

void ModeQStabilize::update()
{
    // set nav_roll and nav_pitch using sticks
    // Scale from normalized input [-1,1] to target angles in centidegrees
    const float roll_input = plane.channel_roll->norm_input();
    const float pitch_input = plane.channel_pitch->norm_input();

    if (plane.quadplane.tailsitter_active()) {
        // tailsitters are different
        set_tailsitter_roll_pitch(roll_input, pitch_input);
        return;
    }

    if ((plane.quadplane.options & QuadPlane::OPTION_INGORE_FW_ANGLE_LIMITS_IN_Q_MODES) == 0) {
        // by default angles are also constrained by forward flight limits
        set_limited_roll_pitch(roll_input, pitch_input);
    } else {
        // use angle max for both roll and pitch
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
        plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
    }
}

// set the desired roll and pitch for a tailsitter
void ModeQStabilize::set_tailsitter_roll_pitch(const float roll_input, const float pitch_input)
{
    // separate limit for roll, if set
    if (plane.quadplane.tailsitter.max_roll_angle > 0) {
        // roll param is in degrees not centidegrees
        plane.nav_roll_cd = plane.quadplane.tailsitter.max_roll_angle * 100.0f * roll_input;
    } else {
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
    }

    // angle max for tailsitter pitch
    plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
}

// set the desired roll and pitch for normal quadplanes, also limited by forward flight limtis
void ModeQStabilize::set_limited_roll_pitch(const float roll_input, const float pitch_input)
{
    plane.nav_roll_cd = roll_input * MIN(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);
    // pitch is further constrained by LIM_PITCH_MIN/MAX which may impose
    // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max_cd, plane.quadplane.aparm.angle_max);
    } else {
        plane.nav_pitch_cd = pitch_input * MIN(-plane.pitch_limit_min_cd, plane.quadplane.aparm.angle_max);
    }
}
