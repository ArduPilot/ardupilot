#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQStabilize::_enter()
{
    quadplane.throttle_wait = false;
    return true;
}

void ModeQStabilize::update()
{
    // fetch normalized roll and pitch inputs
    // Beware that QuadPlane::tailsitter_check_input (called from Plane::read_radio)
    // may alter the control_in values for roll and yaw, but not the corresponding
    // radio_in values. This means that the results for norm_input would not necessarily
    // be correct for tailsitters, so get_control_in() must be used instead.
    const float roll_in_norm = (float)plane.channel_roll->get_control_in() / plane.channel_roll->get_range();
    const float pitch_in_norm = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();

    // convert roll and pitch inputs to Euler angles and scale both to angle_max
    float target_roll_deg, target_pitch_deg;
    const float angle_max_deg = plane.quadplane.aparm.angle_max * .01f;
    rc_input_to_roll_pitch(roll_in_norm, pitch_in_norm, angle_max_deg, angle_max_deg, target_roll_deg, target_pitch_deg);

    if (plane.quadplane.tailsitter.active()) {
        // apply tailsitter roll and pitch limits
        set_tailsitter_roll_pitch(target_roll_deg, target_pitch_deg);
        return;
    }

    if ((plane.quadplane.options & QuadPlane::OPTION_IGNORE_FW_ANGLE_LIMITS_IN_Q_MODES) == 0) {
        // by default angles are also constrained by forward flight limits
        set_limited_roll_pitch(target_roll_deg, target_pitch_deg);
    } else {
        // use angle max for both roll and pitch
        plane.nav_roll_cd = target_roll_deg * 100;
        plane.nav_pitch_cd = target_pitch_deg * 100;
    }
}

// quadplane stabilize mode
void ModeQStabilize::run()
{
    // special check for ESC calibration in QSTABILIZE
    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        return;
    }

    // normal QSTABILIZE mode
    float pilot_throttle_scaled = quadplane.get_pilot_throttle();
    quadplane.hold_stabilize(pilot_throttle_scaled);
}

// set the desired roll and pitch for a tailsitter
void ModeQStabilize::set_tailsitter_roll_pitch(const float roll_input, const float pitch_input)
{
    // separate limit for roll, if set
    plane.nav_roll_cd = 100 * roll_input;
    if (plane.quadplane.tailsitter.max_roll_angle > 0) {
        // roll param is in degrees not centidegrees
        plane.nav_roll_cd *= 100 * plane.quadplane.tailsitter.max_roll_angle / plane.quadplane.aparm.angle_max;
    }

    // angle max for tailsitter pitch
    plane.nav_pitch_cd = 100 * pitch_input;

    plane.quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
}

// set the desired roll and pitch for normal quadplanes, also limited by forward flight limtis
void ModeQStabilize::set_limited_roll_pitch(const float roll_input, const float pitch_input)
{
    plane.nav_roll_cd = 100 * roll_input;
    if (plane.roll_limit_cd < plane.quadplane.aparm.angle_max) {
        plane.nav_roll_cd *= (float)plane.roll_limit_cd / plane.quadplane.aparm.angle_max;
    }
    // pitch is further constrained by LIM_PITCH_MIN/MAX which may impose
    // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
    plane.nav_pitch_cd = 100 * pitch_input;
    if (pitch_input > 0) {
        if (plane.aparm.pitch_limit_max_cd < plane.quadplane.aparm.angle_max) {
            plane.nav_pitch_cd *= (float)plane.aparm.pitch_limit_max_cd / plane.quadplane.aparm.angle_max;
        }
    } else {
        if (plane.aparm.pitch_limit_min_cd > -plane.quadplane.aparm.angle_max) {
            plane.nav_pitch_cd *= fabsf(plane.aparm.pitch_limit_min_cd) / plane.quadplane.aparm.angle_max;
        }
    }
}

#endif
