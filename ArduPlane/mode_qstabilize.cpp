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
    // set nav_roll and nav_pitch using sticks
    // Beware that QuadPlane::tailsitter_check_input (called from Plane::read_radio)
    // may alter the control_in values for roll and yaw, but not the corresponding
    // radio_in values. This means that the results for norm_input would not necessarily
    // be correct for tailsitters, so get_control_in() must be used instead.
    // normalize control_input to [-1,1]
    const float roll_input = (float)plane.channel_roll->get_control_in() / plane.channel_roll->get_range();
    const float pitch_input = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();

    // then scale to target angles in centidegrees
    if (plane.quadplane.tailsitter.active()) {
        // tailsitters are different
        set_tailsitter_roll_pitch(roll_input, pitch_input);
        return;
    }

    if (!plane.quadplane.option_is_set(QuadPlane::OPTION::INGORE_FW_ANGLE_LIMITS_IN_Q_MODES)) {
        // by default angles are also constrained by forward flight limits
        set_limited_roll_pitch(roll_input, pitch_input);
    } else {
        // use angle max for both roll and pitch
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
        plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
    }
}

// quadplane stabilize mode
void ModeQStabilize::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    // special check for ESC calibration in QSTABILIZE
    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }

    // normal QSTABILIZE mode
    float pilot_throttle_scaled = quadplane.get_pilot_throttle();
    quadplane.hold_stabilize(pilot_throttle_scaled);

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();
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

    plane.quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
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

#endif
