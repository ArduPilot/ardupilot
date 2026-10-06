#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_ENABLED

#include "AC_AttitudeControl_Multi_6DoF.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// 6DoF control is extracted from the existing copter code by treating desired angles as thrust angles rather than vehicle attitude.
// Vehicle attitude is then set separately, typically the vehicle would maintain 0 roll and pitch.
// rate commands result in the vehicle behaving as a ordinary copter.

// run lowest level body-frame rate controller and send outputs to the motors
void AC_AttitudeControl_Multi_6DoF::rate_controller_run() {

    // pass current offsets to motors and run baseclass controller
    // motors require the offsets to know which way is up
    float roll_deg = roll_offset_deg;
    float pitch_deg = pitch_offset_deg;
    // if 6DoF control, always point directly up
    // this stops horizontal drift due to error between target and true attitude
    if (lateral_enable) {
        roll_deg = AP::ahrs().get_roll_deg();
    }
    if (forward_enable) {
        pitch_deg = AP::ahrs().get_pitch_deg();
    }
    _motors.set_roll_pitch(roll_deg,pitch_deg);

    AC_AttitudeControl_Multi::rate_controller_run();
}

/*
    override all input to the attitude controller and convert desired angles into thrust angles and substitute
*/

// Sets desired roll and pitch angles (in radians) and yaw rate (in radians/s).
// Used when roll/pitch stabilization is needed with manual or autonomous yaw rate control.
// Applies acceleration-limited input shaping for smooth transitions and computes body-frame angular velocity targets.
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_euler_rate_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads) {

    set_forward_lateral_rad(euler_pitch_angle_rad, euler_roll_angle_rad);

    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_euler_rate_yaw_rad(euler_roll_angle_rad, euler_pitch_angle_rad, euler_yaw_rate_rads);
}


// Sets desired roll, pitch, and yaw angles (in radians).
// Used to follow an absolute attitude setpoint. Input shaping and yaw slew limits are applied.
// Outputs are passed to the rate controller via shaped angular velocity targets.
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_angle_rad, bool slew_yaw) {

    set_forward_lateral_rad(euler_pitch_angle_rad, euler_roll_angle_rad);

    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_yaw_rad(euler_roll_angle_rad, euler_pitch_angle_rad, euler_yaw_angle_rad, slew_yaw);
}

// Sets desired thrust vector and heading rate (in radians/s).
// Used for tilt-based navigation with independent yaw control.
// The thrust vector defines the desired orientation (e.g., pointing direction for vertical thrust),
// while the heading rate adjusts yaw. The input is shaped by acceleration and slew limits.
void AC_AttitudeControl_Multi_6DoF::input_thrust_vector_rate_heading_rads(const Vector3f& thrust_vector, float heading_rate_rads, bool slew_yaw)
{
    // convert thrust vector to a roll and pitch angles
    // this negates the advantage of using thrust vector control, but works just fine
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    input_euler_angle_roll_pitch_euler_rate_yaw_rad(angle_target.x, angle_target.y, heading_rate_rads);
}

// Sets desired thrust vector and heading (in radians) with heading rate (in radians/s).
// Used for advanced attitude control where thrust direction is separated from yaw orientation.
// Heading slew is constrained based on configured limits.
void AC_AttitudeControl_Multi_6DoF::input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_angle_rad, float heading_rate_rads)
{
    // convert thrust vector to a roll and pitch angles
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    // note that we are throwing away heading rate here
    input_euler_angle_roll_pitch_yaw_rad(angle_target.x, angle_target.y, heading_angle_rad, true);
}

void AC_AttitudeControl_Multi_6DoF::set_forward_lateral_rad(float &euler_pitch_angle_rad, float &euler_roll_angle_rad)
{
    // pitch/forward
    if (forward_enable) {
        _motors.set_forward(-sinf(euler_pitch_angle_rad));
        euler_pitch_angle_rad = radians(pitch_offset_deg);
    } else {
        _motors.set_forward(0.0f);
        euler_pitch_angle_rad += radians(pitch_offset_deg);
    }
    euler_pitch_angle_rad = wrap_PI(euler_pitch_angle_rad);

    // roll/lateral
    if (lateral_enable) {
        _motors.set_lateral(sinf(euler_roll_angle_rad));
        euler_roll_angle_rad = radians(roll_offset_deg);
    } else {
        _motors.set_lateral(0.0f);
        euler_roll_angle_rad += radians(roll_offset_deg);
    }
    euler_roll_angle_rad = wrap_PI(euler_roll_angle_rad);
}

/*
    all other input functions should zero thrust vectoring
*/

// Command euler yaw rate and pitch angle with roll angle specified in body frame
// (used only by tailsitter quadplanes)
void AC_AttitudeControl_Multi_6DoF::input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(bool plane_controls, float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(plane_controls, euler_roll_angle_rad, euler_pitch_angle_rad, euler_yaw_rate_rads);
}

// Sets desired roll, pitch, and yaw angular rates (in radians/s).
// This command is used to apply angular rate targets in the earth frame.
// The inputs are shaped using acceleration limits and time constants.
// Resulting targets are converted into body-frame angular velocities
// and passed to the rate controller.
void AC_AttitudeControl_Multi_6DoF::input_euler_rate_roll_pitch_yaw_rads(float euler_roll_rate_rads, float euler_pitch_rate_rads, float euler_yaw_rate_rads) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_euler_rate_roll_pitch_yaw_rads(euler_roll_rate_rads, euler_pitch_rate_rads, euler_yaw_rate_rads);
}


// Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
// This command is used by fully stabilized acro modes.
// It applies angular velocity targets in the body frame,
// shaped using acceleration limits and passed to the rate controller.
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_rads(roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads);
}

// Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
// Used by Copter's rate-only acro mode.
// Applies raw angular velocity targets directly to the rate controller with smoothing
// and no attitude feedback or stabilization.
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw_2_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_2_rads(roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads);
}

// Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
// Used by Plane's acro mode with rate error integration.
// Integrates attitude error over time to generate target angular rates.
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw_3_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_3_rads(roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads);
}

// Applies a one-time angular offset in body-frame roll/pitch/yaw angles (in radians).
// Used for initiating step responses during autotuning or manual test inputs.
void AC_AttitudeControl_Multi_6DoF::input_angle_step_bf_roll_pitch_yaw_rad(float roll_angle_step_bf_rad, float pitch_angle_step_bf_rad, float yaw_angle_step_bf_rad) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_angle_step_bf_roll_pitch_yaw_rad(roll_angle_step_bf_rad, pitch_angle_step_bf_rad, yaw_angle_step_bf_rad);
}

// Sets a desired attitude using a quaternion and body-frame angular velocity (rad/s).
// The desired quaternion is incrementally updated each timestep. Angular velocity is shaped by acceleration limits and feedforward.
void AC_AttitudeControl_Multi_6DoF::input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_body) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_HAL::panic("input_quaternion not implemented AC_AttitudeControl_Multi_6DoF");
#endif

    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_quaternion(attitude_desired_quat, ang_vel_body);
}


AC_AttitudeControl_Multi_6DoF *AC_AttitudeControl_Multi_6DoF::_singleton = nullptr;

#endif // AP_SCRIPTING_ENABLED
