#ifdef ENABLE_SCRIPTING

#include "AC_AttitudeControl_Multi_6DoF.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// 6DoF control is extracted from the existing copter code by treating desired angles as thrust angles rather than vehicle attitude.
// Vehicle attitude is then set separately, typically the vehicle would matain 0 roll and pitch.
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
        roll_deg = degrees(AP::ahrs().get_roll());
    }
    if (forward_enable) {
        pitch_deg = degrees(AP::ahrs().get_pitch());
    }
    _motors.set_roll_pitch(roll_deg,pitch_deg);

    AC_AttitudeControl_Multi::rate_controller_run();
}

/*
    override all input to the attitude controller and convert desired angles into thrust angles and substitute
*/

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) {

    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);

    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw) {

    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);

    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_angle_cd, slew_yaw);
}

// Command a thrust vector and heading rate
void AC_AttitudeControl_Multi_6DoF::input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds)
{
    // convert thrust vector to a roll and pitch angles
    // this negates the advantage of using thrust vector control, but works just fine
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    input_euler_angle_roll_pitch_euler_rate_yaw(degrees(angle_target.x) * 100.0f, degrees(angle_target.y) * 100.0f, heading_rate_cds);
}

// Command a thrust vector, heading and heading rate
void AC_AttitudeControl_Multi_6DoF::input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds)
{
    // convert thrust vector to a roll and pitch angles
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    // note that we are throwing away heading rate here
    input_euler_angle_roll_pitch_yaw(degrees(angle_target.x) * 100.0f, degrees(angle_target.y) * 100.0f, heading_angle_cd, true);
}

void AC_AttitudeControl_Multi_6DoF::set_forward_lateral(float &euler_pitch_angle_cd, float &euler_roll_angle_cd)
{
    // pitch/forward
    if (forward_enable) {
        _motors.set_forward(-sinf(radians(euler_pitch_angle_cd * 0.01f)));
        euler_pitch_angle_cd = pitch_offset_deg * 100.0f;
    } else {
        _motors.set_forward(0.0f);
        euler_pitch_angle_cd += pitch_offset_deg * 100.0f;
    }
    euler_pitch_angle_cd = wrap_180_cd(euler_pitch_angle_cd);

    // roll/lateral
    if (lateral_enable) {
        _motors.set_lateral(sinf(radians(euler_roll_angle_cd * 0.01f)));
        euler_roll_angle_cd = roll_offset_deg * 100.0f;
    } else {
        _motors.set_lateral(0.0f);
        euler_roll_angle_cd += roll_offset_deg * 100.0f;
    }
    euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd);
}

/*
    all other input functions should zero thrust vectoring
*/

// Command euler yaw rate and pitch angle with roll angle specified in body frame
// (used only by tailsitter quadplanes)
void AC_AttitudeControl_Multi_6DoF::input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_euler_rate_yaw_euler_angle_pitch_bf_roll(plane_controls, euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_euler_rate_roll_pitch_yaw(euler_roll_rate_cds, euler_pitch_rate_cds, euler_yaw_rate_cds);
}

// Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_2(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_3(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// Command an angular step (i.e change) in body frame angle
void AC_AttitudeControl_Multi_6DoF::input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_angle_step_bf_roll_pitch_yaw(roll_angle_step_bf_cd, pitch_angle_step_bf_cd, yaw_angle_step_bf_cd);
}

// Command a Quaternion attitude with feedforward and smoothing
// not used anywhere in current code, panic in SITL so this implementaiton is not overlooked
void AC_AttitudeControl_Multi_6DoF::input_quaternion(Quaternion attitude_desired_quat) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_HAL::panic("input_quaternion not implemented AC_AttitudeControl_Multi_6DoF");
#endif

    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_quaternion(attitude_desired_quat);
}


AC_AttitudeControl_Multi_6DoF *AC_AttitudeControl_Multi_6DoF::_singleton = nullptr;

#endif // ENABLE_SCRIPTING
