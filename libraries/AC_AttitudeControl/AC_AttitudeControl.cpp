#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // default gains for Plane
 # define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT  0.2f    // Soft
 #define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN     5.0     // Min lean angle so that vehicle can maintain limited control
 #define AC_ATTITUDE_CONTROL_AFTER_RATE_CONTROL 0
#else
 // default gains for Copter and Sub
 # define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT  0.15f   // Medium
 #define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN     10.0   // Min lean angle so that vehicle can maintain limited control
 #define AC_ATTITUDE_CONTROL_AFTER_RATE_CONTROL 1
#endif

// lean angle max (in degrees) for all vehicles
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MAX 80.0        // lean angle max in degrees

// default angle max for all vehicles
#ifndef AC_ATTITUDE_CONTROL_ANGLE_MAX_DEFAULT
 # define AC_ATTITUDE_CONTROL_ANGLE_MAX_DEFAULT 30.0f   // default max lean angle in degrees
#endif

AC_AttitudeControl *AC_AttitudeControl::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] = {

    // 0, 1 were RATE_RP_MAX, RATE_Y_MAX

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in RTL and Auto flight modes
    // @Units: cdeg/s
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW", 2, AC_AttitudeControl, _slew_yaw_cds, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS),

    // 3 was for ACCEL_RP_MAX

    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: cdeg/s/s
    // @Range: 0 72000
    // @Values: 0:Disabled, 9000:VerySlow, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX", 4, AC_AttitudeControl, _accel_yaw_max_cdss, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedforward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    // @Param: ACCEL_R_MAX
    // @DisplayName: Acceleration Max for Roll
    // @Description: Maximum acceleration in roll axis
    // @Units: cdeg/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_R_MAX", 6, AC_AttitudeControl, _accel_roll_max_cdss, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // @Param: ACCEL_P_MAX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: cdeg/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_P_MAX", 7, AC_AttitudeControl, _accel_pitch_max_cdss, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // IDs 8,9,10,11 RESERVED (in use on Solo)

    // @Param: ANGLE_BOOST
    // @DisplayName: Angle Boost
    // @Description: Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ANGLE_BOOST", 12, AC_AttitudeControl, _angle_boost_enabled, 1),

    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 13, AC_AttitudeControl, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 14, AC_AttitudeControl, AC_P),

    // @Param: ANG_YAW_P
    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 6.000
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_yaw, "ANG_YAW_", 15, AC_AttitudeControl, AC_P),

    // @Param: ANG_LIM_TC
    // @DisplayName: Angle Limit (to maintain altitude) Time Constant
    // @Description: Angle Limit (to maintain altitude) Time Constant
    // @Range: 0.5 10.0
    // @User: Advanced
    AP_GROUPINFO("ANG_LIM_TC", 16, AC_AttitudeControl, _angle_limit_tc, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT),

    // @Param: RATE_R_MAX
    // @DisplayName: Angular Velocity Max for Roll
    // @Description: Maximum angular velocity in roll axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_R_MAX", 17, AC_AttitudeControl, _ang_vel_roll_max_degs, 0.0f),

    // @Param: RATE_P_MAX
    // @DisplayName: Angular Velocity Max for Pitch
    // @Description: Maximum angular velocity in pitch axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_P_MAX", 18, AC_AttitudeControl, _ang_vel_pitch_max_degs, 0.0f),

    // @Param: RATE_Y_MAX
    // @DisplayName: Angular Velocity Max for Yaw
    // @Description: Maximum angular velocity in yaw axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_Y_MAX", 19, AC_AttitudeControl, _ang_vel_yaw_max_degs, 0.0f),

    // @Param: INPUT_TC
    // @DisplayName: Attitude control input time constant
    // @Description: Attitude control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_GROUPINFO("INPUT_TC", 20, AC_AttitudeControl, _input_tc, AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT),

    // @Param: LAND_R_MULT
    // @DisplayName: Landed roll gain multiplier
    // @Description: Roll gain multiplier active when landed. A factor of 1.0 means no reduction in gain while landed. Reduce this factor to reduce ground oscitation in the roll axis. 
    // @Range: 0.25 1.0
    // @User: Advanced
    AP_GROUPINFO("LAND_R_MULT", 21, AC_AttitudeControl, _land_roll_mult, 1.0),

    // @Param: LAND_P_MULT
    // @DisplayName: Landed pitch gain multiplier
    // @Description: Pitch gain multiplier active when landed. A factor of 1.0 means no reduction in gain while landed. Reduce this factor to reduce ground oscitation in the pitch axis. 
    // @Range: 0.25 1.0
    // @User: Advanced
    AP_GROUPINFO("LAND_P_MULT", 22, AC_AttitudeControl, _land_pitch_mult, 1.0),

    // @Param: LAND_Y_MULT
    // @DisplayName: Landed yaw gain multiplier
    // @Description: Yaw gain multiplier active when landed. A factor of 1.0 means no reduction in gain while landed. Reduce this factor to reduce ground oscitation in the yaw axis. 
    // @Range: 0.25 1.0
    // @User: Advanced
    AP_GROUPINFO("LAND_Y_MULT", 23, AC_AttitudeControl, _land_yaw_mult, 1.0),

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: deg
    // @Increment: 0.1
    // @Range: 10.0 80.0
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 24, AC_AttitudeControl, _angle_max_deg, AC_ATTITUDE_CONTROL_ANGLE_MAX_DEFAULT),

    AP_GROUPEND
};

constexpr Vector3f AC_AttitudeControl::VECTORF_111;

// get the slew yaw rate limit in rad/s
float AC_AttitudeControl::get_slew_yaw_max_rads() const
{
    if (!is_positive(_ang_vel_yaw_max_degs)) {
        return cd_to_rad(_slew_yaw_cds);
    }
    return MIN(radians(_ang_vel_yaw_max_degs), cd_to_rad(_slew_yaw_cds));
}

// get the latest gyro for the purposes of attitude control
// Counter-inuitively the lowest latency for rate control is achieved by running rate control
// *before* attitude control. This is because you want rate control to run as close as possible
// to the time that a gyro sample was read to minimise jitter and control errors. Running rate
// control after attitude control might makes sense logically, but the overhead of attitude
// control calculations (and other updates) compromises the actual rate control.
//
// In the case of running rate control in a separate thread, the ordering between rate and attitude
// updates is less important, except that gyro sample used should be the latest
//
// Currently quadplane runs rate control after attitude control, necessitating the following code
// to minimise latency.
// However this code can be removed once quadplane updates it's structure to run the rate loops before
// the Attitude controller.
const Vector3f AC_AttitudeControl::get_latest_gyro() const
{
#if AC_ATTITUDE_CONTROL_AFTER_RATE_CONTROL
    // rate updates happen before attitude updates so the last gyro value is the last rate gyro value
    // this also allows a separate rate thread to be the source of gyro data
    return _rate_gyro_rads;
#else
    // rate updates happen after attitude updates so the AHRS must be consulted for the last gyro value
    return _ahrs.get_gyro_latest();
#endif
}

// Ensure attitude controller have zero errors to relax rate controller output
void AC_AttitudeControl::relax_attitude_controllers()
{
    // take a copy of the last gyro used by the rate controller before using it
    Vector3f gyro = get_latest_gyro();
    // Initialize the attitude variables to the current attitude
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target_rad);
    _attitude_ang_error.initialise();

    // Initialize the angular rate variables to the current rate
    _ang_vel_target_rads = gyro;
    body_to_euler_derivative(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);

    // Initialize remaining variables
    _thrust_error_angle_rad = 0.0f;

    // Reset the PID filters
    get_rate_roll_pid().reset_filter();
    get_rate_pitch_pid().reset_filter();
    get_rate_yaw_pid().reset_filter();

    // Reset the I terms
    reset_rate_controller_I_terms();
    // finally update the attitude target
    _ang_vel_body_rads = gyro;
}

void AC_AttitudeControl::reset_rate_controller_I_terms()
{
    get_rate_roll_pid().reset_I();
    get_rate_pitch_pid().reset_I();
    get_rate_yaw_pid().reset_I();
}

// reset rate controller I terms smoothly to zero in 0.5 seconds
void AC_AttitudeControl::reset_rate_controller_I_terms_smoothly()
{
    get_rate_roll_pid().relax_integrator(0.0, _dt_s, AC_ATTITUDE_RATE_RELAX_TC);
    get_rate_pitch_pid().relax_integrator(0.0, _dt_s, AC_ATTITUDE_RATE_RELAX_TC);
    get_rate_yaw_pid().relax_integrator(0.0, _dt_s, AC_ATTITUDE_RATE_RELAX_TC);
}

// Reduce attitude control gains while landed to stop ground resonance
void AC_AttitudeControl::landed_gain_reduction(bool landed)
{
    if (is_positive(_input_tc)) {
        // use 2.0 x tc to match the response time to 86% commanded
        const float spool_step = _dt_s / (2.0 * _input_tc);
        if (landed) {
            _landed_gain_ratio = MIN(1.0, _landed_gain_ratio + spool_step);
        } else {
            _landed_gain_ratio = MAX(0.0, _landed_gain_ratio - spool_step);
        }
    } else {
        _landed_gain_ratio = landed ? 1.0 : 0.0;
    }
    Vector3f scale_mult = VECTORF_111 * (1.0 - _landed_gain_ratio) + Vector3f{_land_roll_mult, _land_pitch_mult, _land_yaw_mult} * _landed_gain_ratio;
    set_PD_scale_mult(scale_mult);
    set_angle_P_scale_mult(scale_mult);
}

// The attitude controller works around the concept of the desired attitude, target attitude
// and measured attitude. The desired attitude is the attitude input into the attitude controller
// that expresses where the higher level code would like the aircraft to move to. The target attitude is moved
// to the desired attitude with jerk, acceleration, and velocity limits. The target angular velocities are fed
// directly into the rate controllers. The angular error between the measured attitude and the target attitude is
// fed into the angle controller and the output of the angle controller summed at the input of the rate controllers.
// By feeding the target angular velocity directly into the rate controllers the measured and target attitudes
// remain very close together.
//
// All input functions below follow the same procedure
// 1. define the desired attitude or attitude change based on the input variables
// 2. update the target attitude based on the angular velocity target and the time since the last loop
// 3. using the desired attitude and input variables, define the target angular velocity so that it should
//    move the target attitude towards the desired attitude
// 4. if _rate_bf_ff_enabled is not being used then make the target attitude
//    and target angular velocities equal to the desired attitude and desired angular velocities.
// 5. ensure _attitude_target, _euler_angle_target_rad, _euler_rate_target_rads and
//    _ang_vel_target_rads have been defined. This ensures input modes can be changed without discontinuity.
// 6. attitude_controller_run_quat is then run to pass the target angular velocities to the rate controllers and
//    integrate them into the target attitude. Any errors between the target attitude and the measured attitude are
//    corrected by first correcting the thrust vector until the angle between the target thrust vector measured
//    trust vector drops below 2*AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD. At this point the heading is also corrected.

// Sets an attitude target using a quaternion and a body-frame angular velocity input (rad/s).
// The desired quaternion is incrementally advanced each timestep using the (limited) angular velocity input.
// If body-frame rate feedforward shaping is enabled, rate/accel targets are generated with acceleration limits
// and input time constants; otherwise the targets are set directly.
void AC_AttitudeControl::input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_body_rads)
{
    // Update internal attitude target state
    update_attitude_target();

    // Limit the body-frame angular velocity input
    ang_vel_limit(ang_vel_body_rads, radians(_ang_vel_roll_max_degs), radians(_ang_vel_pitch_max_degs), radians(_ang_vel_yaw_max_degs));

    // Convert the limited body-frame angular velocity input into the frame used for quaternion integration
    Vector3f ang_vel_target = attitude_desired_quat * ang_vel_body_rads;

    if (_rate_bf_ff_enabled) {
        // Compute attitude error (target -> desired) and express as a small-axis-angle vector
        Quaternion attitude_error_quat = _attitude_target.inverse() * attitude_desired_quat;
        Vector3f attitude_error_angle;
        attitude_error_quat.to_axis_angle(attitude_error_angle);

        // Shape the attitude error into angular velocity and acceleration targets with configured
        // rate and acceleration limits and time constants (roll/pitch use _input_tc, yaw uses _rate_y_tc).
        attitude_command_model(wrap_PI(attitude_error_angle.x), 0.0, _ang_vel_target_rads.x, _ang_accel_target_rads.x, radians(_ang_vel_roll_max_degs), get_accel_roll_max_radss(), _input_tc, _dt_s);
        attitude_command_model(wrap_PI(attitude_error_angle.y), 0.0, _ang_vel_target_rads.y, _ang_accel_target_rads.y, radians(_ang_vel_pitch_max_degs), get_accel_pitch_max_radss(), _input_tc, _dt_s);
        attitude_command_model(wrap_PI(attitude_error_angle.z), 0.0, _ang_vel_target_rads.z, _ang_accel_target_rads.z, radians(_ang_vel_yaw_max_degs), get_accel_yaw_max_radss(), _rate_y_tc, _dt_s);
    } else {
        // No shaping: directly set attitude and angular velocity targets
        _attitude_target = attitude_desired_quat;
        _ang_vel_target_rads = ang_vel_target;
    }

    // Update stored Euler-angle representation of the attitude target
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    body_to_euler_derivative(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);

    // Incrementally advance the desired quaternion using the (limited) angular velocity input
    Quaternion attitude_desired_update;
    attitude_desired_update.from_axis_angle(ang_vel_target * _dt_s);
    attitude_desired_quat = attitude_desired_quat * attitude_desired_update;
    attitude_desired_quat.normalize();

    // Run quaternion attitude controller
    attitude_controller_run_quat();
}

// Sets the desired roll and pitch angles (in centidegrees) and yaw rate (in centidegrees/s).
// See input_euler_angle_roll_pitch_euler_rate_yaw_rad() for full details.
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    const float euler_roll_angle_rad = cd_to_rad(euler_roll_angle_cd);
    const float euler_pitch_angle_rad = cd_to_rad(euler_pitch_angle_cd);
    const float euler_yaw_rate_rads = cd_to_rad(euler_yaw_rate_cds);
    input_euler_angle_roll_pitch_euler_rate_yaw_rad(euler_roll_angle_rad, euler_pitch_angle_rad, euler_yaw_rate_rads);
}

// Sets the desired roll and pitch angle inputs (radians) and a yaw rate input (radians/s).
// Used when roll/pitch stabilization is required while yaw is controlled as a rate.
// If body-frame rate feedforward shaping is enabled, shapes Euler rate/acceleration targets
// with configured limits and time constants and converts them back to body-frame targets.
// Otherwise, updates the attitude target directly and zeros rate/accel feedforward targets.
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads)
{
    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle_rad += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // Convert body-frame angular acceleration limits (roll, pitch, yaw) into equivalent
        // Euler-angle acceleration limits for the current attitude target.
        const Vector3f euler_accel = euler_accel_limit(_attitude_target, Vector3f{get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()});
        const Vector3f euler_rate_max_rads = euler_accel_limit(_attitude_target, Vector3f{radians(_ang_vel_roll_max_degs), radians(_ang_vel_pitch_max_degs), radians(_ang_vel_yaw_max_degs)});

        // Convert the body-frame angular acceleration target into an equivalent Euler-angle
        // acceleration target for the current attitude target.
        Vector3f euler_accel_target_rads;
        body_to_euler_derivative(_attitude_target, _ang_accel_target_rads, euler_accel_target_rads);

        // Shape Euler roll/pitch angle error into Euler rate/acceleration targets.
        // The shaper applies Euler rate/acceleration limits and the configured input time constant.
        attitude_command_model(wrap_PI(euler_roll_angle_rad - _euler_angle_target_rad.x), 0.0, _euler_rate_target_rads.x, euler_accel_target_rads.x, fabsf(euler_rate_max_rads.x), euler_accel.x, _input_tc, _dt_s);
        attitude_command_model(wrap_PI(euler_pitch_angle_rad - _euler_angle_target_rad.y), 0.0, _euler_rate_target_rads.y, euler_accel_target_rads.y, fabsf(euler_rate_max_rads.y), euler_accel.y, _input_tc, _dt_s);

        // Shape yaw rate input into Euler yaw rate/acceleration targets, applying the configured yaw rate time constant
        // and limiting Euler acceleration about the yaw axis.
        attitude_command_model(0.0, euler_yaw_rate_rads, _euler_rate_target_rads.z, euler_accel_target_rads.z, fabsf(euler_rate_max_rads.z), euler_accel.z, _rate_y_tc, _dt_s);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_derivative_to_body(_attitude_target, _euler_rate_target_rads, _ang_vel_target_rads);

        // Convert euler angle derivative of desired attitude into a body-frame angular acceleration vector for feedforward
        euler_derivative_to_body(_attitude_target, euler_accel_target_rads, _ang_accel_target_rads);
    } else {
        // No shaping/feedforward: directly update roll/pitch attitude targets and integrate yaw target from yaw rate.
        _euler_angle_target_rad.x = euler_roll_angle_rad;
        _euler_angle_target_rad.y = euler_pitch_angle_rad;
        _euler_angle_target_rad.z += euler_yaw_rate_rads * _dt_s;

        // Compute quaternion target attitude
        _attitude_target.from_euler(_euler_angle_target_rad.x, _euler_angle_target_rad.y, _euler_angle_target_rad.z);

        // Zero rate and acceleration feedforward targets
        _euler_rate_target_rads.zero();
        _ang_vel_target_rads.zero();
        _ang_accel_target_rads.zero();
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Sets the desired roll, pitch, and yaw angles (in centidegrees).
// See input_euler_angle_roll_pitch_yaw_rad() for full details.
void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
    // Convert from centidegrees on public interface to radians
    const float euler_roll_angle_rad = cd_to_rad(euler_roll_angle_cd);
    const float euler_pitch_angle_rad = cd_to_rad(euler_pitch_angle_cd);
    const float euler_yaw_angle_rad = cd_to_rad(euler_yaw_angle_cd);

    input_euler_angle_roll_pitch_yaw_rad(euler_roll_angle_rad, euler_pitch_angle_rad, euler_yaw_angle_rad, slew_yaw);
}

// Sets the desired roll, pitch, and yaw angle inputs (radians) to follow an absolute attitude setpoint.
// Optional yaw slew limiting constrains the rate of change of the yaw target.
// If body-frame rate feedforward shaping is enabled, shapes Euler rate/acceleration targets
// with configured limits and time constants and converts them back to body-frame targets.
// Otherwise, updates the attitude target directly (with optional yaw slew) and zeros rate/accel feedforward targets.
void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_angle_rad, bool slew_yaw)
{
    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle_rad += get_roll_trim_rad();

    const float slew_yaw_max_rads = get_slew_yaw_max_rads();

    if (_rate_bf_ff_enabled) {
        // Convert body-frame angular acceleration limits (roll, pitch, yaw) into equivalent
        // Euler-angle acceleration limits for the current attitude target.
        const Vector3f euler_accel = euler_accel_limit(_attitude_target, Vector3f{get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()});
        
        float yaw_rate_max_rads = radians(_ang_vel_yaw_max_degs);
        if (slew_yaw) {
            yaw_rate_max_rads = MIN(yaw_rate_max_rads, slew_yaw_max_rads);
        }
        const Vector3f euler_rate_max_rads = euler_accel_limit(_attitude_target, Vector3f{radians(_ang_vel_roll_max_degs), radians(_ang_vel_pitch_max_degs), yaw_rate_max_rads});

        Vector3f euler_accel_target_rads;
        body_to_euler_derivative(_attitude_target, _ang_accel_target_rads, euler_accel_target_rads);
        attitude_command_model(wrap_PI(euler_roll_angle_rad - _euler_angle_target_rad.x), 0.0, _euler_rate_target_rads.x, euler_accel_target_rads.x, fabsf(euler_rate_max_rads.x), euler_accel.x, _input_tc, _dt_s);
        attitude_command_model(wrap_PI(euler_pitch_angle_rad - _euler_angle_target_rad.y), 0.0, _euler_rate_target_rads.y, euler_accel_target_rads.y, fabsf(euler_rate_max_rads.y), euler_accel.y, _input_tc, _dt_s);
        attitude_command_model(wrap_PI(euler_yaw_angle_rad - _euler_angle_target_rad.z), 0.0, _euler_rate_target_rads.z, euler_accel_target_rads.z, fabsf(euler_rate_max_rads.z), euler_accel.z, _input_tc, _dt_s);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_derivative_to_body(_attitude_target, _euler_rate_target_rads, _ang_vel_target_rads);

        // Convert euler angle derivative of desired attitude into a body-frame angular acceleration vector for feedforward
        euler_derivative_to_body(_attitude_target, euler_accel_target_rads, _ang_accel_target_rads);
    } else {
        // No shaping/feedforward: directly update roll/pitch attitude targets and update yaw target
        // with optional slew limiting, then zero rate/accel feedforward targets.
        _euler_angle_target_rad.x = euler_roll_angle_rad;
        _euler_angle_target_rad.y = euler_pitch_angle_rad;

        if (slew_yaw) {
            // Constrain yaw target change to the configured slew rate.
            const float yaw_error = wrap_PI(euler_yaw_angle_rad - _euler_angle_target_rad.z);
            const float yaw_step = constrain_float(yaw_error, -slew_yaw_max_rads * _dt_s, slew_yaw_max_rads * _dt_s);
            _euler_angle_target_rad.z = wrap_PI(_euler_angle_target_rad.z + yaw_step);
        } else {
            _euler_angle_target_rad.z = euler_yaw_angle_rad;
        }

        // Compute quaternion target attitude
        _attitude_target.from_euler(_euler_angle_target_rad.x, _euler_angle_target_rad.y, _euler_angle_target_rad.z);

        // Zero rate and acceleration feedforward targets
        _euler_rate_target_rads.zero();
        _ang_vel_target_rads.zero();
        _ang_accel_target_rads.zero();
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Sets the desired roll, pitch, and yaw Euler angle rate inputs (radians/s).
// If body-frame rate feedforward shaping is enabled, the inputs are shaped using acceleration limits
// and time constants to generate Euler rate/acceleration targets, which are then converted into
// body-frame angular velocity/acceleration targets for the rate controller.
// Otherwise, Euler angle targets are integrated directly from the rate inputs and feedforward targets are zeroed.
void AC_AttitudeControl::input_euler_rate_roll_pitch_yaw_rads(float euler_roll_rate_rads, float euler_pitch_rate_rads, float euler_yaw_rate_rads)
{
    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target_rad);

    if (_rate_bf_ff_enabled) {
        // Convert body-frame angular acceleration limits (roll, pitch, yaw) into
        // equivalent Euler-angle acceleration limits for the current attitude target.
        const Vector3f euler_accel = euler_accel_limit(_attitude_target, Vector3f{get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()});

        // Convert the body-frame angular acceleration target into an equivalent Euler-angle
        // acceleration target for the current attitude target.
        Vector3f euler_accel_target_rads;
        body_to_euler_derivative(_attitude_target, _ang_accel_target_rads, euler_accel_target_rads);

        // Shape Euler rate inputs into Euler rate/acceleration targets, applying acceleration limits and time constants.
        attitude_command_model(0.0, euler_roll_rate_rads, _euler_rate_target_rads.x, euler_accel_target_rads.x, 0.0, euler_accel.x, _rate_rp_tc, _dt_s);
        attitude_command_model(0.0, euler_pitch_rate_rads, _euler_rate_target_rads.y, euler_accel_target_rads.y, 0.0, euler_accel.y, _rate_rp_tc, _dt_s);
        attitude_command_model(0.0, euler_yaw_rate_rads, _euler_rate_target_rads.z, euler_accel_target_rads.z, 0.0, euler_accel.z, _rate_y_tc, _dt_s);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_derivative_to_body(_attitude_target, _euler_rate_target_rads, _ang_vel_target_rads);

        // Convert euler angle derivative of desired attitude into a body-frame angular acceleration vector for feedforward
        euler_derivative_to_body(_attitude_target, euler_accel_target_rads, _ang_accel_target_rads);
    } else {
        // No shaping/feedforward: integrate Euler angle targets from Euler rate inputs.
        // Pitch is constrained to ±85 degrees to avoid gimbal lock discontinuities.
        _euler_angle_target_rad.x = wrap_PI(_euler_angle_target_rad.x + euler_roll_rate_rads * _dt_s);
        _euler_angle_target_rad.y = constrain_float(_euler_angle_target_rad.y + euler_pitch_rate_rads * _dt_s, radians(-85.0f), radians(85.0f));
        _euler_angle_target_rad.z = wrap_2PI(_euler_angle_target_rad.z + euler_yaw_rate_rads * _dt_s);

        // Zero rate and acceleration feedforward targets
        _euler_rate_target_rads.zero();
        _ang_vel_target_rads.zero();
        _ang_accel_target_rads.zero();

        // Compute quaternion target attitude
        _attitude_target.from_euler(_euler_angle_target_rad.x, _euler_angle_target_rad.y, _euler_angle_target_rad.z);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Fully stabilized acro
// Sets the desired roll, pitch, and yaw angular rates (in centidegrees/s).
// See input_rate_bf_roll_pitch_yaw_rads() for full details.
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    const float roll_rate_bf_rads = cd_to_rad(roll_rate_bf_cds);
    const float pitch_rate_bf_rads = cd_to_rad(pitch_rate_bf_cds);
    const float yaw_rate_bf_rads = cd_to_rad(yaw_rate_bf_cds);

    input_rate_bf_roll_pitch_yaw_rads(roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads);
}

// Sets the desired roll, pitch, and yaw body-frame angular rate inputs (radians/s).
// Used by fully stabilized acro modes.
// If body-frame rate feedforward shaping is enabled, the inputs are shaped using acceleration limits
// and time constants to generate body-frame angular velocity/acceleration targets for the rate controller.
// Otherwise, the attitude target is incrementally rotated using the rate inputs and feedforward targets are zeroed.
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads)
{
    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target_rad);

    if (_rate_bf_ff_enabled) {
        // Shape body-frame rate inputs into body-frame angular velocity/acceleration targets,
        // applying acceleration limits and configured time constants.
        attitude_command_model(0.0, roll_rate_bf_rads, _ang_vel_target_rads.x, _ang_accel_target_rads.x, 0.0, get_accel_roll_max_radss(), _rate_rp_tc, _dt_s);
        attitude_command_model(0.0, pitch_rate_bf_rads, _ang_vel_target_rads.y, _ang_accel_target_rads.y, 0.0, get_accel_pitch_max_radss(), _rate_rp_tc, _dt_s);
        attitude_command_model(0.0, yaw_rate_bf_rads, _ang_vel_target_rads.z, _ang_accel_target_rads.z, 0.0, get_accel_yaw_max_radss(), _rate_y_tc, _dt_s);

        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        body_to_euler_derivative(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);
    } else {
        // No shaping/feedforward: incrementally rotate the attitude target using the body-frame rate inputs.
        Quaternion attitude_target_update;
        attitude_target_update.from_axis_angle(Vector3f{roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads} * _dt_s);
        _attitude_target = _attitude_target * attitude_target_update;
        _attitude_target.normalize();

        // Zero rate and acceleration feedforward targets
        _euler_rate_target_rads.zero();
        _ang_vel_target_rads.zero();
        _ang_accel_target_rads.zero();
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Sets the desired roll, pitch, and yaw angular rates in body-frame (in centidegrees/s).
// See input_rate_bf_roll_pitch_yaw_2_rads() for full details.
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_2_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    const float roll_rate_bf_rads = cd_to_rad(roll_rate_bf_cds);
    const float pitch_rate_bf_rads = cd_to_rad(pitch_rate_bf_cds);
    const float yaw_rate_bf_rads = cd_to_rad(yaw_rate_bf_cds);

    input_rate_bf_roll_pitch_yaw_2_rads(roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads);
}

// Sets the desired roll, pitch, and yaw body-frame angular rate inputs (radians/s).
// Used by Copter's rate-only acro mode.
// Shapes the body-frame rate inputs using acceleration limits and time constants to produce
// body-frame angular velocity/acceleration targets for the rate controller.
// Attitude targets are updated from the current AHRS attitude to keep target state coherent for mode transitions.
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_2_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads)
{
    // Shape body-frame rate inputs into body-frame angular velocity/acceleration targets,
    // applying acceleration limits and configured time constants.
    attitude_command_model(0.0, roll_rate_bf_rads, _ang_vel_target_rads.x, _ang_accel_target_rads.x, 0.0, get_accel_roll_max_radss(), _rate_rp_tc, _dt_s);
    attitude_command_model(0.0, pitch_rate_bf_rads, _ang_vel_target_rads.y, _ang_accel_target_rads.y, 0.0, get_accel_pitch_max_radss(), _rate_rp_tc, _dt_s);
    attitude_command_model(0.0, yaw_rate_bf_rads, _ang_vel_target_rads.z, _ang_accel_target_rads.z, 0.0, get_accel_yaw_max_radss(), _rate_y_tc, _dt_s);

    // Update attitude and Euler targets from the current vehicle attitude (used for conditioning mode transitions).
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target_rad);
    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    body_to_euler_derivative(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);

    // Update body-frame angular velocity target used by the rate controller.
    _ang_vel_body_rads = _ang_vel_target_rads;
}

// Sets the desired roll, pitch, and yaw angular rates in body-frame (in centidegrees/s).
// See input_rate_bf_roll_pitch_yaw_3_rads() for full details.
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_3_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    const float roll_rate_bf_rads = cd_to_rad(roll_rate_bf_cds);
    const float pitch_rate_bf_rads = cd_to_rad(pitch_rate_bf_cds);
    const float yaw_rate_bf_rads = cd_to_rad(yaw_rate_bf_cds);

    input_rate_bf_roll_pitch_yaw_3_rads(roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads);
}

// Sets the desired roll, pitch, and yaw body-frame angular rate inputs (radians/s).
// Used by Plane's acro mode with rate error integration.
// Maintains an integrated attitude error quaternion using (target_rate - gyro) and combines it
// with shaped rate inputs to form the final body-frame angular velocity target for the rate controller.
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_3_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads)
{
    // Extract current integrated attitude error as axis-angle
    Vector3f attitude_error;
    _attitude_ang_error.to_axis_angle(attitude_error);

    Quaternion attitude_ang_error_update_quat;

    // Limit the magnitude of the integrated attitude error (prevents windup)
    float err_mag = attitude_error.length();
    if (err_mag > AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD) {
        attitude_error *= AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD / err_mag;
        _attitude_ang_error.from_axis_angle(attitude_error);
    }

    // Integrate attitude error using rate error: (target body rates - measured gyro)
    Vector3f gyro_latest = get_latest_gyro();
    attitude_ang_error_update_quat.from_axis_angle((_ang_vel_target_rads - gyro_latest) * _dt_s);
    _attitude_ang_error = attitude_ang_error_update_quat * _attitude_ang_error;
    _attitude_ang_error.normalize();

    // Shape body-frame rate inputs into body-frame angular velocity/acceleration targets,
    // applying acceleration limits and configured time constants.
    attitude_command_model(0.0, roll_rate_bf_rads, _ang_vel_target_rads.x, _ang_accel_target_rads.x, 0.0, get_accel_roll_max_radss(), _rate_rp_tc, _dt_s);
    attitude_command_model(0.0, pitch_rate_bf_rads, _ang_vel_target_rads.y, _ang_accel_target_rads.y, 0.0, get_accel_pitch_max_radss(), _rate_rp_tc, _dt_s);
    attitude_command_model(0.0, yaw_rate_bf_rads, _ang_vel_target_rads.z, _ang_accel_target_rads.z, 0.0, get_accel_yaw_max_radss(), _rate_y_tc, _dt_s);

    // Retrieve current vehicle attitude (body to NED)
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // Update attitude target by applying the integrated attitude error to the current attitude
    _attitude_target = attitude_body * _attitude_ang_error;
    _attitude_target.normalize();

    // Update stored Euler-angle representation of the attitude target
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    body_to_euler_derivative(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);

    // Compute body-frame angular velocity correction from integrated attitude error and add shaped rate input
    _attitude_ang_error.to_axis_angle(attitude_error);
    Vector3f ang_vel_body_rads = update_ang_vel_target_from_att_error(attitude_error);
    ang_vel_body_rads += _ang_vel_target_rads;

    // Update body-frame angular velocity target used by the rate controller
    _ang_vel_body_rads = ang_vel_body_rads;
}

/*
  set the body frame target rates to the specified rates, used by the
  quadplane code when we want to slave the VTOL controller rates to
  the fixed wing rates
 */

// Directly sets the body-frame angular rates without smoothing (in centidegrees/s).
// See input_rate_bf_roll_pitch_yaw_no_shaping_rads() for full details.
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_no_shaping_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    const float roll_rate_bf_rads = cd_to_rad(roll_rate_bf_cds);
    const float pitch_rate_bf_rads = cd_to_rad(pitch_rate_bf_cds);
    const float yaw_rate_bf_rads = cd_to_rad(yaw_rate_bf_cds);

    input_rate_bf_roll_pitch_yaw_no_shaping_rads(roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads);
}

// Directly sets body-frame angular rate targets (radians/s) without shaping.
// Used when an external controller (e.g. fixed-wing controller) provides VTOL body rates.
// No smoothing, acceleration limiting, or input shaping is applied.
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_no_shaping_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads)
{
    // Set body-frame angular velocity targets directly from inputs
    _ang_vel_target_rads.x = roll_rate_bf_rads;
    _ang_vel_target_rads.y = pitch_rate_bf_rads;
    _ang_vel_target_rads.z = yaw_rate_bf_rads;

    // Update attitude and Euler targets from the current vehicle attitude
    // (used for mode transitions / logging / target state consistency).
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    body_to_euler_derivative(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);

    // Update body-frame angular velocity target used by the rate controller.
    _ang_vel_body_rads = _ang_vel_target_rads;
}

// Applies a one-time angular offset to the attitude target using body-frame roll, pitch,
// and yaw angles (radians).
// Used for step-response excitation during autotuning or manual test inputs.
// The offset is applied instantaneously; no rate or acceleration shaping is performed.
void AC_AttitudeControl::input_angle_step_bf_roll_pitch_yaw_rad(float roll_angle_step_bf_rad, float pitch_angle_step_bf_rad, float yaw_angle_step_bf_rad)
{
    // Apply the requested body-frame angular step to the attitude target
    Quaternion attitude_target_update;
    attitude_target_update.from_axis_angle(Vector3f{roll_angle_step_bf_rad, pitch_angle_step_bf_rad, yaw_angle_step_bf_rad});
    _attitude_target = _attitude_target * attitude_target_update;
    _attitude_target.normalize();

    // Update stored Euler-angle representation of the attitude target
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Zero rate and acceleration feedforward targets (pure attitude step)
    _euler_rate_target_rads.zero();
    _ang_vel_target_rads.zero();
    _ang_accel_target_rads.zero();

    // Run quaternion attitude controller
    attitude_controller_run_quat();
}

// Applies a one-time body-frame angular rate step (radians/s) in roll, pitch, and yaw.
// Used to inject discrete disturbances or step inputs for system identification.
// This sets the body-frame rate command directly for this update; no shaping is applied.
void AC_AttitudeControl::input_rate_step_bf_roll_pitch_yaw_rads(float roll_rate_step_bf_rads, float pitch_rate_step_bf_rads, float yaw_rate_step_bf_rads)
{
    // Update attitude and Euler targets from the current vehicle attitude
    // (used for mode transitions / logging / target state consistency).
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Zero rate and acceleration feedforward targets so the controller cleanly returns to zero rate
    // after the step input is removed.
    _ang_vel_target_rads.zero();
    _ang_accel_target_rads.zero();
    _euler_rate_target_rads.zero();

    // Apply the requested body-frame angular rate step directly to the rate controller input.
    _ang_vel_body_rads = Vector3f{roll_rate_step_bf_rads, pitch_rate_step_bf_rads, yaw_rate_step_bf_rads};
}

// Sets the desired thrust vector and a yaw/heading rate input (radians/s).
// Used for tilt-based navigation with independent yaw control.
// The thrust vector determines the desired tilt (attitude) while the heading rate commands yaw.
// Optional yaw slew limiting constrains the heading rate input.
void AC_AttitudeControl::input_thrust_vector_rate_heading_rads(const Vector3f& thrust_vector, float heading_rate_rads, bool slew_yaw)
{
    if (slew_yaw) {
        // Constrain heading rate input using the configured yaw slew rate limit (0 disables limiting).
        const float slew_yaw_max_rads = get_slew_yaw_max_rads();
        heading_rate_rads = constrain_float(heading_rate_rads, -slew_yaw_max_rads, slew_yaw_max_rads);
    }

    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Convert thrust vector to an attitude quaternion (zero yaw; yaw is commanded separately).
    Quaternion thrust_vec_quat = attitude_from_thrust_vector(thrust_vector, 0.0f);

    // Compute the attitude correction required to align the current target attitude with the thrust vector.
    float thrust_vector_diff_angle;
    Quaternion thrust_vec_correction_quat;
    Vector3f attitude_error;
    float returned_thrust_vector_angle;
    thrust_vector_rotation_angles(thrust_vec_quat, _attitude_target, thrust_vec_correction_quat, attitude_error, returned_thrust_vector_angle, thrust_vector_diff_angle);

    if (_rate_bf_ff_enabled) {
        // Shape roll/pitch attitude error into body-frame angular velocity/acceleration targets,
        // applying configured rate/acceleration limits and input time constant.
        attitude_command_model(attitude_error.x, 0.0, _ang_vel_target_rads.x, _ang_accel_target_rads.x, radians(_ang_vel_roll_max_degs), get_accel_roll_max_radss(), _input_tc, _dt_s);
        attitude_command_model(attitude_error.y, 0.0, _ang_vel_target_rads.y, _ang_accel_target_rads.y, radians(_ang_vel_pitch_max_degs), get_accel_pitch_max_radss(), _input_tc, _dt_s);

        // Shape yaw rate input into yaw angular velocity/acceleration targets, applying yaw limits and time constant.
        attitude_command_model(0.0, heading_rate_rads, _ang_vel_target_rads.z, _ang_accel_target_rads.z, radians(_ang_vel_yaw_max_degs), get_accel_yaw_max_radss(), _rate_y_tc, _dt_s);
    } else {
        // No shaping/feedforward: directly update the attitude target using the thrust-vector correction and yaw increment.
        Quaternion yaw_quat;
        yaw_quat.from_axis_angle(Vector3f{0.0f, 0.0f, heading_rate_rads * _dt_s});
        _attitude_target = _attitude_target * thrust_vec_correction_quat * yaw_quat;
        _attitude_target.normalize();

        // Zero rate and acceleration feedforward targets
        _euler_rate_target_rads.zero();
        _ang_vel_target_rads.zero();
        _ang_accel_target_rads.zero();
    }

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    body_to_euler_derivative(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Sets the desired thrust vector, heading angle (radians), and heading rate input (radians/s).
// Used when thrust direction (tilt) is commanded independently from yaw/heading.
// Heading rate is constrained using the configured yaw slew rate limit (0 disables limiting).
void AC_AttitudeControl::input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_angle_rad, float heading_rate_rads)
{
    // Constrain heading rate input using the configured yaw slew rate limit.
    const float slew_yaw_max_rads = get_slew_yaw_max_rads();
    heading_rate_rads = constrain_float(heading_rate_rads, -slew_yaw_max_rads, slew_yaw_max_rads);

    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target_rad);

    // Convert thrust vector and heading into a desired attitude quaternion.
    const Quaternion desired_attitude_quat = attitude_from_thrust_vector(thrust_vector, heading_angle_rad);

    if (_rate_bf_ff_enabled) {
        // Compute attitude error required to move from the current target attitude to the desired attitude.
        Vector3f attitude_error;
        float thrust_vector_diff_angle;
        Quaternion thrust_vec_correction_quat;
        float returned_thrust_vector_angle;
        thrust_vector_rotation_angles(desired_attitude_quat, _attitude_target, thrust_vec_correction_quat, attitude_error, returned_thrust_vector_angle, thrust_vector_diff_angle);

        // Shape attitude error and heading rate into body-frame angular velocity/acceleration targets,
        // applying configured rate/acceleration limits and time constants (roll/pitch use _input_tc, yaw uses _rate_y_tc).
        attitude_command_model(attitude_error.x, 0.0, _ang_vel_target_rads.x, _ang_accel_target_rads.x, radians(_ang_vel_roll_max_degs),  get_accel_roll_max_radss(),  _input_tc,  _dt_s);
        attitude_command_model(attitude_error.y, 0.0, _ang_vel_target_rads.y, _ang_accel_target_rads.y, radians(_ang_vel_pitch_max_degs), get_accel_pitch_max_radss(), _input_tc,  _dt_s);
        attitude_command_model(attitude_error.z, heading_rate_rads, _ang_vel_target_rads.z, _ang_accel_target_rads.z, radians(_ang_vel_yaw_max_degs),   get_accel_yaw_max_radss(),   _rate_y_tc, _dt_s);
    } else {
        // No shaping/feedforward: directly set the attitude target to the desired attitude.
        _attitude_target = desired_attitude_quat;

        // Zero rate and acceleration feedforward targets
        _euler_rate_target_rads.zero();
        _ang_vel_target_rads.zero();
        _ang_accel_target_rads.zero();
    }

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    body_to_euler_derivative(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command a thrust vector and heading rate
void AC_AttitudeControl::input_thrust_vector_heading(const Vector3f& thrust_vector, HeadingCommand heading)
{
    switch (heading.heading_mode) {
    case HeadingMode::Rate_Only:
        input_thrust_vector_rate_heading_rads(thrust_vector, heading.yaw_rate_rads);
        break;
    case HeadingMode::Angle_Only:
        input_thrust_vector_heading_rad(thrust_vector, heading.yaw_angle_rad, 0.0);
        break;
    case HeadingMode::Angle_And_Rate:
        input_thrust_vector_heading_rad(thrust_vector, heading.yaw_angle_rad, heading.yaw_rate_rads);
        break;
    }
}

Quaternion AC_AttitudeControl::attitude_from_thrust_vector(Vector3f thrust_vector, float heading_angle_rad) const
{
    const Vector3f thrust_vector_up{0.0f, 0.0f, -1.0f};

    if (is_zero(thrust_vector.length_squared())) {
        thrust_vector = thrust_vector_up;
    } else {
        thrust_vector.normalize();
    }

    // the cross product of the desired and target thrust vector defines the rotation vector
    Vector3f thrust_vec_cross = thrust_vector_up % thrust_vector;

    // the dot product is used to calculate the angle between the target and desired thrust vectors
    const float thrust_vector_angle = acosf(constrain_float(thrust_vector_up * thrust_vector, -1.0f, 1.0f));

    // Normalize the thrust rotation vector
    const float thrust_vector_length = thrust_vec_cross.length();
    if (is_zero(thrust_vector_length) || is_zero(thrust_vector_angle)) {
        thrust_vec_cross = thrust_vector_up;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    Quaternion thrust_vec_quat;
    thrust_vec_quat.from_axis_angle(thrust_vec_cross, thrust_vector_angle);
    Quaternion yaw_quat;
    yaw_quat.from_axis_angle(Vector3f{0.0f, 0.0f, 1.0f}, heading_angle_rad);
    return thrust_vec_quat*yaw_quat;
}

// Calculates the body frame angular velocities to follow the target attitude
void AC_AttitudeControl::update_attitude_target()
{
    // rotate target and normalize
    Quaternion attitude_target_update;
    attitude_target_update.from_axis_angle(_ang_vel_target_rads * _dt_s);
    _attitude_target *= attitude_target_update;
    _attitude_target.normalize();
}

// Calculates the body frame angular velocities to follow the target attitude
void AC_AttitudeControl::attitude_controller_run_quat()
{
    // This represents a quaternion rotation in NED frame to the body
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    thrust_heading_rotation_angles(_attitude_target, attitude_body, attitude_error, _thrust_angle_rad, _thrust_error_angle_rad);

    // Compute the angular velocity corrections in the body frame from the attitude error
    Vector3f ang_vel_body_rads = update_ang_vel_target_from_att_error(attitude_error);

    // ensure angular velocity does not go over configured limits
    ang_vel_limit(ang_vel_body_rads, radians(_ang_vel_roll_max_degs), radians(_ang_vel_pitch_max_degs), radians(_ang_vel_yaw_max_degs));

    // rotation from the target frame to the body frame
    Quaternion rotation_target_to_body = attitude_body.inverse() * _attitude_target;

    // target angle velocity vector in the body frame
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _ang_vel_target_rads;
    Vector3f gyro = get_latest_gyro();
    // Correct the thrust vector and smoothly add feedforward and yaw input
    _feedforward_scalar = 1.0f;
    if (_thrust_error_angle_rad > AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD * 2.0f) {
        ang_vel_body_rads.z = gyro.z;
    } else if (_thrust_error_angle_rad > AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD) {
        _feedforward_scalar = (1.0f - (_thrust_error_angle_rad - AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD) / AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD);
        ang_vel_body_rads.x += ang_vel_body_feedforward.x * _feedforward_scalar;
        ang_vel_body_rads.y += ang_vel_body_feedforward.y * _feedforward_scalar;
        ang_vel_body_rads.z += ang_vel_body_feedforward.z;
        ang_vel_body_rads.z = gyro.z * (1.0 - _feedforward_scalar) + ang_vel_body_rads.z * _feedforward_scalar;
    } else {
        ang_vel_body_rads += ang_vel_body_feedforward;
    }

    // Record error to handle EKF resets
    _attitude_ang_error = attitude_body.inverse() * _attitude_target;
    // finally update the attitude target
    _ang_vel_body_rads = ang_vel_body_rads;
}

// thrust_heading_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
// The maximum error in the yaw axis is limited based on static output saturation.
void AC_AttitudeControl::thrust_heading_rotation_angles(Quaternion& attitude_target, const Quaternion& attitude_body, Vector3f& attitude_error_rad, float& thrust_angle_rad, float& thrust_error_angle_rad) const
{
    Quaternion thrust_vector_correction;
    thrust_vector_rotation_angles(attitude_target, attitude_body, thrust_vector_correction, attitude_error_rad, thrust_angle_rad, thrust_error_angle_rad);

    // Todo: Limit roll an pitch error based on output saturation and maximum error.

    // Limit Yaw Error based to the maximum that would saturate the output when yaw rate is zero.
    Quaternion heading_vec_correction_quat;

    float heading_accel_max = constrain_float(get_accel_yaw_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS);
    if (!is_zero(get_rate_yaw_pid().kP())) {
        float heading_error_max = MIN(inv_sqrt_controller(1.0 / get_rate_yaw_pid().kP(), _p_angle_yaw.kP(), heading_accel_max), AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD);
        if (!is_zero(_p_angle_yaw.kP()) && fabsf(attitude_error_rad.z) > heading_error_max) {
            attitude_error_rad.z = constrain_float(wrap_PI(attitude_error_rad.z), -heading_error_max, heading_error_max);
            heading_vec_correction_quat.from_axis_angle(Vector3f{0.0f, 0.0f, attitude_error_rad.z});
            attitude_target = attitude_body * thrust_vector_correction * heading_vec_correction_quat;
        }
    }
}

// thrust_vector_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
// The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
void AC_AttitudeControl::thrust_vector_rotation_angles(const Quaternion& attitude_target, const Quaternion& attitude_body, Quaternion& thrust_vector_correction, Vector3f& attitude_error_rad, float& thrust_angle_rad, float& thrust_error_angle_rad) const
{
    // The direction of thrust is [0,0,-1] is any body-fixed frame, inc. body frame and target frame.
    const Vector3f thrust_vector_up{0.0f, 0.0f, -1.0f};

    // attitude_target and attitude_body are passive rotations from target / body frames to the NED frame
    
    // Rotating [0,0,-1] by attitude_target expresses (gets a view of) the target thrust vector in the inertial frame
    const Vector3f att_target_thrust_vec = attitude_target * thrust_vector_up; // target thrust vector

    // Rotating [0,0,-1] by attitude_target expresses (gets a view of) the current thrust vector in the inertial frame
    const Vector3f att_body_thrust_vec = attitude_body * thrust_vector_up; // current thrust vector

    // the dot product is used to calculate the current lean angle for use of external functions
    thrust_angle_rad = acosf(constrain_float(thrust_vector_up * att_body_thrust_vec,-1.0f,1.0f));

    // the cross product of the desired and target thrust vector defines the rotation vector
    Vector3f thrust_vec_cross = att_body_thrust_vec % att_target_thrust_vec;

    // the dot product is used to calculate the angle between the target and desired thrust vectors
    thrust_error_angle_rad = acosf(constrain_float(att_body_thrust_vec * att_target_thrust_vec, -1.0f, 1.0f));

    // Normalize the thrust rotation vector
    float thrust_vector_length = thrust_vec_cross.length();
    if (is_zero(thrust_vector_length) || is_zero(thrust_error_angle_rad)) {
        thrust_vec_cross = thrust_vector_up;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    // thrust_vector_correction is defined relative to the body frame but its axis `thrust_vec_cross` was computed in
    // the inertial frame. First rotate it by the inverse of attitude_body to express it back in the body frame
    thrust_vec_cross = attitude_body.inverse() * thrust_vec_cross;
    thrust_vector_correction.from_axis_angle(thrust_vec_cross, thrust_error_angle_rad);

    // calculate the angle error in x and y.
    Vector3f rotation_rad;
    thrust_vector_correction.to_axis_angle(rotation_rad);
    attitude_error_rad.x = rotation_rad.x;
    attitude_error_rad.y = rotation_rad.y;

    // calculate the remaining rotation required after thrust vector is rotated transformed to the body frame
    // heading_vector_correction
    Quaternion heading_vec_correction_quat = thrust_vector_correction.inverse() * attitude_body.inverse() * attitude_target;

    // calculate the angle error in z (x and y should be zero here).
    heading_vec_correction_quat.to_axis_angle(rotation_rad);
    attitude_error_rad.z = rotation_rad.z;
}

// calculates the velocity correction from an angle error. The angular velocity has acceleration and
// deceleration limits including basic jerk limiting using _input_tc
void AC_AttitudeControl::attitude_command_model(float error_angle, float desired_ang_vel, float& target_ang_vel, float& target_ang_accel, float max_ang_vel, float accel_max, float input_tc, float dt) const
{
    if (!is_positive(dt)) {
        return;
    }
    
    // protect against divide by zero
    if (!is_positive(accel_max)) {
        // no acceleration set so default to 1800 degrees/s²
        accel_max = radians(1800);
    }

    if (!is_positive(input_tc)) {
        // no acceleration set so default to achieve maximum acceleration in 10 clock cycles
        input_tc = dt * 10.0;
    }

    shape_angle_vel_accel( error_angle, desired_ang_vel, 0.0,
                        0.0, target_ang_vel, target_ang_accel,
                        -max_ang_vel, max_ang_vel, accel_max,
                        accel_max / input_tc, dt, false);
    target_ang_vel += target_ang_accel * dt;
}

// calculates the expected angular velocity correction from an angle error based on the AC_AttitudeControl settings.
// This function can be used to predict the delay associated with angle requests.
void AC_AttitudeControl::command_model_rate_predictor(const Vector2f &error_angle_rad, Vector2f& target_ang_vel_rads, Vector2f& target_ang_accel_rads, float dt) const
{
    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        attitude_command_model(wrap_PI(error_angle_rad.x), 0.0, target_ang_vel_rads.x, target_ang_accel_rads.x, radians(_ang_vel_roll_max_degs), get_accel_roll_max_radss(), _input_tc, _dt_s);
        attitude_command_model(wrap_PI(error_angle_rad.y), 0.0, target_ang_vel_rads.y, target_ang_accel_rads.y, radians(_ang_vel_pitch_max_degs), get_accel_pitch_max_radss(), _input_tc, _dt_s);
    } else {
        const float angleP_roll = _p_angle_roll.kP() * _angle_P_scale.x;
        const float angleP_pitch = _p_angle_pitch.kP() * _angle_P_scale.y;
        target_ang_vel_rads.x = angleP_roll * wrap_PI(error_angle_rad.x);
        target_ang_vel_rads.y = angleP_pitch * wrap_PI(error_angle_rad.y);
    }
    // Limit the angular velocity correction
    Vector3f ang_vel_rads(target_ang_vel_rads.x, target_ang_vel_rads.y, 0.0f);
    ang_vel_limit(ang_vel_rads, radians(_ang_vel_roll_max_degs), radians(_ang_vel_pitch_max_degs), 0.0f);

    target_ang_vel_rads.x = ang_vel_rads.x;
    target_ang_vel_rads.y = ang_vel_rads.y;
}

// scale I to represent the current angle P
void AC_AttitudeControl::scale_I_to_angle_P()
{
    Vector3f i_scale{
        _p_angle_roll.kP() * _angle_P_scale.x,
        _p_angle_pitch.kP() * _angle_P_scale.y,
        _p_angle_yaw.kP() * _angle_P_scale.z
    };
    set_I_scale_mult(i_scale);
}

// perform any required parameter conversions
void AC_AttitudeControl::convert_parameters()
{
    // PARAMETER_CONVERSION - Added: Jan-2026 for 4.7

    // return immediately if no conversion is needed
    if (_angle_max_deg.configured()) {
        return;
    }

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    static const AP_Param::ConversionInfo conversion_info_001[] = {
        { 205, 10, AP_PARAM_INT16, "Q_A_ANGLE_MAX" },  // ANGLE_MAX moved to Q_A_ANGLE_MAX
    };
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
    static const AP_Param::ConversionInfo conversion_info_001[] = {
        { 167, 0, AP_PARAM_INT16, "ATC_ANGLE_MAX" },  // ANGLE_MAX moved to ATC_ANGLE_MAX
    };
#else
    static const AP_Param::ConversionInfo conversion_info_001[] = {
        { 34, 0, AP_PARAM_INT16, "ATC_ANGLE_MAX" },   // ANGLE_MAX moved to ATC_ANGLE_MAX
    };
#endif
    AP_Param::convert_old_parameters_scaled(conversion_info_001, ARRAY_SIZE(conversion_info_001), 0.01, 0);
}

// limits angular velocity
void AC_AttitudeControl::ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max_rads, float ang_vel_pitch_max_rads, float ang_vel_yaw_max_rads) const
{
    if (is_zero(ang_vel_roll_max_rads) || is_zero(ang_vel_pitch_max_rads)) {
        if (!is_zero(ang_vel_roll_max_rads)) {
            euler_rad.x = constrain_float(euler_rad.x, -ang_vel_roll_max_rads, ang_vel_roll_max_rads);
        }
        if (!is_zero(ang_vel_pitch_max_rads)) {
            euler_rad.y = constrain_float(euler_rad.y, -ang_vel_pitch_max_rads, ang_vel_pitch_max_rads);
        }
    } else {
        const Vector2f thrust_vector_ang_vel(euler_rad.x / ang_vel_roll_max_rads, euler_rad.y / ang_vel_pitch_max_rads);
        float thrust_vector_length = thrust_vector_ang_vel.length();
        if (thrust_vector_length > 1.0f) {
            euler_rad.x = thrust_vector_ang_vel.x * ang_vel_roll_max_rads / thrust_vector_length;
            euler_rad.y = thrust_vector_ang_vel.y * ang_vel_pitch_max_rads / thrust_vector_length;
        }
    }
    if (!is_zero(ang_vel_yaw_max_rads)) {
        euler_rad.z = constrain_float(euler_rad.z, -ang_vel_yaw_max_rads, ang_vel_yaw_max_rads);
    }
}

// translates body frame acceleration limits to the euler axis
Vector3f AC_AttitudeControl::euler_accel_limit(const Quaternion &att, const Vector3f &euler_accel)
{
    if (!is_positive(euler_accel.x) || !is_positive(euler_accel.y) || !is_positive(euler_accel.z)) {
        return Vector3f { euler_accel };
    }

    const float phi = att.get_euler_roll();
    const float theta = att.get_euler_pitch();

    const float sin_phi = constrain_float(fabsf(sinf(phi)), 0.1f, 1.0f);
    const float cos_phi = constrain_float(fabsf(cosf(phi)), 0.1f, 1.0f);
    const float sin_theta = constrain_float(fabsf(sinf(theta)), 0.1f, 1.0f);
    const float cos_theta = constrain_float(fabsf(cosf(theta)), 0.1f, 1.0f);

    return Vector3f {
        euler_accel.x,
        MIN(euler_accel.y / cos_phi, euler_accel.z / sin_phi),
        MIN(MIN(euler_accel.x / sin_theta, euler_accel.y / (sin_phi * cos_theta)), euler_accel.z / (cos_phi * cos_theta))
    };
}

// Sets attitude target to vehicle attitude and sets all rates to zero
// If reset_rate is false rates are not reset to allow the rate controllers to run
void AC_AttitudeControl::reset_target_and_rate(bool reset_rate)
{
    // move attitude target to current attitude
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target_rad);

    if (reset_rate) {
        _ang_vel_target_rads.zero();
        _ang_accel_target_rads.zero();
        _euler_rate_target_rads.zero();
    }
}

// Sets yaw target to vehicle heading and sets yaw rate to zero
// If reset_rate is false rates are not reset to allow the rate controllers to run
void AC_AttitudeControl::reset_yaw_target_and_rate(bool reset_rate)
{
    // move attitude target to current heading
    float yaw_shift = _ahrs.yaw - _euler_angle_target_rad.z;
    Quaternion _attitude_target_update;
    _attitude_target_update.from_axis_angle(Vector3f{0.0f, 0.0f, yaw_shift});
    _attitude_target = _attitude_target_update * _attitude_target;

    if (reset_rate) {
        // set yaw rate to zero
        _euler_rate_target_rads.z = 0.0f;
        _ang_accel_target_rads.z = 0.0;

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_derivative_to_body(_attitude_target, _euler_rate_target_rads, _ang_vel_target_rads);
    }
}

// Shifts the target attitude to maintain the current error in the event of an EKF reset
void AC_AttitudeControl::inertial_frame_reset()
{
    // Retrieve quaternion body attitude
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // Recalculate the target quaternion
    _attitude_target = attitude_body * _attitude_ang_error;

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target_rad);
}

// euler_derivative_to_body - transform euler angle derivative to body-frame
// Converts euler derivatives (rate, acceleration, etc.) to body-frame equivalents.
// The same transformation applies regardless of derivative order.
// Uses the kinematic relationship for 321 (yaw-pitch-roll) euler sequence.
void AC_AttitudeControl::euler_derivative_to_body(const Quaternion& att, const Vector3f& euler_derivative_rads, Vector3f& body_derivative_rads)
{
    const float theta = att.get_euler_pitch();
    const float phi = att.get_euler_roll();

    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);
    const float sin_phi = sinf(phi);
    const float cos_phi = cosf(phi);

    body_derivative_rads.x = euler_derivative_rads.x - sin_theta * euler_derivative_rads.z;
    body_derivative_rads.y = cos_phi * euler_derivative_rads.y + sin_phi * cos_theta * euler_derivative_rads.z;
    body_derivative_rads.z = -sin_phi * euler_derivative_rads.y + cos_theta * cos_phi * euler_derivative_rads.z;
}

// body_to_euler_derivative - transform body-frame derivative to euler angle derivative
// Converts body-frame derivatives (rate, acceleration, etc.) to euler equivalents.
// The same transformation applies regardless of derivative order.
// Uses the kinematic relationship for 321 (yaw-pitch-roll) euler sequence.
// Returns false if the vehicle is pitched 90 degrees up or down (gimbal lock)
bool AC_AttitudeControl::body_to_euler_derivative(const Quaternion& att, const Vector3f& body_derivative_rads, Vector3f& euler_derivative_rads)
{
    const float theta = att.get_euler_pitch();
    const float phi = att.get_euler_roll();

    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);
    const float sin_phi = sinf(phi);
    const float cos_phi = cosf(phi);

    // When the vehicle pitches all the way up or all the way down, the euler angles become discontinuous. In this case, we just return false.
    if (is_zero(cos_theta)) {
        return false;
    }

    euler_derivative_rads.x = body_derivative_rads.x + sin_phi * (sin_theta / cos_theta) * body_derivative_rads.y + cos_phi * (sin_theta / cos_theta) * body_derivative_rads.z;
    euler_derivative_rads.y = cos_phi * body_derivative_rads.y - sin_phi * body_derivative_rads.z;
    euler_derivative_rads.z = (sin_phi / cos_theta) * body_derivative_rads.y + (cos_phi / cos_theta) * body_derivative_rads.z;
    return true;
}

// Update rate_target_ang_vel using attitude_error_rot_vec_rad
Vector3f AC_AttitudeControl::update_ang_vel_target_from_att_error(const Vector3f &attitude_error_rot_vec_rad)
{
    Vector3f rate_target_ang_vel;

    // Compute the roll angular velocity demand from the roll angle error
    const float angleP_roll = _p_angle_roll.kP() * _angle_P_scale.x;
    if (_use_sqrt_controller && !is_zero(get_accel_roll_max_radss())) {
        rate_target_ang_vel.x = sqrt_controller(attitude_error_rot_vec_rad.x, angleP_roll, constrain_float(get_accel_roll_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt_s);
    } else {
        rate_target_ang_vel.x = angleP_roll * attitude_error_rot_vec_rad.x;
    }

    // Compute the pitch angular velocity demand from the pitch angle error
    const float angleP_pitch = _p_angle_pitch.kP() * _angle_P_scale.y;
    if (_use_sqrt_controller && !is_zero(get_accel_pitch_max_radss())) {
        rate_target_ang_vel.y = sqrt_controller(attitude_error_rot_vec_rad.y, angleP_pitch, constrain_float(get_accel_pitch_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt_s);
    } else {
        rate_target_ang_vel.y = angleP_pitch * attitude_error_rot_vec_rad.y;
    }

    // Compute the yaw angular velocity demand from the yaw angle error
    const float angleP_yaw = _p_angle_yaw.kP() * _angle_P_scale.z;
    if (_use_sqrt_controller && !is_zero(get_accel_yaw_max_radss())) {
        rate_target_ang_vel.z = sqrt_controller(attitude_error_rot_vec_rad.z, angleP_yaw, constrain_float(get_accel_yaw_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS), _dt_s);
    } else {
        rate_target_ang_vel.z = angleP_yaw * attitude_error_rot_vec_rad.z;
    }

    return rate_target_ang_vel;
}

// Enable or disable body-frame feed forward
void AC_AttitudeControl::accel_limiting(bool enable_limits)
{
    if (enable_limits) {
        // If enabling limits, reload from eeprom or set to defaults
        if (is_zero(_accel_roll_max_cdss)) {
            _accel_roll_max_cdss.load();
        }
        if (is_zero(_accel_pitch_max_cdss)) {
            _accel_pitch_max_cdss.load();
        }
        if (is_zero(_accel_yaw_max_cdss)) {
            _accel_yaw_max_cdss.load();
        }
    } else {
        _accel_roll_max_cdss.set(0.0f);
        _accel_pitch_max_cdss.set(0.0f);
        _accel_yaw_max_cdss.set(0.0f);
    }
}

// Returns maximum allowable tilt angle (in centidegrees) for pilot input when in altitude hold mode.
// See get_althold_lean_angle_max_rad() for full details.
float AC_AttitudeControl::get_althold_lean_angle_max_cd() const
{
    // convert to centi-degrees for public interface
    return rad_to_cd(get_althold_lean_angle_max_rad());
}

// Returns maximum allowable tilt angle (in radians) for pilot input when in altitude hold mode.
// Used to limit lean angle based on available thrust margin, prioritising altitude stability.
float AC_AttitudeControl::get_althold_lean_angle_max_rad() const
{
    return MAX(_althold_lean_angle_max_rad, radians(AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN));
}

// Return configured tilt angle limit in centidegrees
float AC_AttitudeControl::lean_angle_max_cd() const
{
    return constrain_float(_angle_max_deg.get(), AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MAX) * 100;
}

// Return configured tilt angle limit in radians
float AC_AttitudeControl::lean_angle_max_rad() const
{
    return radians(constrain_float(_angle_max_deg.get(), AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MAX));
}

// Return roll rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float dt_average = AP::scheduler().get_filtered_loop_time();
    float alpha = MIN(get_rate_roll_pid().get_filt_E_alpha(dt_average), get_rate_roll_pid().get_filt_D_alpha(dt_average));
    float alpha_remaining = 1 - alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    float rate_max = 2.0f * throttle_hover * AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_roll_pid().kD()) / _dt_s + get_rate_roll_pid().kP());
    if (is_positive(_ang_vel_roll_max_degs)) {
        rate_max = MIN(rate_max, get_ang_vel_roll_max_rads());
    }
    return rate_max;
}

// Return pitch rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    const float dt_average = AP::scheduler().get_filtered_loop_time();
    const float alpha = MIN(get_rate_pitch_pid().get_filt_E_alpha(dt_average), get_rate_pitch_pid().get_filt_D_alpha(dt_average));
    const float alpha_remaining = 1 - alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    const float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    float rate_max = 2.0f * throttle_hover * AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_pitch_pid().kD()) / _dt_s + get_rate_pitch_pid().kP());
    if (is_positive(_ang_vel_pitch_max_degs)) {
        rate_max = MIN(rate_max, get_ang_vel_pitch_max_rads());
    }
    return rate_max;
}

// Return yaw rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    const float dt_average = AP::scheduler().get_filtered_loop_time();
    const float alpha = MIN(get_rate_yaw_pid().get_filt_E_alpha(dt_average), get_rate_yaw_pid().get_filt_D_alpha(dt_average));
    const float alpha_remaining = 1 - alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    const float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    float rate_max = 2.0f * throttle_hover * AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_yaw_pid().kD()) / _dt_s + get_rate_yaw_pid().kP());
    if (is_positive(_ang_vel_yaw_max_degs)) {
        rate_max = MIN(rate_max, get_ang_vel_yaw_max_rads());
    }
    return rate_max;
}

bool AC_AttitudeControl::pre_arm_checks(const char *param_prefix,
                                        char *failure_msg,
                                        const uint8_t failure_msg_len)
{
    // validate AC_P members:
    const struct {
        const char *pid_name;
        AC_P &p;
    } ps[] = {
        { "ANG_PIT", get_angle_pitch_p() },
        { "ANG_RLL", get_angle_roll_p() },
        { "ANG_YAW", get_angle_yaw_p() }
    };
    for (uint8_t i=0; i<ARRAY_SIZE(ps); i++) {
        // all AC_P's must have a positive P value:
        if (!is_positive(ps[i].p.kP())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, ps[i].pid_name);
            return false;
        }
    }

    // validate AC_PID members:
    const struct {
        const char *pid_name;
        AC_PID &pid;
    } pids[] = {
        { "RAT_RLL", get_rate_roll_pid() },
        { "RAT_PIT", get_rate_pitch_pid() },
        { "RAT_YAW", get_rate_yaw_pid() },
    };
    for (uint8_t i=0; i<ARRAY_SIZE(pids); i++) {
        // if the PID has a positive FF then we just ensure kP and
        // kI aren't negative
        AC_PID &pid = pids[i].pid;
        const char *pid_name = pids[i].pid_name;
        if (is_positive(pid.ff())) {
            // kP and kI must be non-negative:
            if (is_negative(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be >= 0", param_prefix, pid_name);
                return false;
            }
            if (is_negative(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be >= 0", param_prefix, pid_name);
                return false;
            }
        } else {
            // kP and kI must be positive:
            if (!is_positive(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, pid_name);
                return false;
            }
            if (!is_positive(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be > 0", param_prefix, pid_name);
                return false;
            }
        }
        // never allow a negative D term (but zero is allowed)
        if (is_negative(pid.kD())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_D must be >= 0", param_prefix, pid_name);
            return false;
        }
    }

    // validate ANGLE_MAX
    if (_angle_max_deg.get() < 10 || _angle_max_deg.get() > 80) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ANGLE_MAX must be >= 10 and <= 80", param_prefix);
        return false;
    }

    return true;
}

/*
  get the slew rate for roll, pitch and yaw, for oscillation
  detection in lua scripts
*/
void AC_AttitudeControl::get_rpy_srate(float &roll_srate, float &pitch_srate, float &yaw_srate)
{
    roll_srate = get_rate_roll_pid().get_pid_info().slew_rate;
    pitch_srate = get_rate_pitch_pid().get_pid_info().slew_rate;
    yaw_srate = get_rate_yaw_pid().get_pid_info().slew_rate;
}
