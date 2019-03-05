#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // default gains for Plane
 # define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT  0.2f    // Soft
#else
 // default gains for Copter and Sub
 # define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT  0.15f   // Medium
#endif

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] = {

    // 0, 1 were RATE_RP_MAX, RATE_Y_MAX

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    // @Units: cdeg/s
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW",    2, AC_AttitudeControl, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS),

    // 3 was for ACCEL_RP_MAX

    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: cdeg/s/s
    // @Range: 0 72000
    // @Values: 0:Disabled, 9000:VerySlow, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX",  4, AC_AttitudeControl, _accel_yaw_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedfoward is enabled or disabled
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
    AP_GROUPINFO("ACCEL_R_MAX", 6, AC_AttitudeControl, _accel_roll_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // @Param: ACCEL_P_MAX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: cdeg/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_P_MAX", 7, AC_AttitudeControl, _accel_pitch_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

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
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 13, AC_AttitudeControl, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 14, AC_AttitudeControl, AC_P),

    // @Param: ANG_YAW_P
    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @Range{Sub}: 0.0 6.000
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
    // @Values: 0:Disabled, 360:Slow, 720:Medium, 1080:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_R_MAX", 17, AC_AttitudeControl, _ang_vel_roll_max, 0.0f),

    // @Param: RATE_P_MAX
    // @DisplayName: Angular Velocity Max for Pitch
    // @Description: Maximum angular velocity in pitch axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 360:Slow, 720:Medium, 1080:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_P_MAX", 18, AC_AttitudeControl, _ang_vel_pitch_max, 0.0f),

    // @Param: RATE_Y_MAX
    // @DisplayName: Angular Velocity Max for Yaw
    // @Description: Maximum angular velocity in yaw axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 360:Slow, 720:Medium, 1080:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_Y_MAX", 19, AC_AttitudeControl, _ang_vel_yaw_max, 0.0f),

    // @Param: INPUT_TC
    // @DisplayName: Attitude control input time constant (aka smoothing)
    // @Description: Attitude control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_GROUPINFO("INPUT_TC", 20, AC_AttitudeControl, _input_tc, AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT),

    AP_GROUPEND
};

// Set output throttle and disable stabilization
void AC_AttitudeControl::set_throttle_out_unstabilized(float throttle_in, bool reset_attitude_control, float filter_cutoff)
{
    _throttle_in = throttle_in;
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (reset_attitude_control) {
        relax_attitude_controllers();
    }
    _motors.set_throttle(throttle_in);
    _angle_boost = 0.0f;
}

// Ensure attitude controller have zero errors to relax rate controller output
void AC_AttitudeControl::relax_attitude_controllers()
{
    // Initialize the attitude variables to the current attitude
    // TODO add _ahrs.get_quaternion()
    _attitude_target_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
    _attitude_ang_error.initialise();

    // Initialize the angular rate variables to the current rate
    _attitude_target_ang_vel = _ahrs.get_gyro();
    ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    _rate_target_ang_vel = _ahrs.get_gyro();

    // Initialize remaining variables
    _thrust_error_angle = 0.0f;

    // Reset the PID filters
    get_rate_roll_pid().reset_filter();
    get_rate_pitch_pid().reset_filter();
    get_rate_yaw_pid().reset_filter();

    // Reset the I terms
    reset_rate_controller_I_terms();
}

void AC_AttitudeControl::reset_rate_controller_I_terms()
{
    get_rate_roll_pid().reset_I();
    get_rate_pitch_pid().reset_I();
    get_rate_yaw_pid().reset_I();
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
// 1. define the desired attitude the aircraft should attempt to achieve using the input variables
// 2. using the desired attitude and input variables, define the target angular velocity so that it should
//    move the target attitude towards the desired attitude
// 3. if _rate_bf_ff_enabled is not being used then make the target attitude
//    and target angular velocities equal to the desired attitude and desired angular velocities.
// 4. ensure _attitude_target_quat, _attitude_target_euler_angle, _attitude_target_euler_rate and
//    _attitude_target_ang_vel have been defined. This ensures input modes can be changed without discontinuity.
// 5. attitude_controller_run_quat is then run to pass the target angular velocities to the rate controllers and
//    integrate them into the target attitude. Any errors between the target attitude and the measured attitude are
//    corrected by first correcting the thrust vector until the angle between the target thrust vector measured
//    trust vector drops below 2*AC_ATTITUDE_THRUST_ERROR_ANGLE. At this point the heading is also corrected.



// Command a Quaternion attitude with feedforward and smoothing
void AC_AttitudeControl::input_quaternion(Quaternion attitude_desired_quat)
{
    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    Quaternion attitude_error_quat = _attitude_target_quat.inverse() * attitude_desired_quat;
    Vector3f attitude_error_angle;
    attitude_error_quat.to_axis_angle(attitude_error_angle);

    if (_rate_bf_ff_enabled) {
        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by _input_tc at the end.
        _attitude_target_ang_vel.x = input_shaping_angle(wrap_PI(attitude_error_angle.x), _input_tc, get_accel_roll_max_radss(), _attitude_target_ang_vel.x, _dt);
        _attitude_target_ang_vel.y = input_shaping_angle(wrap_PI(attitude_error_angle.y), _input_tc, get_accel_pitch_max_radss(), _attitude_target_ang_vel.y, _dt);
        _attitude_target_ang_vel.z = input_shaping_angle(wrap_PI(attitude_error_angle.z), _input_tc, get_accel_yaw_max_radss(), _attitude_target_ang_vel.z, _dt);

        // Limit the angular velocity
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        _attitude_target_quat = attitude_desired_quat;

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_angle = radians(euler_roll_angle_cd*0.01f);
    float euler_pitch_angle = radians(euler_pitch_angle_cd*0.01f);
    float euler_yaw_rate = radians(euler_yaw_rate_cds*0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by smoothing_gain at the end.
        _attitude_target_euler_rate.x = input_shaping_angle(wrap_PI(euler_roll_angle-_attitude_target_euler_angle.x), _input_tc, euler_accel.x, _attitude_target_euler_rate.x, _dt);
        _attitude_target_euler_rate.y = input_shaping_angle(wrap_PI(euler_pitch_angle-_attitude_target_euler_angle.y), _input_tc, euler_accel.y, _attitude_target_euler_rate.y, _dt);

        // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        // the output rate towards the input rate.
        _attitude_target_euler_rate.z = input_shaping_ang_vel(_attitude_target_euler_rate.z, euler_yaw_rate, euler_accel.z, _dt);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
        // Limit the angular velocity
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        _attitude_target_euler_angle.x = euler_roll_angle;
        _attitude_target_euler_angle.y = euler_pitch_angle;
        _attitude_target_euler_angle.z += euler_yaw_rate*_dt;
        // Compute quaternion target attitude
        _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_angle = radians(euler_roll_angle_cd*0.01f);
    float euler_pitch_angle = radians(euler_pitch_angle_cd*0.01f);
    float euler_yaw_angle = radians(euler_yaw_angle_cd*0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by _input_tc at the end.
        _attitude_target_euler_rate.x = input_shaping_angle(wrap_PI(euler_roll_angle-_attitude_target_euler_angle.x), _input_tc, euler_accel.x, _attitude_target_euler_rate.x, _dt);
        _attitude_target_euler_rate.y = input_shaping_angle(wrap_PI(euler_pitch_angle-_attitude_target_euler_angle.y), _input_tc, euler_accel.y, _attitude_target_euler_rate.y, _dt);
        _attitude_target_euler_rate.z = input_shaping_angle(wrap_PI(euler_yaw_angle-_attitude_target_euler_angle.z), _input_tc, euler_accel.z, _attitude_target_euler_rate.z, _dt);
        if (slew_yaw) {
            _attitude_target_euler_rate.z = constrain_float(_attitude_target_euler_rate.z, -get_slew_yaw_rads(), get_slew_yaw_rads());
        }

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
        // Limit the angular velocity
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        _attitude_target_euler_angle.x = euler_roll_angle;
        _attitude_target_euler_angle.y = euler_pitch_angle;
        if (slew_yaw) {
            // Compute constrained angle error
            float angle_error = constrain_float(wrap_PI(euler_yaw_angle-_attitude_target_euler_angle.z), -get_slew_yaw_rads()*_dt, get_slew_yaw_rads()*_dt);
            // Update attitude target from constrained angle error
            _attitude_target_euler_angle.z = wrap_PI(angle_error + _attitude_target_euler_angle.z);
        } else {
            _attitude_target_euler_angle.z = euler_yaw_angle;
        }
        // Compute quaternion target attitude
        _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_rate = radians(euler_roll_rate_cds*0.01f);
    float euler_pitch_rate = radians(euler_pitch_rate_cds*0.01f);
    float euler_yaw_rate = radians(euler_yaw_rate_cds*0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting is enabled, the input shaper constrains angular acceleration, slewing
        // the output rate towards the input rate.
        _attitude_target_euler_rate.x = input_shaping_ang_vel(_attitude_target_euler_rate.x, euler_roll_rate, euler_accel.x, _dt);
        _attitude_target_euler_rate.y = input_shaping_ang_vel(_attitude_target_euler_rate.y, euler_pitch_rate, euler_accel.y, _dt);
        _attitude_target_euler_rate.z = input_shaping_ang_vel(_attitude_target_euler_rate.z, euler_yaw_rate, euler_accel.z, _dt);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        // Pitch angle is restricted to +- 85.0 degrees to avoid gimbal lock discontinuities.
        _attitude_target_euler_angle.x = wrap_PI(_attitude_target_euler_angle.x + euler_roll_rate*_dt);
        _attitude_target_euler_angle.y = constrain_float(_attitude_target_euler_angle.y + euler_pitch_rate*_dt, radians(-85.0f), radians(85.0f));
        _attitude_target_euler_angle.z = wrap_2PI(_attitude_target_euler_angle.z + euler_yaw_rate*_dt);

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);

        // Compute quaternion target attitude
        _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads = radians(roll_rate_bf_cds*0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds*0.01f);
    float yaw_rate_rads = radians(yaw_rate_bf_cds*0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    if (_rate_bf_ff_enabled) {
        // Compute acceleration-limited body frame rates
        // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
        // the output rate towards the input rate.
        _attitude_target_ang_vel.x = input_shaping_ang_vel(_attitude_target_ang_vel.x, roll_rate_rads, get_accel_roll_max_radss(), _dt);
        _attitude_target_ang_vel.y = input_shaping_ang_vel(_attitude_target_ang_vel.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt);
        _attitude_target_ang_vel.z = input_shaping_ang_vel(_attitude_target_ang_vel.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt);

        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        // When feedforward is not enabled, the quaternion is calculated and is input into the target and the feedforward rate is zeroed.
        Quaternion attitude_target_update_quat;
        attitude_target_update_quat.from_axis_angle(Vector3f(roll_rate_rads * _dt, pitch_rate_rads * _dt, yaw_rate_rads * _dt));
        _attitude_target_quat = _attitude_target_quat * attitude_target_update_quat;
        _attitude_target_quat.normalize();

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an angular velocity with angular velocity smoothing using rate loops only with no attitude loop stabilization
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads = radians(roll_rate_bf_cds*0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds*0.01f);
    float yaw_rate_rads = radians(yaw_rate_bf_cds*0.01f);

    // Compute acceleration-limited body frame rates
    // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
    // the output rate towards the input rate.
    _attitude_target_ang_vel.x = input_shaping_ang_vel(_attitude_target_ang_vel.x, roll_rate_rads, get_accel_roll_max_radss(), _dt);
    _attitude_target_ang_vel.y = input_shaping_ang_vel(_attitude_target_ang_vel.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt);
    _attitude_target_ang_vel.z = input_shaping_ang_vel(_attitude_target_ang_vel.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt);

    // Update the unused targets attitude based on current attitude to condition mode change
    _attitude_target_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    _rate_target_ang_vel = _attitude_target_ang_vel;
}

// Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads = radians(roll_rate_bf_cds*0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds*0.01f);
    float yaw_rate_rads = radians(yaw_rate_bf_cds*0.01f);

    // Update attitude error
    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    Quaternion attitude_ang_error_update_quat;
    attitude_ang_error_update_quat.from_axis_angle(Vector3f((_attitude_target_ang_vel.x-gyro_latest.x) * _dt, (_attitude_target_ang_vel.y-gyro_latest.y) * _dt, (_attitude_target_ang_vel.z-gyro_latest.z) * _dt));
    _attitude_ang_error = attitude_ang_error_update_quat * _attitude_ang_error;

    // Compute acceleration-limited body frame rates
    // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
    // the output rate towards the input rate.
    _attitude_target_ang_vel.x = input_shaping_ang_vel(_attitude_target_ang_vel.x, roll_rate_rads, get_accel_roll_max_radss(), _dt);
    _attitude_target_ang_vel.y = input_shaping_ang_vel(_attitude_target_ang_vel.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt);
    _attitude_target_ang_vel.z = input_shaping_ang_vel(_attitude_target_ang_vel.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt);

    // Retrieve quaternion vehicle attitude
    // TODO add _ahrs.get_quaternion()
    Quaternion attitude_vehicle_quat;
    attitude_vehicle_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());

    // Update the unused targets attitude based on current attitude to condition mode change
    _attitude_target_quat = attitude_vehicle_quat*_attitude_ang_error;

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);

    // Compute the angular velocity target from the integrated rate error
    Vector3f attitude_error_vector;
    _attitude_ang_error.to_axis_angle(attitude_error_vector);
    _rate_target_ang_vel = update_ang_vel_target_from_att_error(attitude_error_vector);
    _rate_target_ang_vel += _attitude_target_ang_vel;

    // ensure Quaternions stay normalized
    _attitude_ang_error.normalize();
}

// Command an angular step (i.e change) in body frame angle
// Used to command a step in angle without exciting the orthogonal axis during autotune
void AC_AttitudeControl::input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd)
{
    // Convert from centidegrees on public interface to radians
    float roll_step_rads = radians(roll_angle_step_bf_cd*0.01f);
    float pitch_step_rads = radians(pitch_angle_step_bf_cd*0.01f);
    float yaw_step_rads = radians(yaw_angle_step_bf_cd*0.01f);

    // rotate attitude target by desired step
    Quaternion attitude_target_update_quat;
    attitude_target_update_quat.from_axis_angle(Vector3f(roll_step_rads, pitch_step_rads, yaw_step_rads));
    _attitude_target_quat = _attitude_target_quat * attitude_target_update_quat;
    _attitude_target_quat.normalize();

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Set rate feedforward requests to zero
    _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
    _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Calculates the body frame angular velocities to follow the target attitude
void AC_AttitudeControl::attitude_controller_run_quat()
{
    // Retrieve quaternion vehicle attitude
    // TODO add _ahrs.get_quaternion()
    Quaternion attitude_vehicle_quat;
    attitude_vehicle_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());

    // Compute attitude error
    Vector3f attitude_error_vector;
    thrust_heading_rotation_angles(_attitude_target_quat, attitude_vehicle_quat, attitude_error_vector, _thrust_error_angle);

    // Compute the angular velocity target from the attitude error
    _rate_target_ang_vel = update_ang_vel_target_from_att_error(attitude_error_vector);

    // Add feedforward term that attempts to ensure that roll and pitch errors rotate with the body frame rather than the reference frame.
    // todo: this should probably be a matrix that couples yaw as well.
    _rate_target_ang_vel.x += constrain_float(attitude_error_vector.y, -M_PI/4, M_PI/4)  * _ahrs.get_gyro().z;
    _rate_target_ang_vel.y += -constrain_float(attitude_error_vector.x, -M_PI/4, M_PI/4) * _ahrs.get_gyro().z;

    ang_vel_limit(_rate_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));

    // Add the angular velocity feedforward, rotated into vehicle frame
    Quaternion attitude_target_ang_vel_quat = Quaternion(0.0f, _attitude_target_ang_vel.x, _attitude_target_ang_vel.y, _attitude_target_ang_vel.z);
    Quaternion to_to_from_quat = attitude_vehicle_quat.inverse() * _attitude_target_quat;
    Quaternion desired_ang_vel_quat = to_to_from_quat.inverse()*attitude_target_ang_vel_quat*to_to_from_quat;

    // Correct the thrust vector and smoothly add feedforward and yaw input
    if(_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE*2.0f){
        _rate_target_ang_vel.z = _ahrs.get_gyro().z;
    }else if(_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE){
        float feedforward_scalar = (1.0f - (_thrust_error_angle-AC_ATTITUDE_THRUST_ERROR_ANGLE)/AC_ATTITUDE_THRUST_ERROR_ANGLE);
        _rate_target_ang_vel.x += desired_ang_vel_quat.q2*feedforward_scalar;
        _rate_target_ang_vel.y += desired_ang_vel_quat.q3*feedforward_scalar;
        _rate_target_ang_vel.z += desired_ang_vel_quat.q4;
        _rate_target_ang_vel.z = _ahrs.get_gyro().z*(1.0-feedforward_scalar) + _rate_target_ang_vel.z*feedforward_scalar;
    } else {
        _rate_target_ang_vel.x += desired_ang_vel_quat.q2;
        _rate_target_ang_vel.y += desired_ang_vel_quat.q3;
        _rate_target_ang_vel.z += desired_ang_vel_quat.q4;
    }

    if (_rate_bf_ff_enabled) {
        // rotate target and normalize
        Quaternion attitude_target_update_quat;
        attitude_target_update_quat.from_axis_angle(Vector3f(_attitude_target_ang_vel.x * _dt, _attitude_target_ang_vel.y * _dt, _attitude_target_ang_vel.z * _dt));
        _attitude_target_quat = _attitude_target_quat * attitude_target_update_quat;
        _attitude_target_quat.normalize();
    }

    // ensure Quaternions stay normalized
    _attitude_target_quat.normalize();

    // Record error to handle EKF resets
    _attitude_ang_error = attitude_vehicle_quat.inverse() * _attitude_target_quat;
}

// thrust_heading_rotation_angles - calculates two ordered rotations to move the att_from_quat quaternion to the att_to_quat quaternion.
// The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
void AC_AttitudeControl::thrust_heading_rotation_angles(Quaternion& att_to_quat, const Quaternion& att_from_quat, Vector3f& att_diff_angle, float& thrust_vec_dot)
{
    Matrix3f att_to_rot_matrix; // rotation from the target body frame to the inertial frame.
    att_to_quat.rotation_matrix(att_to_rot_matrix);
    Vector3f att_to_thrust_vec = att_to_rot_matrix*Vector3f(0.0f,0.0f,1.0f);

    Matrix3f att_from_rot_matrix; // rotation from the current body frame to the inertial frame.
    att_from_quat.rotation_matrix(att_from_rot_matrix);
    Vector3f att_from_thrust_vec = att_from_rot_matrix*Vector3f(0.0f,0.0f,1.0f);

    // the cross product of the desired and target thrust vector defines the rotation vector
    Vector3f thrust_vec_cross = att_from_thrust_vec % att_to_thrust_vec;

    // the dot product is used to calculate the angle between the target and desired thrust vectors
    thrust_vec_dot = acosf(constrain_float(att_from_thrust_vec * att_to_thrust_vec,-1.0f,1.0f));

    // Normalize the thrust rotation vector
    float thrust_vector_length = thrust_vec_cross.length();
    if(is_zero(thrust_vector_length) || is_zero(thrust_vec_dot)){
        thrust_vec_cross = Vector3f(0,0,1);
        thrust_vec_dot = 0.0f;
    }else{
        thrust_vec_cross /= thrust_vector_length;
    }
    Quaternion thrust_vec_correction_quat;
    thrust_vec_correction_quat.from_axis_angle(thrust_vec_cross, thrust_vec_dot);

    // Rotate thrust_vec_correction_quat to the att_from frame
    thrust_vec_correction_quat = att_from_quat.inverse()*thrust_vec_correction_quat*att_from_quat;

    // calculate the remaining rotation required after thrust vector is rotated transformed to the att_from frame
    Quaternion yaw_vec_correction_quat = thrust_vec_correction_quat.inverse()*att_from_quat.inverse()*att_to_quat;

    // calculate the angle error in x and y.
    Vector3f rotation;
    thrust_vec_correction_quat.to_axis_angle(rotation);
    att_diff_angle.x = rotation.x;
    att_diff_angle.y = rotation.y;

    // calculate the angle error in z (x and y should be zero here).
    yaw_vec_correction_quat.to_axis_angle(rotation);
    att_diff_angle.z = rotation.z;

    // Todo: Limit roll an pitch error based on output saturation and maximum error.

    // Limit Yaw Error based on maximum acceleration - Update to include output saturation and maximum error.
    // Currently the limit is based on the maximum acceleration using the linear part of the SQRT controller.
    // This should be updated to be based on an angle limit, saturation, or unlimited based on user defined parameters.
    if(!is_zero(_p_angle_yaw.kP()) && fabsf(att_diff_angle.z) > AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS/_p_angle_yaw.kP()){
        att_diff_angle.z = constrain_float(wrap_PI(att_diff_angle.z), -AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS/_p_angle_yaw.kP(), AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS/_p_angle_yaw.kP());
        yaw_vec_correction_quat.from_axis_angle(Vector3f(0.0f,0.0f,att_diff_angle.z));
        att_to_quat = att_from_quat*thrust_vec_correction_quat*yaw_vec_correction_quat;
    }
}

// calculates the velocity correction from an angle error. The angular velocity has acceleration and
// deceleration limits including basic jerk limiting using _input_tc
float AC_AttitudeControl::input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float dt)
{
    // Calculate the velocity as error approaches zero with acceleration limited by accel_max_radss
    float desired_ang_vel = sqrt_controller(error_angle, 1.0f / MAX(input_tc, 0.01f), accel_max, dt);

    // Acceleration is limited directly to smooth the beginning of the curve.
    return input_shaping_ang_vel(target_ang_vel, desired_ang_vel, accel_max, dt);
}

// limits the acceleration and deceleration of a velocity request
float AC_AttitudeControl::input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max, float dt)
{
    // Acceleration is limited directly to smooth the beginning of the curve.
    if (is_positive(accel_max)) {
        float delta_ang_vel = accel_max * dt;
        return constrain_float(desired_ang_vel, target_ang_vel-delta_ang_vel, target_ang_vel+delta_ang_vel);
    } else {
        return desired_ang_vel;
    }
}

// calculates the expected angular velocity correction from an angle error based on the AC_AttitudeControl settings.
// This function can be used to predict the delay associated with angle requests.
void AC_AttitudeControl::input_shaping_rate_predictor(const Vector2f &error_angle, Vector2f& target_ang_vel, float dt) const
{
    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        target_ang_vel.x = input_shaping_angle(wrap_PI(error_angle.x), _input_tc, get_accel_roll_max_radss(), target_ang_vel.x, dt);
        target_ang_vel.y = input_shaping_angle(wrap_PI(error_angle.y), _input_tc, get_accel_pitch_max_radss(), target_ang_vel.y, dt);
    } else {
        target_ang_vel.x =  _p_angle_roll.get_p(wrap_PI(error_angle.x));
        target_ang_vel.y =  _p_angle_pitch.get_p(wrap_PI(error_angle.y));
    }
    // Limit the angular velocity correction
    Vector3f ang_vel(target_ang_vel.x, target_ang_vel.y, 0.0f);
    ang_vel_limit(ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), 0.0f);

    target_ang_vel.x =  ang_vel.x;
    target_ang_vel.y =  ang_vel.y;
}

// limits angular velocity
void AC_AttitudeControl::ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max, float ang_vel_pitch_max, float ang_vel_yaw_max) const
{
    if (is_zero(ang_vel_roll_max) || is_zero(ang_vel_pitch_max)) {
        if (!is_zero(ang_vel_roll_max)) {
            euler_rad.x = constrain_float(euler_rad.x, -ang_vel_roll_max, ang_vel_roll_max);
        }
        if (!is_zero(ang_vel_pitch_max)) {
            euler_rad.y = constrain_float(euler_rad.y, -ang_vel_pitch_max, ang_vel_pitch_max);
        }
    } else {
        Vector2f thrust_vector_ang_vel(euler_rad.x/ang_vel_roll_max, euler_rad.y/ang_vel_pitch_max);
        float thrust_vector_length = thrust_vector_ang_vel.length();
        if (thrust_vector_length > 1.0f) {
            euler_rad.x = thrust_vector_ang_vel.x * ang_vel_roll_max / thrust_vector_length;
            euler_rad.y = thrust_vector_ang_vel.y * ang_vel_pitch_max / thrust_vector_length;
        }
    }
    if (!is_zero(ang_vel_yaw_max)) {
        euler_rad.z = constrain_float(euler_rad.z, -ang_vel_yaw_max, ang_vel_yaw_max);
    }
}

// translates body frame acceleration limits to the euler axis
Vector3f AC_AttitudeControl::euler_accel_limit(const Vector3f &euler_rad, const Vector3f &euler_accel)
{
    float sin_phi = constrain_float(fabsf(sinf(euler_rad.x)), 0.1f, 1.0f);
    float cos_phi = constrain_float(fabsf(cosf(euler_rad.x)), 0.1f, 1.0f);
    float sin_theta = constrain_float(fabsf(sinf(euler_rad.y)), 0.1f, 1.0f);

    Vector3f rot_accel;
    if(is_zero(euler_accel.x) || is_zero(euler_accel.y) || is_zero(euler_accel.z) || is_negative(euler_accel.x) || is_negative(euler_accel.y) || is_negative(euler_accel.z)) {
        rot_accel.x = euler_accel.x;
        rot_accel.y = euler_accel.y;
        rot_accel.z = euler_accel.z;
    } else {
        rot_accel.x = euler_accel.x;
        rot_accel.y = MIN(euler_accel.y/cos_phi, euler_accel.z/sin_phi);
        rot_accel.z = MIN(MIN(euler_accel.x/sin_theta, euler_accel.y/sin_phi), euler_accel.z/cos_phi);
    }
    return rot_accel;
}

// Shifts earth frame yaw target by yaw_shift_cd. yaw_shift_cd should be in centidegrees and is added to the current target heading
void AC_AttitudeControl::shift_ef_yaw_target(float yaw_shift_cd)
{
    float yaw_shift = radians(yaw_shift_cd*0.01f);
    Quaternion _attitude_target_update_quat;
    _attitude_target_update_quat.from_axis_angle(Vector3f(0.0f, 0.0f, yaw_shift));
    _attitude_target_quat = _attitude_target_update_quat*_attitude_target_quat;
}

// Shifts earth frame yaw target by yaw_shift_cd. yaw_shift_cd should be in centidegrees and is added to the current target heading
void AC_AttitudeControl::inertial_frame_reset()
{
    // Retrieve quaternion vehicle attitude
    // TODO add _ahrs.get_quaternion()
    Quaternion attitude_vehicle_quat;
    attitude_vehicle_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());

    // Recalculate the target quaternion
    _attitude_target_quat = attitude_vehicle_quat * _attitude_ang_error;

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);
}

// Convert a 321-intrinsic euler angle derivative to an angular velocity vector
void AC_AttitudeControl::euler_rate_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    ang_vel_rads.x = euler_rate_rads.x - sin_theta * euler_rate_rads.z;
    ang_vel_rads.y = cos_phi  * euler_rate_rads.y + sin_phi * cos_theta * euler_rate_rads.z;
    ang_vel_rads.z = -sin_phi * euler_rate_rads.y + cos_theta * cos_phi * euler_rate_rads.z;
}

// Convert an angular velocity vector to a 321-intrinsic euler angle derivative
// Returns false if the vehicle is pitched 90 degrees up or down
bool AC_AttitudeControl::ang_vel_to_euler_rate(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    // When the vehicle pitches all the way up or all the way down, the euler angles become discontinuous. In this case, we just return false.
    if (is_zero(cos_theta)) {
        return false;
    }

    euler_rate_rads.x = ang_vel_rads.x + sin_phi * (sin_theta/cos_theta) * ang_vel_rads.y + cos_phi * (sin_theta/cos_theta) * ang_vel_rads.z;
    euler_rate_rads.y = cos_phi  * ang_vel_rads.y - sin_phi * ang_vel_rads.z;
    euler_rate_rads.z = (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z;
    return true;
}

// Update rate_target_ang_vel using attitude_error_rot_vec_rad
Vector3f AC_AttitudeControl::update_ang_vel_target_from_att_error(const Vector3f &attitude_error_rot_vec_rad)
{
    Vector3f rate_target_ang_vel;
    // Compute the roll angular velocity demand from the roll angle error
    if (_use_sqrt_controller) {
        rate_target_ang_vel.x = sqrt_controller(attitude_error_rot_vec_rad.x, _p_angle_roll.kP(), constrain_float(get_accel_roll_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt);
    }else{
        rate_target_ang_vel.x = _p_angle_roll.kP() * attitude_error_rot_vec_rad.x;
    }
    // todo: Add Angular Velocity Limit

    // Compute the pitch angular velocity demand from the roll angle error
    if (_use_sqrt_controller) {
        rate_target_ang_vel.y = sqrt_controller(attitude_error_rot_vec_rad.y, _p_angle_pitch.kP(), constrain_float(get_accel_pitch_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt);
    }else{
        rate_target_ang_vel.y = _p_angle_pitch.kP() * attitude_error_rot_vec_rad.y;
    }
    // todo: Add Angular Velocity Limit

    // Compute the yaw angular velocity demand from the roll angle error
    if (_use_sqrt_controller) {
        rate_target_ang_vel.z = sqrt_controller(attitude_error_rot_vec_rad.z, _p_angle_yaw.kP(), constrain_float(get_accel_yaw_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS), _dt);
    }else{
        rate_target_ang_vel.z = _p_angle_yaw.kP() * attitude_error_rot_vec_rad.z;
    }
    // todo: Add Angular Velocity Limit
    return rate_target_ang_vel;
}

// Run the roll angular velocity PID controller and return the output
float AC_AttitudeControl::rate_target_to_motor_roll(float rate_actual_rads, float rate_target_rads)
{
    float rate_error_rads = rate_target_rads - rate_actual_rads;

    // pass error to PID controller
    get_rate_roll_pid().set_input_filter_d(rate_error_rads);
    get_rate_roll_pid().set_desired_rate(rate_target_rads);

    float integrator = get_rate_roll_pid().get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!_motors.limit.roll_pitch || ((is_positive(integrator) && is_negative(rate_error_rads)) || (is_negative(integrator) && is_positive(rate_error_rads)))) {
        integrator = get_rate_roll_pid().get_i();
    }

    // Compute output in range -1 ~ +1
    float output = get_rate_roll_pid().get_p() + integrator + get_rate_roll_pid().get_d() + get_rate_roll_pid().get_ff(rate_target_rads);

    // Constrain output
    return output;
}

// Run the pitch angular velocity PID controller and return the output
float AC_AttitudeControl::rate_target_to_motor_pitch(float rate_actual_rads, float rate_target_rads)
{
    float rate_error_rads = rate_target_rads - rate_actual_rads;

    // pass error to PID controller
    get_rate_pitch_pid().set_input_filter_d(rate_error_rads);
    get_rate_pitch_pid().set_desired_rate(rate_target_rads);

    float integrator = get_rate_pitch_pid().get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!_motors.limit.roll_pitch || ((is_positive(integrator) && is_negative(rate_error_rads)) || (is_negative(integrator) && is_positive(rate_error_rads)))) {
        integrator = get_rate_pitch_pid().get_i();
    }

    // Compute output in range -1 ~ +1
    float output = get_rate_pitch_pid().get_p() + integrator + get_rate_pitch_pid().get_d() + get_rate_pitch_pid().get_ff(rate_target_rads);

    // Constrain output
    return output;
}

// Run the yaw angular velocity PID controller and return the output
float AC_AttitudeControl::rate_target_to_motor_yaw(float rate_actual_rads, float rate_target_rads)
{
    float rate_error_rads = rate_target_rads - rate_actual_rads;

    // pass error to PID controller
    get_rate_yaw_pid().set_input_filter_all(rate_error_rads);
    get_rate_yaw_pid().set_desired_rate(rate_target_rads);

    float integrator = get_rate_yaw_pid().get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!_motors.limit.yaw || ((is_positive(integrator) && is_negative(rate_error_rads)) || (is_negative(integrator) && is_positive(rate_error_rads)))) {
        integrator = get_rate_yaw_pid().get_i();
    }

    // Compute output in range -1 ~ +1
    float output = get_rate_yaw_pid().get_p() + integrator + get_rate_yaw_pid().get_d() + get_rate_yaw_pid().get_ff(rate_target_rads);

    // Constrain output
    return output;
}

// Enable or disable body-frame feed forward
void AC_AttitudeControl::accel_limiting(bool enable_limits)
{
    if (enable_limits) {
        // If enabling limits, reload from eeprom or set to defaults
        if (is_zero(_accel_roll_max)) {
            _accel_roll_max.load();
        }
        if (is_zero(_accel_pitch_max)) {
            _accel_pitch_max.load();
        }
        if (is_zero(_accel_yaw_max)) {
            _accel_yaw_max.load();
        }
    } else {
        _accel_roll_max = 0.0f;
        _accel_pitch_max = 0.0f;
        _accel_yaw_max = 0.0f;
    }
}

// Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
float AC_AttitudeControl::get_althold_lean_angle_max() const
{
    // convert to centi-degrees for public interface
    return MAX(ToDeg(_althold_lean_angle_max), AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN) * 100.0f;
}

// Proportional controller with piecewise sqrt sections to constrain second derivative
float AC_AttitudeControl::sqrt_controller(float error, float p, float second_ord_lim, float dt)
{
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // second order limit is zero or negative.
        correction_rate = error*p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            correction_rate = safe_sqrt(2.0f*second_ord_lim*(error));
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0f*second_ord_lim*(-error));
        } else {
            correction_rate = 0.0f;
        }
    } else {
        // Both the P and second order limit have been defined.
        float linear_dist = second_ord_lim/sq(p);
        if (error > linear_dist) {
            correction_rate = safe_sqrt(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
        } else if (error < -linear_dist) {
            correction_rate = -safe_sqrt(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
        } else {
            correction_rate = error*p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        return constrain_float(correction_rate, -fabsf(error)/dt, fabsf(error)/dt);
    } else {
        return correction_rate;
    }
}

// Inverse proportional controller with piecewise sqrt sections to constrain second derivative
float AC_AttitudeControl::stopping_point(float first_ord_mag, float p, float second_ord_lim)
{
    if (is_positive(second_ord_lim) && !is_zero(second_ord_lim) && is_zero(p)) {
        return (first_ord_mag*first_ord_mag)/(2.0f*second_ord_lim);
    } else if ((is_negative(second_ord_lim) || is_zero(second_ord_lim)) && !is_zero(p)) {
        return first_ord_mag/p;
    } else if ((is_negative(second_ord_lim) || is_zero(second_ord_lim)) && is_zero(p)) {
        return 0.0f;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    float linear_velocity = second_ord_lim/p;

    if (fabsf(first_ord_mag) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        return first_ord_mag/p;
    } else {
        float linear_dist = second_ord_lim/sq(p);
        float overshoot = (linear_dist*0.5f) + sq(first_ord_mag)/(2.0f*second_ord_lim);
        if (is_positive(first_ord_mag)){
            return overshoot;
        } else {
            return -overshoot;
        }
    }
}

// Return roll rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float alpha = get_rate_roll_pid().get_filt_alpha();
    float alpha_remaining = 1-alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    return 2.0f*throttle_hover*AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*get_rate_roll_pid().kD())/_dt + get_rate_roll_pid().kP());
}

// Return pitch rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    float alpha = get_rate_pitch_pid().get_filt_alpha();
    float alpha_remaining = 1-alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    return 2.0f*throttle_hover*AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*get_rate_pitch_pid().kD())/_dt + get_rate_pitch_pid().kP());
}

// Return yaw rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    float alpha = get_rate_yaw_pid().get_filt_alpha();
    float alpha_remaining = 1-alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    return 2.0f*throttle_hover*AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*get_rate_yaw_pid().kD())/_dt + get_rate_yaw_pid().kP());
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
    return true;
}
