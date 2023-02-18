#include <AP_HAL/AP_HAL.h>
#include "AC_Loiter.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#define LOITER_SPEED_DEFAULT                1250.0f // default loiter speed in cm/s
#define LOITER_SPEED_MIN                    20.0f   // minimum loiter speed in cm/s
#define LOITER_ACCEL_MAX_DEFAULT            500.0f  // default acceleration in loiter mode
#define LOITER_BRAKE_ACCEL_DEFAULT          250.0f  // minimum acceleration in loiter mode
#define LOITER_BRAKE_JERK_DEFAULT           500.0f  // maximum jerk in cm/s/s/s in loiter mode
#define LOITER_BRAKE_START_DELAY_DEFAULT    1.0f    // delay (in seconds) before loiter braking begins after sticks are released
#define LOITER_VEL_CORRECTION_MAX           200.0f  // max speed used to correct position errors in loiter
#define LOITER_POS_CORRECTION_MAX           200.0f  // max position error in loiter
#define LOITER_ACTIVE_TIMEOUT_MS            200     // loiter controller is considered active if it has been called within the past 200ms (0.2 seconds)

const AP_Param::GroupInfo AC_Loiter::var_info[] = {

    // @Param: ANG_MAX
    // @DisplayName: Loiter pilot angle max
    // @Description{Copter, Sub}: Loiter maximum pilot requested lean angle. Set to zero for 2/3 of PSC_ANGLE_MAX/ANGLE_MAX. The maximum vehicle lean angle is still limited by PSC_ANGLE_MAX/ANGLE_MAX
    // @Description: Loiter maximum pilot requested lean angle. Set to zero for 2/3 of Q_P_ANGLE_MAX/Q_ANGLE_MAX. The maximum vehicle lean angle is still limited by Q_P_ANGLE_MAX/Q_ANGLE_MAX
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ANG_MAX",  1, AC_Loiter, _angle_max, 0.0f),

    // @Param: SPEED
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: cm/s
    // @Range: 20 3500
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED", 2, AC_Loiter, _speed_cms, LOITER_SPEED_DEFAULT),

    // @Param: ACC_MAX
    // @DisplayName: Loiter maximum correction acceleration
    // @Description: Loiter maximum correction acceleration in cm/s/s.  Higher values cause the copter to correct position errors more aggressively.
    // @Units: cm/s/s
    // @Range: 100 981
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ACC_MAX", 3, AC_Loiter, _accel_cmss, LOITER_ACCEL_MAX_DEFAULT),

    // @Param: BRK_ACCEL
    // @DisplayName: Loiter braking acceleration
    // @Description: Loiter braking acceleration in cm/s/s. Higher values stop the copter more quickly when the stick is centered.
    // @Units: cm/s/s
    // @Range: 25 250
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("BRK_ACCEL", 4, AC_Loiter, _brake_accel_cmss, LOITER_BRAKE_ACCEL_DEFAULT),

    // @Param: BRK_JERK
    // @DisplayName: Loiter braking jerk
    // @Description: Loiter braking jerk in cm/s/s/s. Higher values will remove braking faster if the pilot moves the sticks during a braking maneuver.
    // @Units: cm/s/s/s
    // @Range: 500 5000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("BRK_JERK", 5, AC_Loiter, _brake_jerk_max_cmsss, LOITER_BRAKE_JERK_DEFAULT),

    // @Param: BRK_DELAY
    // @DisplayName: Loiter brake start delay (in seconds)
    // @Description: Loiter brake start delay (in seconds)
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BRK_DELAY",  6, AC_Loiter, _brake_delay, LOITER_BRAKE_START_DELAY_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Loiter::AC_Loiter(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/// init_target to a position in cm from ekf origin
void AC_Loiter::init_target(const Vector2f& position)
{
    sanity_check_params();

    // initialise position controller speed and acceleration
    _pos_control.set_correction_speed_accel_xy(LOITER_VEL_CORRECTION_MAX, _accel_cmss);
    _pos_control.set_pos_error_max_xy_cm(LOITER_POS_CORRECTION_MAX);

    // initialise position controller
    _pos_control.init_xy_controller_stopping_point();

    // initialise desired acceleration and angles to zero to remain on station
    _predicted_accel.zero();
    _desired_accel.zero();
    _predicted_euler_angle.zero();
    _brake_accel = 0.0f;

    // set target position
    _pos_control.set_pos_target_xy_cm(position.x, position.y);
}

/// initialize's position and feed-forward velocity from current pos and velocity
void AC_Loiter::init_target()
{
    sanity_check_params();

    // initialise position controller speed and acceleration
    _pos_control.set_correction_speed_accel_xy(LOITER_VEL_CORRECTION_MAX, _accel_cmss);
    _pos_control.set_pos_error_max_xy_cm(LOITER_POS_CORRECTION_MAX);

    // initialise position controller and move target accelerations smoothly towards zero
    _pos_control.relax_velocity_controller_xy();

    // initialise predicted acceleration and angles from the position controller
    _predicted_accel.x = _pos_control.get_accel_target_cmss().x;
    _predicted_accel.y = _pos_control.get_accel_target_cmss().y;
    _predicted_euler_angle.x = radians(_pos_control.get_roll_cd()*0.01f);
    _predicted_euler_angle.y = radians(_pos_control.get_pitch_cd()*0.01f);
    _brake_accel = 0.0f;
}

/// reduce response for landing
void AC_Loiter::soften_for_landing()
{
    _pos_control.soften_for_landing_xy();
}

/// set pilot desired acceleration in centi-degrees
//   dt should be the time (in seconds) since the last call to this function
void AC_Loiter::set_pilot_desired_acceleration(float euler_roll_angle_cd, float euler_pitch_angle_cd)
{
    const float dt = _attitude_control.get_dt();
    // Convert from centidegrees on public interface to radians
    const float euler_roll_angle = radians(euler_roll_angle_cd * 0.01f);
    const float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);

    // convert our desired attitude to an acceleration vector assuming we are not accelerating vertically
    const Vector3f desired_euler {euler_roll_angle, euler_pitch_angle, _ahrs.yaw};
    const Vector3f desired_accel = _pos_control.lean_angles_to_accel(desired_euler);

    _desired_accel.x = desired_accel.x;
    _desired_accel.y = desired_accel.y;

    // difference between where we think we should be and where we want to be
    Vector2f angle_error(wrap_PI(euler_roll_angle - _predicted_euler_angle.x), wrap_PI(euler_pitch_angle - _predicted_euler_angle.y));

    // calculate the angular velocity that we would expect given our desired and predicted attitude
    _attitude_control.input_shaping_rate_predictor(angle_error, _predicted_euler_rate, dt);

    // update our predicted attitude based on our predicted angular velocity
    _predicted_euler_angle += _predicted_euler_rate * dt;

    // convert our predicted attitude to an acceleration vector assuming we are not accelerating vertically
    const Vector3f predicted_euler {_predicted_euler_angle.x, _predicted_euler_angle.y, _ahrs.yaw};
    const Vector3f predicted_accel = _pos_control.lean_angles_to_accel(predicted_euler);

    _predicted_accel.x = predicted_accel.x;
    _predicted_accel.y = predicted_accel.y;
}

/// get vector to stopping point based on a horizontal position and velocity
void AC_Loiter::get_stopping_point_xy(Vector2f& stopping_point) const
{
    Vector2p stop;
    _pos_control.get_stopping_point_xy_cm(stop);
    stopping_point = stop.tofloat();
}

/// get maximum lean angle when using loiter
float AC_Loiter::get_angle_max_cd() const
{
    if (!is_positive(_angle_max)) {
        return MIN(_attitude_control.lean_angle_max_cd(), _pos_control.get_lean_angle_max_cd()) * (2.0f/3.0f);
    }
    return MIN(_angle_max*100.0f, _pos_control.get_lean_angle_max_cd());
}

/// run the loiter controller
void AC_Loiter::update(bool avoidance_on)
{
    calc_desired_velocity(avoidance_on);
    _pos_control.update_xy_controller();
}

// sanity check parameters
void AC_Loiter::sanity_check_params()
{
    _speed_cms.set(MAX(_speed_cms, LOITER_SPEED_MIN));
    _accel_cmss.set(MIN(_accel_cmss, GRAVITY_MSS * 100.0f * tanf(ToRad(_attitude_control.lean_angle_max_cd() * 0.01f))));
}

/// calc_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
///		updated velocity sent directly to position controller
void AC_Loiter::calc_desired_velocity(bool avoidance_on)
{
    float ekfGndSpdLimit, ahrsControlScaleXY;
    AP::ahrs().getControlLimits(ekfGndSpdLimit, ahrsControlScaleXY);

    const float dt = _pos_control.get_dt();

    // calculate a loiter speed limit which is the minimum of the value set by the LOITER_SPEED
    // parameter and the value set by the EKF to observe optical flow limits
    float gnd_speed_limit_cms = MIN(_speed_cms, ekfGndSpdLimit * 100.0f);
    gnd_speed_limit_cms = MAX(gnd_speed_limit_cms, LOITER_SPEED_MIN);

    float pilot_acceleration_max = angle_to_accel(get_angle_max_cd() * 0.01) * 100;

    // range check dt
    if (is_negative(dt)) {
        return;
    }

    // get loiters desired velocity from the position controller where it is being stored.
    const Vector3f &desired_vel_3d = _pos_control.get_vel_desired_cms();
    Vector2f desired_vel{desired_vel_3d.x,desired_vel_3d.y};

    // update the desired velocity using our predicted acceleration
    desired_vel.x += _predicted_accel.x * dt;
    desired_vel.y += _predicted_accel.y * dt;

    Vector2f loiter_accel_brake;
    float desired_speed = desired_vel.length();
    if (!is_zero(desired_speed)) {
        Vector2f desired_vel_norm = desired_vel / desired_speed;

        // calculate a drag acceleration based on the desired speed.
        float drag_decel = pilot_acceleration_max * desired_speed / gnd_speed_limit_cms;

        // calculate a braking acceleration if sticks are at zero
        float loiter_brake_accel = 0.0f;
        if (_desired_accel.is_zero()) {
            if ((AP_HAL::millis() - _brake_timer) > _brake_delay * 1000.0f) {
                float brake_gain = _pos_control.get_vel_xy_pid().kP() * 0.5f;
                loiter_brake_accel = constrain_float(sqrt_controller(desired_speed, brake_gain, _brake_jerk_max_cmsss, dt), 0.0f, _brake_accel_cmss);
            }
        } else {
            loiter_brake_accel = 0.0f;
            _brake_timer = AP_HAL::millis();
        }
        _brake_accel += constrain_float(loiter_brake_accel - _brake_accel, -_brake_jerk_max_cmsss * dt, _brake_jerk_max_cmsss * dt);
        loiter_accel_brake = desired_vel_norm * _brake_accel;

        // update the desired velocity using the drag and braking accelerations
        desired_speed = MAX(desired_speed - (drag_decel + _brake_accel) * dt, 0.0f);
        desired_vel = desired_vel_norm * desired_speed;
    }

    // add braking to the desired acceleration
    _desired_accel -= loiter_accel_brake;

    // Apply EKF limit to desired velocity -  this limit is calculated by the EKF and adjusted as required to ensure certain sensor limits are respected (eg optical flow sensing)
    float horizSpdDem = desired_vel.length();
    if (horizSpdDem > gnd_speed_limit_cms) {
        desired_vel.x = desired_vel.x * gnd_speed_limit_cms / horizSpdDem;
        desired_vel.y = desired_vel.y * gnd_speed_limit_cms / horizSpdDem;
    }

#if !APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    if (avoidance_on) {
        // Limit the velocity to prevent fence violations
        // TODO: We need to also limit the _desired_accel
        AC_Avoid *_avoid = AP::ac_avoid();
        if (_avoid != nullptr) {
            Vector3f avoidance_vel_3d{desired_vel.x, desired_vel.y, 0.0f};
            _avoid->adjust_velocity(avoidance_vel_3d, _pos_control.get_pos_xy_p().kP(), _accel_cmss, _pos_control.get_pos_z_p().kP(), _pos_control.get_max_accel_z_cmss(), dt);
            desired_vel = Vector2f{avoidance_vel_3d.x, avoidance_vel_3d.y};
        }
    }
#endif // !APM_BUILD_ArduPlane

    // get loiters desired velocity from the position controller where it is being stored.
    Vector2p target_pos = _pos_control.get_pos_target_cm().xy();

    // update the target position using our predicted velocity
    target_pos += (desired_vel * dt).topostype();

    // send adjusted feed forward acceleration and velocity back to the Position Controller
    _pos_control.set_pos_vel_accel_xy(target_pos, desired_vel, _desired_accel);
}
