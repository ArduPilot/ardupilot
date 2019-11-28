#include <AP_HAL/AP_HAL.h>
#include "AC_Loiter.h"

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
    // @DisplayName: Loiter Angle Max
    // @Description: Loiter maximum lean angle. Set to zero for 2/3 of PSC_ANGLE_MAX or ANGLE_MAX
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ANG_MAX",  1, AC_Loiter, _angle_max, 0.0f),

    // @Param: SPEED
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: cm/s
    // @Range: 20 2000
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
    // @Description: Loiter braking jerk in cm/s/s/s. Higher values will remove braking faster if the pilot moves the sticks during a braking manuver.
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
void AC_Loiter::init_target(const Vector3f& position)
{
    sanity_check_params();

    // initialise pos controller speed, acceleration
    _pos_control.set_speed_xy(LOITER_VEL_CORRECTION_MAX);
    _pos_control.set_accel_xy(_accel_cmss);

    // initialise desired acceleration and angles to zero to remain on station
    _predicted_accel.zero();
    _desired_accel = _predicted_accel;
    _predicted_euler_angle.zero();

    // set target position
    _pos_control.set_xy_target(position.x, position.y);

    // set vehicle velocity and acceleration to zero
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.set_desired_accel_xy(0.0f,0.0f);

    // initialise position controller
    _pos_control.init_xy_controller();
}

/// initialize's position and feed-forward velocity from current pos and velocity
void AC_Loiter::init_target()
{
    const Vector3f& curr_pos = _inav.get_position();
    const Vector3f& curr_vel = _inav.get_velocity();

    sanity_check_params();

    // initialise pos controller speed and acceleration
    _pos_control.set_speed_xy(LOITER_VEL_CORRECTION_MAX);
    _pos_control.set_accel_xy(_accel_cmss);
    _pos_control.set_leash_length_xy(LOITER_POS_CORRECTION_MAX);

    _predicted_accel = _desired_accel;
    // update angle targets that will be passed to stabilize controller
    float roll_cd, pitch_cd;
    _pos_control.accel_to_lean_angles(_predicted_accel.x, _predicted_accel.y, roll_cd, pitch_cd);
    _predicted_euler_angle.x = radians(roll_cd*0.01);
    _predicted_euler_angle.y = radians(pitch_cd*0.01);
    // set target position
    _pos_control.set_xy_target(curr_pos.x, curr_pos.y);

    // set vehicle velocity and acceleration to current state
    _pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);
    _pos_control.set_desired_accel_xy(_desired_accel.x, _desired_accel.y);

    // initialise position controller
    _pos_control.init_xy_controller();
}

/// reduce response for landing
void AC_Loiter::soften_for_landing()
{
    const Vector3f& curr_pos = _inav.get_position();

    // set target position to current position
    _pos_control.set_xy_target(curr_pos.x, curr_pos.y);

    // also prevent I term build up in xy velocity controller. Note
    // that this flag is reset on each loop, in run_xy_controller()
    _pos_control.set_limit_accel_xy();
}

/// set pilot desired acceleration in centi-degrees
//   dt should be the time (in seconds) since the last call to this function
void AC_Loiter::set_pilot_desired_acceleration(float euler_roll_angle_cd, float euler_pitch_angle_cd, float dt)
{
    // Convert from centidegrees on public interface to radians
    const float euler_roll_angle = radians(euler_roll_angle_cd*0.01f);
    const float euler_pitch_angle = radians(euler_pitch_angle_cd*0.01f);

    // convert our desired attitude to an acceleration vector assuming we are hovering
    const float pilot_cos_pitch_target = constrain_float(cosf(euler_pitch_angle), 0.5f, 1.0f);
    const float pilot_accel_rgt_cms = GRAVITY_MSS*100.0f * tanf(euler_roll_angle)/pilot_cos_pitch_target;
    const float pilot_accel_fwd_cms = -GRAVITY_MSS*100.0f * tanf(euler_pitch_angle);

    // rotate acceleration vectors input to lat/lon frame
    _desired_accel.x = (pilot_accel_fwd_cms*_ahrs.cos_yaw() - pilot_accel_rgt_cms*_ahrs.sin_yaw());
    _desired_accel.y = (pilot_accel_fwd_cms*_ahrs.sin_yaw() + pilot_accel_rgt_cms*_ahrs.cos_yaw());

    // difference between where we think we should be and where we want to be
    Vector2f angle_error(wrap_PI(euler_roll_angle - _predicted_euler_angle.x), wrap_PI(euler_pitch_angle - _predicted_euler_angle.y));

    // calculate the angular velocity that we would expect given our desired and predicted attitude
    _attitude_control.input_shaping_rate_predictor(angle_error, _predicted_euler_rate, dt);

    // update our predicted attitude based on our predicted angular velocity
    _predicted_euler_angle += _predicted_euler_rate * dt;

    // convert our predicted attitude to an acceleration vector assuming we are hovering
    const float pilot_predicted_cos_pitch_target = cosf(_predicted_euler_angle.y);
    const float pilot_predicted_accel_rgt_cms = GRAVITY_MSS*100.0f * tanf(_predicted_euler_angle.x)/pilot_predicted_cos_pitch_target;
    const float pilot_predicted_accel_fwd_cms = -GRAVITY_MSS*100.0f * tanf(_predicted_euler_angle.y);

    // rotate acceleration vectors input to lat/lon frame
    _predicted_accel.x = (pilot_predicted_accel_fwd_cms*_ahrs.cos_yaw() - pilot_predicted_accel_rgt_cms*_ahrs.sin_yaw());
    _predicted_accel.y = (pilot_predicted_accel_fwd_cms*_ahrs.sin_yaw() + pilot_predicted_accel_rgt_cms*_ahrs.cos_yaw());
}

/// get vector to stopping point based on a horizontal position and velocity
void AC_Loiter::get_stopping_point_xy(Vector3f& stopping_point) const
{
    _pos_control.get_stopping_point_xy(stopping_point);
}

/// get maximum lean angle when using loiter
float AC_Loiter::get_angle_max_cd() const
{
    if (is_zero(_angle_max)) {
        return MIN(_attitude_control.lean_angle_max(), _pos_control.get_lean_angle_max_cd()) * (2.0f/3.0f);
    }
    return MIN(_angle_max*100.0f, _pos_control.get_lean_angle_max_cd());
}

/// run the loiter controller
void AC_Loiter::update(float ekfGndSpdLimit, float ekfNavVelGainScaler)
{
    // calculate dt
    float dt = _pos_control.time_since_last_xy_update();
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // initialise pos controller speed and acceleration
    _pos_control.set_speed_xy(_speed_cms);
    _pos_control.set_accel_xy(_accel_cmss);

    calc_desired_velocity(dt,ekfGndSpdLimit);
    _pos_control.update_xy_controller(ekfNavVelGainScaler);
}

// sanity check parameters
void AC_Loiter::sanity_check_params()
{
    _speed_cms = MAX(_speed_cms, LOITER_SPEED_MIN);
    _accel_cmss = MIN(_accel_cmss, GRAVITY_MSS * 100.0f * tanf(ToRad(_attitude_control.lean_angle_max() * 0.01f)));
}

/// calc_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
///		updated velocity sent directly to position controller
void AC_Loiter::calc_desired_velocity(float nav_dt, float ekfGndSpdLimit)
{
    // calculate a loiter speed limit which is the minimum of the value set by the LOITER_SPEED
    // parameter and the value set by the EKF to observe optical flow limits
    float gnd_speed_limit_cms = MIN(_speed_cms, ekfGndSpdLimit*100.0f);
    gnd_speed_limit_cms = MAX(gnd_speed_limit_cms, LOITER_SPEED_MIN);

    float pilot_acceleration_max = GRAVITY_MSS*100.0f * tanf(radians(get_angle_max_cd()*0.01f));

    // range check nav_dt
    if (nav_dt < 0) {
        return;
    }

    _pos_control.set_speed_xy(gnd_speed_limit_cms);
    _pos_control.set_accel_xy(_accel_cmss);
    _pos_control.set_leash_length_xy(LOITER_POS_CORRECTION_MAX);

    // get loiters desired velocity from the position controller where it is being stored.
    const Vector3f &desired_vel_3d = _pos_control.get_desired_velocity();
    Vector2f desired_vel(desired_vel_3d.x,desired_vel_3d.y);

    // update the desired velocity using our predicted acceleration
    desired_vel.x += _predicted_accel.x * nav_dt;
    desired_vel.y += _predicted_accel.y * nav_dt;

    Vector2f loiter_accel_brake;
    float desired_speed = desired_vel.length();
    if (!is_zero(desired_speed)) {
        Vector2f desired_vel_norm = desired_vel/desired_speed;

        // TODO: consider using a velocity squared relationship like
        // pilot_acceleration_max*(desired_speed/gnd_speed_limit_cms)^2;
        // the drag characteristic of a multirotor should be examined to generate a curve
        // we could add a expo function here to fine tune it

        // calculate a drag acceleration based on the desired speed.
        float drag_decel = pilot_acceleration_max*desired_speed/gnd_speed_limit_cms;

        // calculate a braking acceleration if sticks are at zero
        float loiter_brake_accel = 0.0f;
        if (_desired_accel.is_zero()) {
            if ((AP_HAL::millis()-_brake_timer) > _brake_delay * 1000.0f) {
                float brake_gain = _pos_control.get_vel_xy_pid().kP() * 0.5f;
                loiter_brake_accel = constrain_float(AC_AttitudeControl::sqrt_controller(desired_speed, brake_gain, _brake_jerk_max_cmsss, nav_dt), 0.0f, _brake_accel_cmss);
            }
        } else {
            loiter_brake_accel = 0.0f;
            _brake_timer = AP_HAL::millis();
        }
        _brake_accel += constrain_float(loiter_brake_accel-_brake_accel, -_brake_jerk_max_cmsss*nav_dt, _brake_jerk_max_cmsss*nav_dt);
        loiter_accel_brake = desired_vel_norm*_brake_accel;

        // update the desired velocity using the drag and braking accelerations
        desired_speed = MAX(desired_speed-(drag_decel+_brake_accel)*nav_dt,0.0f);
        desired_vel = desired_vel_norm*desired_speed;
    }

    // add braking to the desired acceleration
    _desired_accel -= loiter_accel_brake;

    // Apply EKF limit to desired velocity -  this limit is calculated by the EKF and adjusted as required to ensure certain sensor limits are respected (eg optical flow sensing)
    float horizSpdDem = desired_vel.length();
    if (horizSpdDem > gnd_speed_limit_cms) {
        desired_vel.x = desired_vel.x * gnd_speed_limit_cms / horizSpdDem;
        desired_vel.y = desired_vel.y * gnd_speed_limit_cms / horizSpdDem;
    }

    // Limit the velocity to prevent fence violations
    // TODO: We need to also limit the _desired_accel
    if (_avoid != nullptr) {
        _avoid->adjust_velocity(_pos_control.get_pos_xy_p().kP(), _accel_cmss, desired_vel, nav_dt);
    }

    // send adjusted feed forward acceleration and velocity back to the Position Controller
    _pos_control.set_desired_accel_xy(_desired_accel.x, _desired_accel.y);
    _pos_control.set_desired_velocity_xy(desired_vel.x, desired_vel.y);
}
