#include <AP_HAL/AP_HAL.h>
#include "AC_Loiter.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AC_Avoidance/AC_Avoid.h>

extern const AP_HAL::HAL& hal;

#define LOITER_SPEED_DEFAULT                1250.0f // default loiter speed in cm/s
#define LOITER_SPEED_MIN                    20.0f   // minimum loiter speed in cm/s
#define LOITER_ACCEL_MAX_DEFAULT            500.0f  // default acceleration in loiter mode
#define LOITER_BRAKE_ACCEL_DEFAULT          250.0f  // minimum acceleration in loiter mode
#define LOITER_BRAKE_JERK_DEFAULT           500.0f  // maximum jerk in cm/s³ in loiter mode
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
    AP_GROUPINFO("ANG_MAX",  1, AC_Loiter, _angle_max_deg, 0.0f),

    // @Param: SPEED
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: cm/s
    // @Range: 20 3500
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED", 2, AC_Loiter, _speed_max_ne_cms, LOITER_SPEED_DEFAULT),

    // @Param: ACC_MAX
    // @DisplayName: Loiter maximum correction acceleration
    // @Description: Loiter maximum correction acceleration in cm/s².  Higher values cause the copter to correct position errors more aggressively.
    // @Units: cm/s²
    // @Range: 100 981
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ACC_MAX", 3, AC_Loiter, _accel_max_ne_cmss, LOITER_ACCEL_MAX_DEFAULT),

    // @Param: BRK_ACCEL
    // @DisplayName: Loiter braking acceleration
    // @Description: Loiter braking acceleration in cm/s². Higher values stop the copter more quickly when the stick is centered.
    // @Units: cm/s²
    // @Range: 25 250
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("BRK_ACCEL", 4, AC_Loiter, _brake_accel_max_cmss, LOITER_BRAKE_ACCEL_DEFAULT),

    // @Param: BRK_JERK
    // @DisplayName: Loiter braking jerk
    // @Description: Loiter braking jerk in cm/s³. Higher values will remove braking faster if the pilot moves the sticks during a braking maneuver.
    // @Units: cm/s³
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
    AP_GROUPINFO("BRK_DELAY",  6, AC_Loiter, _brake_delay_s, LOITER_BRAKE_START_DELAY_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Loiter::AC_Loiter(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/// initialise loiter target to a position in cm from ekf origin
void AC_Loiter::init_target_cm(const Vector2f& position_ne_cm)
{
    sanity_check_params();

    // initialise position controller speed and acceleration
    _pos_control.set_correction_speed_accel_NE_cm(LOITER_VEL_CORRECTION_MAX, _accel_max_ne_cmss);
    _pos_control.set_pos_error_max_NE_cm(LOITER_POS_CORRECTION_MAX);

    // initialise position controller
    _pos_control.init_NE_controller_stopping_point();

    // initialise desired acceleration and angles to zero to remain on station
    _predicted_accel_ne_cmss.zero();
    _desired_accel_ne_cmss.zero();
    _predicted_euler_angle_rad.zero();
    _brake_accel_cmss = 0.0f;

    // set target position
    _pos_control.set_pos_desired_NE_cm(position_ne_cm);
}

/// initialize's position and feed-forward velocity from current pos and velocity
void AC_Loiter::init_target()
{
    sanity_check_params();

    // initialise position controller speed and acceleration
    _pos_control.set_correction_speed_accel_NE_cm(LOITER_VEL_CORRECTION_MAX, _accel_max_ne_cmss);
    _pos_control.set_pos_error_max_NE_cm(LOITER_POS_CORRECTION_MAX);

    // initialise position controller and move target accelerations smoothly towards zero
    _pos_control.relax_velocity_controller_NE();

    // initialise predicted acceleration and angles from the position controller
    _predicted_accel_ne_cmss = _pos_control.get_accel_target_NEU_cmss().xy();
    _predicted_euler_angle_rad.x = _pos_control.get_roll_rad();
    _predicted_euler_angle_rad.y = _pos_control.get_pitch_rad();
    _brake_accel_cmss = 0.0f;
}

/// reduce response for landing
void AC_Loiter::soften_for_landing()
{
    _pos_control.soften_for_landing_NE();
}

/// set pilot desired acceleration in centidegrees
void AC_Loiter::set_pilot_desired_acceleration_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd)
{
    set_pilot_desired_acceleration_rad(cd_to_rad(euler_roll_angle_cd), cd_to_rad(euler_pitch_angle_cd));
}

/// set pilot desired acceleration in radians
//   dt should be the time (in seconds) since the last call to this function
void AC_Loiter::set_pilot_desired_acceleration_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad)
{
    const float dt = _attitude_control.get_dt_s();

    // convert our desired attitude to an acceleration vector assuming we are not accelerating vertically
    const Vector3f desired_euler_rad {euler_roll_angle_rad, euler_pitch_angle_rad, _ahrs.yaw};
    const Vector3f desired_accel_NEU_cmss = _pos_control.lean_angles_rad_to_accel_NEU_cmss(desired_euler_rad);

    _desired_accel_ne_cmss.x = desired_accel_NEU_cmss.x;
    _desired_accel_ne_cmss.y = desired_accel_NEU_cmss.y;

    // difference between where we think we should be and where we want to be
    Vector2f angle_error(wrap_PI(euler_roll_angle_rad - _predicted_euler_angle_rad.x), wrap_PI(euler_pitch_angle_rad - _predicted_euler_angle_rad.y));

    // calculate the angular velocity that we would expect given our desired and predicted attitude
    _attitude_control.input_shaping_rate_predictor(angle_error, _predicted_euler_rate, dt);

    // update our predicted attitude based on our predicted angular velocity
    _predicted_euler_angle_rad += _predicted_euler_rate * dt;

    // convert our predicted attitude to an acceleration vector assuming we are not accelerating vertically
    const Vector3f predicted_euler_rad {_predicted_euler_angle_rad.x, _predicted_euler_angle_rad.y, _ahrs.yaw};
    const Vector3f predicted_accel = _pos_control.lean_angles_rad_to_accel_NEU_cmss(predicted_euler_rad);

    _predicted_accel_ne_cmss.x = predicted_accel.x;
    _predicted_accel_ne_cmss.y = predicted_accel.y;
}

/// get vector to stopping point based on a horizontal position and velocity
void AC_Loiter::get_stopping_point_NE_cm(Vector2f& stopping_point_ne_cm) const
{
    Vector2p stop_ne_cm;
    _pos_control.get_stopping_point_NE_cm(stop_ne_cm);
    stopping_point_ne_cm = stop_ne_cm.tofloat();
}

/// get maximum lean angle when using loiter
float AC_Loiter::get_angle_max_rad() const
{
    if (!is_positive(_angle_max_deg)) {
        return MIN(_attitude_control.lean_angle_max_rad(), _pos_control.get_lean_angle_max_rad()) * (2.0f / 3.0f);
    }
    return MIN(radians(_angle_max_deg), _pos_control.get_lean_angle_max_rad());
}
float AC_Loiter::get_angle_max_cd() const
{
    return rad_to_cd(get_angle_max_rad());
}

/// run the loiter controller
void AC_Loiter::update(bool avoidance_on)
{
    calc_desired_velocity(avoidance_on);
    _pos_control.update_NE_controller();
}

//set maximum horizontal speed
void AC_Loiter::set_speed_max_NE_cms(float speed_max_ne_cms)
{
    _speed_max_ne_cms.set(MAX(speed_max_ne_cms, LOITER_SPEED_MIN));
}

// sanity check parameters
void AC_Loiter::sanity_check_params()
{
    _speed_max_ne_cms.set(MAX(_speed_max_ne_cms, LOITER_SPEED_MIN));
    _accel_max_ne_cmss.set(MIN(_accel_max_ne_cmss, GRAVITY_MSS * 100.0f * tanf(_attitude_control.lean_angle_max_rad())));
}

/// calc_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
///		updated velocity sent directly to position controller
void AC_Loiter::calc_desired_velocity(bool avoidance_on)
{
    float ekfGndSpdLimit, ahrsControlScaleXY;
    AP::ahrs().getControlLimits(ekfGndSpdLimit, ahrsControlScaleXY);

    const float dt = _pos_control.get_dt_s();

    // calculate a loiter speed limit which is the minimum of the value set by the LOITER_SPEED
    // parameter and the value set by the EKF to observe optical flow limits
    float gnd_speed_limit_cms = MIN(_speed_max_ne_cms, ekfGndSpdLimit * 100.0f);
    gnd_speed_limit_cms = MAX(gnd_speed_limit_cms, LOITER_SPEED_MIN);

    float pilot_acceleration_max = angle_rad_to_accel_mss(get_angle_max_rad()) * 100;

    // range check dt
    if (is_negative(dt)) {
        return;
    }

    // get loiters desired velocity from the position controller where it is being stored.
    Vector2f desired_vel = _pos_control.get_vel_desired_NEU_cms().xy();

    // update the desired velocity using our predicted acceleration
    desired_vel += _predicted_accel_ne_cmss * dt;

    Vector2f loiter_accel_brake;
    float desired_speed = desired_vel.length();
    if (!is_zero(desired_speed)) {
        Vector2f desired_vel_norm = desired_vel / desired_speed;

        // calculate a drag acceleration based on the desired speed.
        float drag_decel = pilot_acceleration_max * desired_speed / gnd_speed_limit_cms;

        // calculate a braking acceleration if sticks are at zero
        float loiter_brake_accel = 0.0f;
        if (_desired_accel_ne_cmss.is_zero()) {
            if ((AP_HAL::millis() - _brake_timer_ms) > _brake_delay_s * 1000.0f) {
                float brake_gain = _pos_control.get_vel_NE_pid().kP() * 0.5f;
                loiter_brake_accel = constrain_float(sqrt_controller(desired_speed, brake_gain, _brake_jerk_max_cmsss, dt), 0.0f, _brake_accel_max_cmss);
            }
        } else {
            loiter_brake_accel = 0.0f;
            _brake_timer_ms = AP_HAL::millis();
        }
        _brake_accel_cmss += constrain_float(loiter_brake_accel - _brake_accel_cmss, -_brake_jerk_max_cmsss * dt, _brake_jerk_max_cmsss * dt);
        loiter_accel_brake = desired_vel_norm * _brake_accel_cmss;

        // update the desired velocity using the drag and braking accelerations
        desired_speed = MAX(desired_speed - (drag_decel + _brake_accel_cmss) * dt, 0.0f);
        desired_vel = desired_vel_norm * desired_speed;
    }

    // add braking to the desired acceleration
    _desired_accel_ne_cmss -= loiter_accel_brake;

    // Apply EKF limit to desired velocity -  this limit is calculated by the EKF and adjusted as required to ensure certain sensor limits are respected (eg optical flow sensing)
    float horizSpdDem = desired_vel.length();
    if (horizSpdDem > gnd_speed_limit_cms) {
        desired_vel = desired_vel * gnd_speed_limit_cms / horizSpdDem;
    }

#if AP_AVOIDANCE_ENABLED && !APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    if (avoidance_on) {
        // Limit the velocity to prevent fence violations
        // TODO: We need to also limit the _desired_accel_ne_cmss
        AC_Avoid *_avoid = AP::ac_avoid();
        if (_avoid != nullptr) {
            Vector3f avoidance_vel_3d{desired_vel.x, desired_vel.y, 0.0f};
            _avoid->adjust_velocity(avoidance_vel_3d, _pos_control.get_pos_NE_p().kP(), _accel_max_ne_cmss, _pos_control.get_pos_U_p().kP(), _pos_control.get_max_accel_U_cmss(), dt);
            desired_vel = Vector2f{avoidance_vel_3d.x, avoidance_vel_3d.y};
        }
    }
#endif // !APM_BUILD_ArduPlane

    // get loiters desired velocity from the position controller where it is being stored.
    Vector2p desired_pos = _pos_control.get_pos_desired_NEU_cm().xy();

    // update the desired position using our desired velocity and acceleration
    desired_pos += (desired_vel * dt).topostype();

    // send adjusted feed forward acceleration and velocity back to the Position Controller
    _pos_control.set_pos_vel_accel_NE_cm(desired_pos, desired_vel, _desired_accel_ne_cmss);
}
