#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_Avoidance/AC_Avoid.h>

class AC_Loiter
{
public:

    /// Constructor
    AC_Loiter(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /// provide pointer to avoidance library
    void set_avoidance(AC_Avoid* avoid_ptr) { _avoid = avoid_ptr; }

    /// init_target to a position in cm from ekf origin
    void init_target(const Vector3f& position);

    /// initialize's position and feed-forward velocity from current pos and velocity
    void init_target();

    /// reduce response for landing
    void soften_for_landing();

    /// set pilot desired acceleration in centi-degrees
    //   dt should be the time (in seconds) since the last call to this function
    void set_pilot_desired_acceleration(float euler_roll_angle_cd, float euler_pitch_angle_cd, float dt);

    /// gets pilot desired acceleration, body frame, [forward,right]
    Vector2f get_pilot_desired_acceleration() const { return Vector2f(_desired_accel.x, _desired_accel.y); }

    /// clear pilot desired acceleration
    void clear_pilot_desired_acceleration() { _desired_accel.zero(); }

    /// get vector to stopping point based on a horizontal position and velocity
    void get_stopping_point_xy(Vector3f& stopping_point) const;

    /// get horizontal distance to loiter target in cm
    float get_distance_to_target() const { return _pos_control.get_distance_to_target(); }

    /// get bearing to target in centi-degrees
    int32_t get_bearing_to_target() const { return _pos_control.get_bearing_to_target(); }

    /// get maximum lean angle when using loiter
    float get_angle_max_cd() const;

    /// run the loiter controller
    void update(float ekfGndSpdLimit, float ekfNavVelGainScaler);

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_roll() const { return _pos_control.get_roll(); }
    int32_t get_pitch() const { return _pos_control.get_pitch(); }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // sanity check parameters
    void sanity_check_params();

    /// updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
    ///		updated velocity sent directly to position controller
    void calc_desired_velocity(float nav_dt, float ekfGndSpdLimit);

    // references and pointers to external libraries
    const AP_InertialNav&   _inav;
    const AP_AHRS_View&     _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;
    AC_Avoid                *_avoid = nullptr;

    // parameters
    AP_Float    _angle_max;             // maximum pilot commanded angle in degrees. Set to zero for 2/3 Angle Max
    AP_Float    _speed_cms;             // maximum horizontal speed in cm/s while in loiter
    AP_Float    _accel_cmss;            // loiter's max acceleration in cm/s/s
    AP_Float    _brake_accel_cmss;      // loiter's acceleration during braking in cm/s/s
    AP_Float    _brake_jerk_max_cmsss;
    AP_Float    _brake_delay;           // delay (in seconds) before loiter braking begins after sticks are released

    // loiter controller internal variables
    Vector2f    _desired_accel;         // slewed pilot's desired acceleration in lat/lon frame
    Vector2f    _predicted_accel;
    Vector2f    _predicted_euler_angle;
    Vector2f    _predicted_euler_rate;
    float       _brake_timer;
    float       _brake_accel;
};
