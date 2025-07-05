#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_Avoidance/AC_Avoid.h>

class AC_Loiter
{
public:

    /// Constructor
    AC_Loiter(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /// initialise loiter target to a position in cm from ekf origin
    void init_target_cm(const Vector2f& position_neu_cm);

    /// initialize's position and feed-forward velocity from current pos and velocity
    void init_target();

    /// reduce response for landing
    void soften_for_landing();

    /// set pilot desired acceleration in radians
    //   dt should be the time (in seconds) since the last call to this function
    void set_pilot_desired_acceleration_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad);

    /// set pilot desired acceleration in centidegrees
    void set_pilot_desired_acceleration_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd);

    /// gets pilot desired acceleration in the earth frame
    Vector2f get_pilot_desired_acceleration_NE_cmss() const { return Vector2f{_desired_accel_ne_cmss.x, _desired_accel_ne_cmss.y}; }

    /// clear pilot desired acceleration
    void clear_pilot_desired_acceleration() {
        set_pilot_desired_acceleration_rad(0.0, 0.0);
    }

    /// get vector to stopping point based on a horizontal position and velocity
    void get_stopping_point_NE_cm(Vector2f& stopping_point) const;

    /// get horizontal distance to loiter target in cm
    float get_distance_to_target_cm() const { return _pos_control.get_pos_error_NE_cm(); }

    /// get bearing to target in centi-degrees
    float get_bearing_to_target_rad() const { return _pos_control.get_bearing_to_target_rad(); }

    /// get maximum lean angle when using loiter
    float get_angle_max_rad() const;
    float get_angle_max_cd() const;

    /// run the loiter controller
    void update(bool avoidance_on = true);

    //set maximum horizontal speed
    void set_speed_max_NE_cms(float speed_max_NE_cms);

    /// get desired roll, pitch which should be fed into stabilize controllers
    float get_roll_rad() const { return _pos_control.get_roll_rad(); }
    float get_pitch_rad() const { return _pos_control.get_pitch_rad(); }
    float get_roll_cd() const { return _pos_control.get_roll_cd(); }
    float get_pitch_cd() const { return _pos_control.get_pitch_cd(); }
    Vector3f get_thrust_vector() const { return _pos_control.get_thrust_vector(); }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // sanity check parameters
    void sanity_check_params();

    /// updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
    ///		updated velocity sent directly to position controller
    void calc_desired_velocity(bool avoidance_on = true);

    // references and pointers to external libraries
    const AP_AHRS_View&     _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;

    // parameters
    AP_Float    _angle_max_deg;         // maximum pilot commanded angle in degrees. Set to zero for 2/3 Angle Max
    AP_Float    _speed_max_ne_cms;      // maximum horizontal speed in cm/s while in loiter
    AP_Float    _accel_max_ne_cmss;     // loiter's max acceleration in cm/s/s
    AP_Float    _brake_accel_max_cmss;  // loiter's maximum acceleration during braking in cm/s/s
    AP_Float    _brake_jerk_max_cmsss;  // loiter's maximum jerk during braking in cm/s/s
    AP_Float    _brake_delay_s;         // delay (in seconds) before loiter braking begins after sticks are released

    // loiter controller internal variables
    Vector2f    _desired_accel_ne_cmss;     // slewed pilot's desired acceleration in lat/lon frame
    Vector2f    _predicted_accel_ne_cmss;   // predicted acceleration in lat/lon frame based on pilot's desired acceleration
    Vector2f    _predicted_euler_angle_rad; // predicted roll/pitch angles in radians based on pilot's desired acceleration
    Vector2f    _predicted_euler_rate;      // predicted roll/pitch rates in radians/sec based on pilot's desired acceleration
    uint32_t    _brake_timer_ms;            // system time that brake was initiated
    float       _brake_accel_cmss;          // acceleration due to braking from previous iteration (used for jerk limiting)
};
