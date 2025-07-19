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
    void init_target_m(const Vector2f& position_neu_m);

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
    Vector2f get_pilot_desired_acceleration_NE_cmss() const { return get_pilot_desired_acceleration_NE_mss() * 100.0; }
    Vector2f get_pilot_desired_acceleration_NE_mss() const { return _desired_accel_ne_mss; }

    /// clear pilot desired acceleration
    void clear_pilot_desired_acceleration() {
        set_pilot_desired_acceleration_rad(0.0, 0.0);
    }

    /// get vector to stopping point based on a horizontal position and velocity
    void get_stopping_point_NE_cm(Vector2f& stopping_point_ne_cm) const;
    void get_stopping_point_NE_m(Vector2f& stopping_point_ne_m) const;

    /// get horizontal distance to loiter target in cm
    float get_distance_to_target_cm() const { return get_distance_to_target_m() * 100.0; }
    float get_distance_to_target_m() const { return _pos_control.get_pos_error_NE_cm(); }

    /// get bearing to target in centi-degrees
    float get_bearing_to_target_rad() const { return _pos_control.get_bearing_to_target_rad(); }

    /// get maximum lean angle when using loiter
    float get_angle_max_rad() const;
    float get_angle_max_cd() const;

    /// run the loiter controller
    void update(bool avoidance_on = true);

    //set maximum horizontal speed
    void set_speed_max_NE_cms(float speed_max_NE_cms);
    void set_speed_max_NE_ms(float speed_max_NE_ms);

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
    AP_Float    _angle_max_deg;         // Maximum pilot-commanded lean angle in degrees. Set to zero to default to 2/3 of PSC_ANGLE_MAX (or Q_ANGLE_MAX for QuadPlane).
    AP_Float    _speed_max_ne_cms;      // Maximum horizontal speed in cm/s while in loiter mode. Used to limit both user and internal trajectory velocities.
    AP_Float    _accel_max_ne_cmss;     // Maximum horizontal acceleration (in cm/s²) applied during normal loiter corrections.
    AP_Float    _brake_accel_max_cmss;  // Maximum braking acceleration (in cm/s²) applied when pilot sticks are released.
    AP_Float    _brake_jerk_max_cmsss;  // Maximum braking jerk (in cm/s³) applied during braking transitions after pilot release.
    AP_Float    _brake_delay_s;         // Delay in seconds before braking begins after sticks are centered. Prevents premature deceleration during brief pauses.

    // loiter controller internal variables
    Vector2f    _desired_accel_ne_mss;      // Pilot-requested horizontal acceleration in m/s² (after smoothing), in the NE (horizontal) frame.
    Vector2f    _predicted_accel_ne_mss;    // Predicted acceleration in m/s² based on internal rate shaping of pilot input.
    Vector2f    _predicted_euler_angle_rad; // Predicted roll/pitch angles (in radians) used for rate shaping of pilot input.
    Vector2f    _predicted_euler_rate;      // Predicted roll/pitch angular rates (in rad/s) for pilot acceleration shaping.
    uint32_t    _brake_timer_ms;            // Timestamp (in ms) when braking logic was last triggered (sticks released).
    float       _brake_accel_mss;           // Current braking acceleration in m/s², updated using jerk limits over time.
};
