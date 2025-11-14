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

    // Sets the initial loiter target position in meters from the EKF origin.
    // - position_ne_m: horizontal position in the NE frame, in meters.
    // - Initializes internal control state including acceleration targets and feed-forward planning.
    void init_target_m(const Vector2p& position_ne_m);

    // Initializes the loiter controller using the current position and velocity.
    // Updates feed-forward velocity, predicted acceleration, and resets control state.
    void init_target();

    // Reduces loiter responsiveness for smoother descent during landing.
    // Internally softens horizontal control gains.
    void soften_for_landing();

    // Sets pilot desired acceleration using Euler angles in centidegrees.
    // See set_pilot_desired_acceleration_rad() for full details.
    void set_pilot_desired_acceleration_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd);

    // Sets pilot desired acceleration using Euler angles in radians.
    // - Internally computes a smoothed acceleration vector based on predictive rate shaping.
    // - Inputs: `euler_roll_angle_rad`, `euler_pitch_angle_rad` in radians.
    // - Applies internal shaping using the current attitude controller dt.
    void set_pilot_desired_acceleration_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad);

    // Returns pilot-requested horizontal acceleration in the NE frame in m/s².
    // This is the internally computed and smoothed acceleration vector applied by the loiter controller.
    const Vector2f& get_pilot_desired_acceleration_NE_mss() const { return _desired_accel_ne_mss; }

    // Clears any pilot-requested acceleration by setting roll and pitch inputs to zero.
    void clear_pilot_desired_acceleration() { set_pilot_desired_acceleration_rad(0.0, 0.0); }

    // Calculates the expected stopping point based on current velocity and position in the NE frame.
    // Result is returned in meters.
    // Uses the position controller’s deceleration model.
    void get_stopping_point_NE_m(Vector2f& stopping_point_ne_m) const;

    // Returns the horizontal distance to the loiter target in meters.
    // Computed using the NE position error from the position controller.
    float get_distance_to_target_m() const { return _pos_control.get_pos_error_NE_m(); }

    // Returns the bearing from current position to the loiter target in radians.
    // Bearing is relative to true north, using the NE frame.
    float get_bearing_to_target_rad() const { return _pos_control.get_bearing_to_target_rad(); }

    // Returns the maximum pilot-commanded lean angle in centidegrees.
    // See get_angle_max_rad() for full details.
    float get_angle_max_cd() const;

    // Returns the maximum pilot-commanded lean angle in radians.
    // - If `_angle_max_deg` is zero, this returns 2/3 of the limiting PSC angle.
    // - Otherwise, returns the minimum of `_angle_max_deg` and PSC’s configured angle limit.
    float get_angle_max_rad() const;

    // Runs the loiter control loop, computing desired acceleration and updating position control.
    // If `avoidance_on` is true, velocity is adjusted using avoidance logic before being applied.
    void update(bool avoidance_on = true);

    // Sets the maximum allowed horizontal loiter speed in m/s.
    // Internally converts to cm/s and clamps to a minimum of LOITER_SPEED_MIN_CMS.
    void set_speed_max_NE_ms(float speed_max_NE_ms);

    // Returns the desired roll angle in centidegrees from the loiter controller.
    float get_roll_cd() const { return rad_to_cd(get_roll_rad()); }

    // Returns the desired pitch angle in centidegrees from the loiter controller.
    float get_pitch_cd() const { return rad_to_cd(get_pitch_rad()); }

    // Returns the desired roll angle in radians from the loiter controller.
    float get_roll_rad() const { return _pos_control.get_roll_rad(); }

    // Returns the desired pitch angle in radians from the loiter controller.
    float get_pitch_rad() const { return _pos_control.get_pitch_rad(); }

    // Returns the desired 3D thrust vector from the loiter controller for attitude control.
    // Directional only; magnitude is handled by the attitude controller.
    Vector3f get_thrust_vector() const { return _pos_control.get_thrust_vector(); }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Ensures internal parameters are within valid safety limits.
    // Applies min/max constraints on speed and acceleration settings.
    void sanity_check_params();

    // Updates feed-forward velocity using pilot-requested acceleration and braking logic.
    // - Applies drag and braking forces when sticks are released.
    // - Velocity is adjusted for fence/avoidance if enabled.
    // - Resulting velocity and acceleration are sent to the position controller.
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
    AP_Int8     _options;               // Loiter options bit mask

    // Bitfields of LOITER_OPTIONS
    enum class LoiterOption {
        COORDINATED_TURN_ENABLED    = (1U << 0),    // Enable Coordinated Turn
    };
    bool loiter_option_is_set(LoiterOption option) const;

    // loiter controller internal variables
    Vector2f    _desired_accel_ne_mss;      // Pilot-requested horizontal acceleration in m/s² (after smoothing), in the NE (horizontal) frame.
    Vector2f    _predicted_accel_ne_mss;    // Predicted acceleration in m/s² based on internal rate shaping of pilot input.
    Vector2f    _predicted_euler_angle_rad; // Predicted roll/pitch angles (in radians) used for rate shaping of pilot input.
    Vector2f    _predicted_euler_rate;      // Predicted roll/pitch angular rates (in rad/s) for pilot acceleration shaping.
    uint32_t    _brake_timer_ms;            // Timestamp (in ms) when braking logic was last triggered (sticks released).
    float       _brake_accel_mss;           // Current braking acceleration in m/s², updated using jerk limits over time.
};
