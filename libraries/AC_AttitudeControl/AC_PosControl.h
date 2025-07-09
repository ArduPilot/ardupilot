#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P.h>            // P library
#include <AC_PID/AC_PID.h>          // PID library
#include <AC_PID/AC_P_1D.h>         // P library (1-axis)
#include <AC_PID/AC_P_2D.h>         // P library (2-axis)
#include <AC_PID/AC_PI_2D.h>        // PI library (2-axis)
#include <AC_PID/AC_PID_Basic.h>    // PID library (1-axis)
#include <AC_PID/AC_PID_2D.h>       // PID library (2-axis)
#include <AP_Scripting/AP_Scripting_config.h>
#include "AC_AttitudeControl.h"     // Attitude control library

#include <AP_Logger/LogStructure.h>

// position controller default definitions
#define POSCONTROL_ACCEL_NE_MSS                 1.0f    // default horizontal acceleration in m/s². This is overwritten by waypoint and loiter controllers
#define POSCONTROL_JERK_NE_MSSS                 5.0f    // default horizontal jerk m/s³

#define POSCONTROL_STOPPING_DIST_UP_MAX_M       3.0f    // max stopping distance (in m) vertically while climbing
#define POSCONTROL_STOPPING_DIST_DOWN_MAX_M     2.0f    // max stopping distance (in m) vertically while descending

#define POSCONTROL_SPEED_MS                     5.0f    // default horizontal speed in m/s
#define POSCONTROL_SPEED_DOWN_MS                -1.5f   // default descent rate in m/s
#define POSCONTROL_SPEED_UP_MS                  2.5f    // default climb rate in m/s

#define POSCONTROL_ACCEL_U_MSS                  2.5f    // default vertical acceleration in m/s²
#define POSCONTROL_JERK_U_MSSS                  5.0f    // default vertical jerk m/s³

#define POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ      2.0f    // low-pass filter on acceleration error (unit: Hz)

#define POSCONTROL_OVERSPEED_GAIN_U             2.0f    // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range

#define POSCONTROL_RELAX_TC                     0.16f   // This is used to decay the I term to 5% in half a second

class AC_PosControl
{
public:

    /// Constructor
    AC_PosControl(AP_AHRS_View& ahrs, const class AP_Motors& motors, AC_AttitudeControl& attitude_control);

    // do not allow copying
    CLASS_NO_COPY(AC_PosControl);

    // Sets the timestep between controller updates (seconds).
    // This should match the IMU sample time used by the controller.
    void set_dt_s(float dt) { _dt_s = dt; }

    // Returns the timestep used in the controller update (seconds).
    float get_dt_s() const { return _dt_s; }

    // Updates internal NEU position and velocity estimates from AHRS.
    // Falls back to vertical-only data if horizontal velocity or position is invalid or vibration forces it.
    // When high_vibes is true, forces use of vertical fallback for velocity.
    void update_estimates(bool high_vibes = false);

    // Returns the jerk limit for horizontal path shaping in cm/s³.
    // See get_shaping_jerk_NE_msss() for full details.
    float get_shaping_jerk_NE_cmsss() const { return get_shaping_jerk_NE_msss() * 100.0; }

    // Returns the jerk limit for horizontal path shaping in m/s³.
    // Used to constrain acceleration changes in trajectory generation.
    float get_shaping_jerk_NE_msss() const { return _shaping_jerk_ne_msss; }


    ///
    /// 3D position shaper
    ///

    // Sets a new NEU position target in centimeters and computes a jerk-limited trajectory.
    // Also updates vertical buffer logic using terrain altitude target.
    // See input_pos_NEU_m() for full details.
    void input_pos_NEU_cm(const Vector3p& pos_neu_cm, float pos_terrain_target_alt_cm, float terrain_buffer_cm);

    // Sets a new NEU position target in meters and computes a jerk-limited trajectory.
    // Updates internal acceleration commands using a smooth kinematic path constrained
    // by configured acceleration and jerk limits. Terrain margin is used to constrain
    // horizontal velocity to avoid vertical buffer violation.
    void input_pos_NEU_m(const Vector3p& pos_neu_m, float pos_terrain_target_alt_m, float terrain_buffer_m);

    // Returns a scaling factor for horizontal velocity in cm/s to respect vertical terrain buffer.
    // See pos_terrain_U_scaler_m() for full details.
    float pos_terrain_U_scaler_cm(float pos_terrain_u_cm, float pos_terrain_u_buffer_cm) const;

    // Returns a scaling factor for horizontal velocity in m/s to ensure
    // the vertical controller maintains a safe distance above terrain.
    float pos_terrain_U_scaler_m(float pos_terrain_u_m, float pos_terrain_u_buffer_m) const;

    ///
    /// Lateral position controller
    ///

    // Sets maximum horizontal speed (cm/s) and acceleration (cm/s²) for NE-axis shaping.
    // Can be called anytime; transitions are handled smoothly.
    // See set_max_speed_accel_NE_m() for full details.
    void set_max_speed_accel_NE_cm(float speed_cms, float accel_cmss);

    // Sets maximum horizontal speed (m/s) and acceleration (m/s²) for NE-axis shaping.
    // These values constrain the kinematic trajectory used by the lateral controller.
    void set_max_speed_accel_NE_m(float speed_ms, float accel_mss);

    // Sets horizontal correction limits for velocity (cm/s) and acceleration (cm/s²).
    // Should be called only during initialization to avoid control discontinuities.
    // See set_correction_speed_accel_NE_m() for full details.
    void set_correction_speed_accel_NE_cm(float speed_cms, float accel_cmss);

    // Sets horizontal correction limits for velocity (m/s) and acceleration (m/s²).
    // These values constrain the PID correction path, not the desired trajectory.
    void set_correction_speed_accel_NE_m(float speed_ms, float accel_mss);

    // Returns maximum horizontal speed in cm/s.
    // See get_max_speed_NE_ms() for full details.
    float get_max_speed_NE_cms() const { return get_max_speed_NE_ms() * 100.0; }

    // Returns maximum horizontal speed in m/s used for shaping the trajectory.
    float get_max_speed_NE_ms() const { return _vel_max_ne_ms; }

    // Returns maximum horizontal acceleration in cm/s².
    // See get_max_accel_NE_mss() for full details.
    float get_max_accel_NE_cmss() const { return get_max_accel_NE_mss() * 100.0; }

    // Returns maximum horizontal acceleration in m/s² used for trajectory shaping.
    float get_max_accel_NE_mss() const { return _accel_max_ne_mss; }

    // Sets maximum allowed horizontal position error in cm.
    // See set_pos_error_max_NE_m() for full details.
    void set_pos_error_max_NE_cm(float error_max_cm) { set_pos_error_max_NE_m(error_max_cm * 0.01); }

    // Sets maximum allowed horizontal position error in meters.
    // Used to constrain the output of the horizontal position P controller.
    void set_pos_error_max_NE_m(float error_max_m) { _p_pos_ne_m.set_error_max(error_max_m); }

    // Returns maximum allowed horizontal position error in cm.
    // See get_pos_error_max_NE_m() for full details.
    float get_pos_error_max_NE_cm() const { return get_pos_error_max_NE_m() * 100.0; }

    // Returns maximum allowed horizontal position error in meters.
    float get_pos_error_max_NE_m() const { return _p_pos_ne_m.get_error_max(); }

    // Initializes NE controller to a stationary stopping point with zero velocity and acceleration.
    // Use when the expected trajectory begins at rest but the starting position is unspecified.
    // The starting position can be retrieved with get_pos_target_NEU_m().
    void init_NE_controller_stopping_point();

    // Smoothly decays NE acceleration over time to zero while maintaining current velocity and position.
    // Reduces output acceleration by ~95% over 0.5 seconds to avoid abrupt transitions.
    void relax_velocity_controller_NE();

    // Softens NE controller for landing by reducing position error and suppressing I-term windup.
    // Used to make descent behavior more stable near ground contact.
    void soften_for_landing_NE();

    // Fully initializes the NE controller with current position, velocity, acceleration, and attitude.
    // Intended for normal startup when the full state is known.
    // Private function shared by other NE initializers.
    void init_NE_controller();

    // Sets the desired NE-plane acceleration in cm/s² using jerk-limited shaping.
    // See input_accel_NE_m() for full details.
    void input_accel_NE_cm(const Vector3f& accel_neu_cmsss);

    // Sets the desired NE-plane acceleration in m/s² using jerk-limited shaping.
    // Smoothly transitions to the specified acceleration from current kinematic state.
    // Constraints: max acceleration and jerk set via set_max_speed_accel_NE_m().
    void input_accel_NE_m(const Vector3f& accel_neu_msss);

    // Sets desired NE-plane velocity and acceleration (cm/s, cm/s²) using jerk-limited shaping.
    // See input_vel_accel_NE_m() for full details.
    void input_vel_accel_NE_cm(Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output = true);

    // Sets desired NE-plane velocity and acceleration (m/s, m/s²) using jerk-limited shaping.
    // Calculates target acceleration using current kinematics constrained by acceleration and jerk limits.
    // If `limit_output` is true, applies limits to total command (desired + correction).
    void input_vel_accel_NE_m(Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss, bool limit_output = true);

    // Sets desired NE position, velocity, and acceleration (cm, cm/s, cm/s²) with jerk-limited shaping.
    // See input_pos_vel_accel_NE_m() for full details.
    void input_pos_vel_accel_NE_cm(Vector2p& pos_ne_cm, Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output = true);

    // Sets desired NE position, velocity, and acceleration (m, m/s, m/s²) with jerk-limited shaping.
    // Calculates acceleration trajectory based on current kinematics and constraints.
    // If `limit_output` is true, limits apply to full command (desired + correction).
    void input_pos_vel_accel_NE_m(Vector2p& pos_ne_m, Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss, bool limit_output = true);

    // Returns true if the NE position controller has run in the last 5 control loop cycles.
    bool is_active_NE() const;

    // Disables NE position correction by setting the target position to the current position.
    // Useful to freeze positional control without disrupting velocity control.
    void stop_pos_NE_stabilisation();

    // Disables NE position and velocity correction by setting target values to current state.
    // Useful to prevent further corrections and freeze motion stabilization in NE axes.
    void stop_vel_NE_stabilisation();

    // Applies a scalar multiplier to the NE control loop.
    // Set to 0 to disable lateral control; 1 for full authority.
    void set_NE_control_scale_factor(float ne_control_scale_factor) {
        _ne_control_scale_factor = ne_control_scale_factor;
    }

    // Runs the NE-axis position controller, computing output acceleration from position and velocity errors.
    // Uses P and PID controllers to generate corrections which are added to feedforward velocity/acceleration.
    // Requires all desired targets to be pre-set using the input_* or set_* methods.
    void update_NE_controller();

    ///
    /// Vertical position controller
    ///

    // Sets maximum climb/descent rate (cm/s) and vertical acceleration (cm/s²) for the U-axis.
    // Descent rate may be positive or negative and is always interpreted as a descent.
    // See set_max_speed_accel_U_m() for full details.
    void set_max_speed_accel_U_cm(float speed_down_cms, float speed_up_cms, float accel_cmss);

    // Sets maximum climb/descent rate (m/s) and vertical acceleration (m/s²) for the U-axis.
    // These values are used for jerk-limited kinematic shaping of the vertical trajectory.
    void set_max_speed_accel_U_m(float speed_down_ms, float speed_up_ms, float accel_mss);

    // Sets vertical correction velocity and acceleration limits (cm/s, cm/s²).
    // Should only be called during initialization to avoid discontinuities.
    // See set_correction_speed_accel_U_mss() for full details.
    void set_correction_speed_accel_U_cmss(float speed_down_cms, float speed_up_cms, float accel_cmss);

    // Sets vertical correction velocity and acceleration limits (m/s, m/s²).
    // These values constrain the correction output of the PID controller.
    void set_correction_speed_accel_U_mss(float speed_down_ms, float speed_up_ms, float accel_mss);

    // Returns maximum vertical acceleration in cm/s².
    // See get_max_accel_U_mss() for full details.
    float get_max_accel_U_cmss() const { return get_max_accel_U_mss() * 100.0; }

    // Returns maximum vertical acceleration in m/s² used for shaping the climb/descent trajectory.
    float get_max_accel_U_mss() const { return _accel_max_u_mss; }

    // Returns maximum allowed positive (upward) position error in cm.
    // See get_pos_error_up_m() for full details.
    float get_pos_error_up_cm() const { return get_pos_error_up_m() * 100.0; }

    // Returns maximum allowed positive (upward) position error in meters.
    float get_pos_error_up_m() const { return _p_pos_u_m.get_error_max(); }

    // Returns maximum allowed negative (downward) position error in cm.
    // See get_pos_error_down_m() for full details.
    float get_pos_error_down_cm() const { return get_pos_error_down_m() * 100.0; }

    // Returns maximum allowed negative (downward) position error in meters.
    float get_pos_error_down_m() const { return _p_pos_u_m.get_error_min(); }

    // Returns maximum climb rate in cm/s.
    // See get_max_speed_up_ms() for full details.
    float get_max_speed_up_cms() const { return get_max_speed_up_ms() * 100.0; }

    // Returns maximum climb rate in m/s used for shaping the vertical trajectory.
    float get_max_speed_up_ms() const { return _vel_max_up_ms; }

    // Returns maximum descent rate in cm/s (typically negative).
    // See get_max_speed_down_ms() for full details.
    float get_max_speed_down_cms() const { return get_max_speed_down_ms() * 100.0; }

    /// Returns maximum descent rate in m/s (typically negative).
    float get_max_speed_down_ms() const { return _vel_max_down_ms; }

    // Initializes U-axis controller to current position, velocity, and acceleration, disallowing descent.
    // Used for takeoff or hold scenarios where downward motion is prohibited.
    void init_U_controller_no_descent();

    // Initializes U-axis controller to a stationary stopping point with zero velocity and acceleration.
    // Used when the trajectory starts at rest but the initial altitude is unspecified.
    // The resulting position target can be retrieved with get_pos_target_NEU_m().
    void init_U_controller_stopping_point();

    // Smoothly decays U-axis acceleration to zero over time while maintaining current vertical velocity.
    // Reduces requested acceleration by ~95% every 0.5 seconds to avoid abrupt transitions.
    // `throttle_setting` is used to determine whether to preserve positive acceleration in low-thrust cases.
    void relax_U_controller(float throttle_setting);

    // Fully initializes the U-axis controller with current position, velocity, acceleration, and attitude.
    // Used during standard controller activation when full state is known.
    // Private function shared by other vertical initializers.
    void init_U_controller();

    // Sets the desired vertical acceleration in cm/s² using jerk-limited shaping.
    // See input_accel_U_m() for full details.
    virtual void input_accel_U_cm(float accel_u_cmss);

    // Sets the desired vertical acceleration in m/s² using jerk-limited shaping.
    // Smoothly transitions to the target acceleration from current kinematic state.
    // Constraints: max acceleration and jerk set via set_max_speed_accel_U_m().
    virtual void input_accel_U_m(float accel_u_mss);

    // Sets desired vertical velocity and acceleration (cm/s, cm/s²) using jerk-limited shaping.
    // See input_vel_accel_U_m() for full details.
    virtual void input_vel_accel_U_cm(float &vel_u_cms, float accel_u_cmss, bool limit_output = true);

    // Sets desired vertical velocity and acceleration (m/s, m/s²) using jerk-limited shaping.
    // Calculates required acceleration using current vertical kinematics.
    // If `limit_output` is true, limits apply to the combined (desired + correction) command.
    virtual void input_vel_accel_U_m(float &vel_u_ms, float accel_u_mss, bool limit_output = true);

    // Generates a vertical trajectory using the given climb rate in cm/s and jerk-limited shaping.
    // Adjusts the internal target altitude based on integrated climb rate.
    // See set_pos_target_U_from_climb_rate_m() for full details.
    void set_pos_target_U_from_climb_rate_cm(float vel_u_cms);

    // Generates a vertical trajectory using the given climb rate in m/s and jerk-limited shaping.
    // Target altitude is updated over time by integrating the climb rate.
    void set_pos_target_U_from_climb_rate_m(float vel_u_ms);

    // Descends at a given rate (cm/s) using jerk-limited shaping for landing.
    // If `ignore_descent_limit` is true, descent output is not limited by the configured max.
    // See land_at_climb_rate_m() for full details.
    void land_at_climb_rate_cm(float vel_u_cms, bool ignore_descent_limit);

    // Descends at a given rate (m/s) using jerk-limited shaping for landing.
    // Used during final descent phase to ensure smooth touchdown.
    void land_at_climb_rate_m(float vel_u_ms, bool ignore_descent_limit);

    // Sets vertical position, velocity, and acceleration in cm using jerk-limited shaping.
    // See input_pos_vel_accel_U_m() for full details.
    void input_pos_vel_accel_U_cm(float &pos_u_cm, float &vel_u_cms, float accel_u_cmss, bool limit_output = true);

    // Sets vertical position, velocity, and acceleration in meters using jerk-limited shaping.
    // Calculates required acceleration using current state and constraints.
    // If `limit_output` is true, limits are applied to combined (desired + correction) command.
    void input_pos_vel_accel_U_m(float &pos_u_m, float &vel_u_ms, float accel_u_mss, bool limit_output = true);

    // Sets target altitude in cm using jerk-limited shaping to gradually move to the new position.
    // See set_alt_target_with_slew_m() for full details.
    void set_alt_target_with_slew_cm(float pos_u_cm);

    // Sets target altitude in meters using jerk-limited shaping.
    void set_alt_target_with_slew_m(float pos_u_m);

    // Returns true if the U-axis controller has run in the last 5 control loop cycles.
    bool is_active_U() const;

    // Runs the vertical (U-axis) position controller.
    // Computes output acceleration based on position and velocity errors using PID correction.
    // Feedforward velocity and acceleration are combined with corrections to produce a smooth vertical command.
    // Desired position, velocity, and acceleration must be set before calling.
    void update_U_controller();



    ///
    /// Accessors
    ///

    // Sets externally computed NEU position, velocity, and acceleration in centimeters, cm/s, and cm/s².
    // See set_pos_vel_accel_NEU_m() for full details.
    void set_pos_vel_accel_NEU_cm(const Vector3p& pos_neu_cm, const Vector3f& vel_neu_cms, const Vector3f& accel_neu_cmss);

    // Sets externally computed NEU position, velocity, and acceleration in meters, m/s, and m/s².
    // Use when path planning or shaping is done outside this controller.
    void set_pos_vel_accel_NEU_m(const Vector3p& pos_neu_m, const Vector3f& vel_neu_ms, const Vector3f& accel_neu_mss);

    // Sets externally computed NE position, velocity, and acceleration in centimeters, cm/s, and cm/s².
    // See set_pos_vel_accel_NE_m() for full details.
    void set_pos_vel_accel_NE_cm(const Vector2p& pos_ne_cm, const Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss);

    // Sets externally computed NE position, velocity, and acceleration in meters, m/s, and m/s².
    // Use when path planning or shaping is done outside this controller.
    void set_pos_vel_accel_NE_m(const Vector2p& pos_ne_m, const Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss);


    /// Position

    // Returns the estimated position in NEU frame, in centimeters relative to EKF origin.
    // See get_pos_estimate_NEU_m() for full details.
    const Vector3p get_pos_estimate_NEU_cm() const { return get_pos_estimate_NEU_m() * 100.0; }

    // Returns the estimated position in NEU frame, in meters relative to EKF origin.
    const Vector3p& get_pos_estimate_NEU_m() const { return _pos_estimate_neu_m; }

    // Returns the target position in NEU frame, in centimeters relative to EKF origin.
    // See get_pos_target_NEU_m() for full details.
    const Vector3p get_pos_target_NEU_cm() const { return get_pos_target_NEU_m() * 100.0; }

    // Returns the target position in NEU frame, in meters relative to EKF origin.
    const Vector3p& get_pos_target_NEU_m() const { return _pos_target_neu_m; }

    // Sets the desired NE position in centimeters relative to EKF origin.
    // See set_pos_desired_NE_m() for full details.
    void set_pos_desired_NE_cm(const Vector2f& pos_desired_ne_cm) { set_pos_desired_NE_m(pos_desired_ne_cm * 0.01); }

    // Sets the desired NE position in meters relative to EKF origin.
    void set_pos_desired_NE_m(const Vector2f& pos_desired_ne_m) { _pos_desired_neu_m.xy() = pos_desired_ne_m.topostype(); }

    // Returns the desired position in NEU frame, in centimeters relative to EKF origin.
    // See get_pos_desired_NEU_m() for full details.
    const Vector3p get_pos_desired_NEU_cm() const { return get_pos_desired_NEU_m() * 100.0; }

    // Returns the desired position in NEU frame, in meters relative to EKF origin.
    const Vector3p& get_pos_desired_NEU_m() const { return _pos_desired_neu_m; }

    // Returns target altitude above EKF origin in centimeters.
    // See get_pos_target_U_m() for full details.
    float get_pos_target_U_cm() const { return get_pos_target_U_m() * 100.0; }

    // Returns target altitude above EKF origin in meters.
    float get_pos_target_U_m() const { return _pos_target_neu_m.z; }

    // Sets desired altitude above EKF origin in centimeters.
    // See set_pos_desired_U_m() for full details.
    void set_pos_desired_U_cm(float pos_desired_u_cm) { set_pos_desired_U_m(pos_desired_u_cm * 0.01); }

    // Sets desired altitude above EKF origin in meters.
    void set_pos_desired_U_m(float pos_desired_u_m) { _pos_desired_neu_m.z = pos_desired_u_m; }

    // Returns desired altitude above EKF origin in centimeters.
    // See get_pos_desired_U_m() for full details.
    float get_pos_desired_U_cm() const { return get_pos_desired_U_m() * 100.0; }

    // Returns desired altitude above EKF origin in meters.
    float get_pos_desired_U_m() const { return _pos_desired_neu_m.z; }


    /// Stopping Point

    // Computes NE stopping point in centimeters based on current position, velocity, and acceleration.
    // See get_stopping_point_NE_m() for full details.
    void get_stopping_point_NE_cm(Vector2p &stopping_point_neu_cm) const;

    // Computes NE stopping point in meters based on current position, velocity, and acceleration.
    void get_stopping_point_NE_m(Vector2p &stopping_point_neu_m) const;

    // Computes vertical stopping point in centimeters based on current velocity and acceleration.
    // See get_stopping_point_U_m() for full details.
    void get_stopping_point_U_cm(postype_t &stopping_point_u_cm) const;

    // Computes vertical stopping point in meters based on current velocity and acceleration.
    void get_stopping_point_U_m(postype_t &stopping_point_u_m) const;


    /// Position Error

    // Returns NEU position error vector in centimeters.
    // See get_pos_error_NEU_m() for full details.
    const Vector3f get_pos_error_NEU_cm() const { return get_pos_error_NEU_m() * 100.0; }

    // Returns NEU position error vector in meters between current and target positions.
    const Vector3f get_pos_error_NEU_m() const { return Vector3f(_p_pos_ne_m.get_error().x, _p_pos_ne_m.get_error().y, _p_pos_u_m.get_error()); }

    // Returns total NE-plane position error magnitude in centimeters.
    // See get_pos_error_NE_m() for full details.
    float get_pos_error_NE_cm() const { return get_pos_error_NE_m() * 100.0; }

    // Returns total NE-plane position error magnitude in meters.
    float get_pos_error_NE_m() const { return _p_pos_ne_m.get_error().length(); }

    // Returns vertical position error (altitude) in centimeters.
    // See get_pos_error_U_m() for full details.
    float get_pos_error_U_cm() const { return get_pos_error_U_m() * 100.0; }

    // Returns vertical position error (altitude) in meters.
    float get_pos_error_U_m() const { return _p_pos_u_m.get_error(); }


    /// Velocity

    // Returns current velocity estimate in NEU frame in cm/s.
    // See get_vel_estimate_NEU_ms() for full details.
    const Vector3f get_vel_estimate_NEU_cms() const { return get_vel_estimate_NEU_ms() * 100.0; }

    // Returns current velocity estimate in NEU frame in m/s.
    const Vector3f& get_vel_estimate_NEU_ms() const { return _vel_estimate_neu_ms; }

    // Sets desired velocity in NEU frame in cm/s.
    // See set_vel_desired_NEU_ms() for full details.
    void set_vel_desired_NEU_cms(const Vector3f &vel_desired_neu_cms) { set_vel_desired_NEU_ms(vel_desired_neu_cms * 0.01); }

    // Sets desired velocity in NEU frame in m/s.
    void set_vel_desired_NEU_ms(const Vector3f &vel_desired_neu_ms) { _vel_desired_neu_ms = vel_desired_neu_ms; }

    // Sets desired horizontal (NE) velocity in cm/s.
    // See set_vel_desired_NE_ms() for full details.
    void set_vel_desired_NE_cms(const Vector2f &vel_desired_ne_cms) { set_vel_desired_NE_ms(vel_desired_ne_cms * 0.01); }

    // Sets desired horizontal (NE) velocity in m/s.
    void set_vel_desired_NE_ms(const Vector2f &vel_desired_ne_ms) { _vel_desired_neu_ms.xy() = vel_desired_ne_ms; }

    // Returns desired velocity in NEU frame in cm/s.
    // See get_vel_desired_NEU_ms() for full details.
    const Vector3f get_vel_desired_NEU_cms() const { return get_vel_desired_NEU_ms() * 100.0; }

    // Returns desired velocity in NEU frame in m/s.
    const Vector3f& get_vel_desired_NEU_ms() const { return _vel_desired_neu_ms; }

    // Returns velocity target in NEU frame in cm/s.
    // See get_vel_target_NEU_ms() for full details.
    const Vector3f get_vel_target_NEU_cms() const { return get_vel_target_NEU_ms() * 100.0; }

    // Returns velocity target in NEU frame in m/s.
    const Vector3f& get_vel_target_NEU_ms() const { return _vel_target_neu_ms; }

    // Sets desired vertical velocity (Up) in cm/s.
    // See set_vel_desired_U_ms() for full details.
    void set_vel_desired_U_cms(float vel_desired_u_cms) { set_vel_desired_U_ms(vel_desired_u_cms * 0.01); }

    // Sets desired vertical velocity (Up) in m/s.
    void set_vel_desired_U_ms(float vel_desired_u_ms) { _vel_desired_neu_ms.z = vel_desired_u_ms; }

    // Returns vertical velocity target (Up) in cm/s.
    // See get_vel_target_U_ms() for full details.
    float get_vel_target_U_cms() const { return get_vel_target_U_ms() * 100.0; }

    // Returns vertical velocity target (Up) in m/s.
    float get_vel_target_U_ms() const { return _vel_target_neu_ms.z; }


    /// Acceleration

    // Sets desired horizontal acceleration in cm/s².
    // See set_accel_desired_NE_mss() for full details.
    void set_accel_desired_NE_cmss(const Vector2f &accel_desired_neu_cmss) { set_accel_desired_NE_mss(accel_desired_neu_cmss * 0.01); }

    // Sets desired horizontal acceleration in m/s².
    void set_accel_desired_NE_mss(const Vector2f &accel_desired_neu_mss) { _accel_desired_neu_mss.xy() = accel_desired_neu_mss; }

    // Returns target NEU acceleration in cm/s².
    // See get_accel_target_NEU_mss() for full details.
    const Vector3f get_accel_target_NEU_cmss() const { return get_accel_target_NEU_mss() * 100.0; }

    // Returns target NEU acceleration in m/s².
    const Vector3f& get_accel_target_NEU_mss() const { return _accel_target_neu_mss; }


    /// Terrain

    // Sets the terrain target altitude above EKF origin in centimeters.
    // See set_pos_terrain_target_U_m() for full details.
    void set_pos_terrain_target_U_cm(float pos_terrain_target_u_cm) { set_pos_terrain_target_U_m(pos_terrain_target_u_cm * 0.01); }

    // Sets the terrain target altitude above EKF origin in meters.
    void set_pos_terrain_target_U_m(float pos_terrain_target_u_m) { _pos_terrain_target_u_m = pos_terrain_target_u_m; }

    // Initializes terrain altitude and terrain target to the same value (in cm).
    // See init_pos_terrain_U_m() for full details.
    void init_pos_terrain_U_cm(float pos_terrain_u_cm);

    // Initializes terrain altitude and terrain target to the same value (in meters).
    void init_pos_terrain_U_m(float pos_terrain_u_m);

    // Returns current terrain altitude in centimeters above EKF origin.
    // See get_pos_terrain_U_m() for full details.
    float get_pos_terrain_U_cm() const { return get_pos_terrain_U_m() * 100.0; }

    // Returns current terrain altitude in meters above EKF origin.
    float get_pos_terrain_U_m() const { return _pos_terrain_u_m; }


    /// Offset

#if AP_SCRIPTING_ENABLED
    // Sets additional position, velocity, and acceleration offsets in meters (NED frame) for scripting.
    // Offsets are added to the controller’s internal target.
    // Used in LUA
    bool set_posvelaccel_offset(const Vector3f &pos_offset_NED_m, const Vector3f &vel_offset_NED_ms, const Vector3f &accel_offset_NED_mss);

    // Retrieves current scripted offsets in meters (NED frame).
    // Used in LUA

    bool get_posvelaccel_offset(Vector3f &pos_offset_NED_m, Vector3f &vel_offset_NED_ms, Vector3f &accel_offset_NED_mss);

    // Retrieves current target velocity (NED frame, m/s) including any scripted offset.
    // Used in LUA
    bool get_vel_target(Vector3f &vel_target_NED_ms);

    // Retrieves current target acceleration (NED frame, m/s²) including any scripted offset.
    // Used in LUA
    bool get_accel_target(Vector3f &accel_target_NED_mss);
#endif

    // Sets NE offset targets (position [cm], velocity [cm/s], acceleration [cm/s²]) from EKF origin.
    // Offsets must be refreshed at least every 3 seconds to remain active.
    // See set_posvelaccel_offset_target_NE_m() for full details.
    void set_posvelaccel_offset_target_NE_cm(const Vector2p& pos_offset_target_ne_cm, const Vector2f& vel_offset_target_ne_cms, const Vector2f& accel_offset_target_ne_cmss);

    // Sets NE offset targets in meters, m/s, and m/s².
    void set_posvelaccel_offset_target_NE_m(const Vector2p& pos_offset_target_ne_m, const Vector2f& vel_offset_target_ne_ms, const Vector2f& accel_offset_target_ne_mss);

    // Sets vertical offset targets (cm, cm/s, cm/s²) from EKF origin.
    // See set_posvelaccel_offset_target_U_m() for full details.
    void set_posvelaccel_offset_target_U_cm(float pos_offset_target_u_cm, float vel_offset_target_u_cms, float accel_offset_target_u_cmss);

    // Sets vertical offset targets (m, m/s, m/s²) from EKF origin.
    void set_posvelaccel_offset_target_U_m(float pos_offset_target_u_m, float vel_offset_target_u_ms, float accel_offset_target_u_mss);

    // Returns current NEU position offset in cm.
    // See get_pos_offset_NEU_m() for full details.
    const Vector3p get_pos_offset_NEU_cm() const { return get_pos_offset_NEU_m() * 100.0; }

    // Returns current NEU position offset in meters.
    const Vector3p& get_pos_offset_NEU_m() const { return _pos_offset_neu_m; }

    // Returns current NEU velocity offset in cm/s.
    // See get_vel_offset_NEU_ms() for full details.
    const Vector3f get_vel_offset_NEU_cms() const { return get_vel_offset_NEU_ms() * 100.0; }

    // Returns current NEU velocity offset in m/s.
    const Vector3f& get_vel_offset_NEU_ms() const { return _vel_offset_neu_ms; }

    // Returns current NEU acceleration offset in cm/s².
    // See get_accel_offset_NEU_mss() for full details.
    const Vector3f get_accel_offset_NEU_cmss() const { return get_accel_offset_NEU_mss() * 100.0; }

    // Returns current NEU acceleration offset in m/s².
    const Vector3f& get_accel_offset_NEU_mss() const { return _accel_offset_neu_mss; }

    // Sets vertical position offset in meters above EKF origin.
    void set_pos_offset_U_m(float pos_offset_u_m) { _pos_offset_neu_m.z = pos_offset_u_m; }

    // Returns vertical position offset in cm above EKF origin.
    // See get_pos_offset_U_m() for full details.
    float get_pos_offset_U_cm() const { return get_pos_offset_U_m() * 100.0; }

    // Returns vertical position offset in meters above EKF origin.
    float get_pos_offset_U_m() const { return _pos_offset_neu_m.z; }

    // Returns vertical velocity offset in cm/s.
    // See get_vel_offset_U_ms() for full details.
    float get_vel_offset_U_cms() const { return get_vel_offset_U_ms() * 100.0; }

    // Returns vertical velocity offset in m/s.
    float get_vel_offset_U_ms() const { return _vel_offset_neu_ms.z; }

    // Returns vertical acceleration offset in cm/s².
    // See get_accel_offset_U_mss() for full details.
    float get_accel_offset_U_cmss() const { return get_accel_offset_U_mss() * 100.0; }

    // Returns vertical acceleration offset in m/s².
    float get_accel_offset_U_mss() const { return _accel_offset_neu_mss.z; }

    /// Outputs

    // Returns desired roll angle in radians for the attitude controller
    float get_roll_rad() const { return _roll_target_rad; }

    // Returns desired pitch angle in radians for the attitude controller.
    float get_pitch_rad() const { return _pitch_target_rad; }

    // Returns desired yaw angle in radians for the attitude controller.
    float get_yaw_rad() const { return _yaw_target_rad; }

    // Returns desired yaw rate in radians/second for the attitude controller.
    float get_yaw_rate_rads() const { return _yaw_rate_target_rads; }

    // Returns desired roll angle in centidegrees for the attitude controller.
    // See get_roll_rad() for full details.
    float get_roll_cd() const { return rad_to_cd(_roll_target_rad); }

    // Returns desired pitch angle in centidegrees for the attitude controller.
    // See get_pitch_rad() for full details.
    float get_pitch_cd() const { return rad_to_cd(_pitch_target_rad); }

    // Returns desired yaw angle in centidegrees for the attitude controller.
    // See get_yaw_rad() for full details.
    float get_yaw_cd() const { return rad_to_cd(_yaw_target_rad); }

    // Returns desired yaw rate in centidegrees/second for the attitude controller.
    // See get_yaw_rate_rads() for full details.
    float get_yaw_rate_cds() const { return rad_to_cd(_yaw_rate_target_rads); }

    // Returns desired thrust direction as a unit vector in the body frame.
    Vector3f get_thrust_vector() const;

    // Returns bearing from current position to position target in radians.
    // 0 = North, positive = clockwise.
    float get_bearing_to_target_rad() const;

    // Returns the maximum allowed roll/pitch angle in radians.
    float get_lean_angle_max_rad() const;

    // Overrides the maximum allowed roll/pitch angle in radians.
    // A value of 0 reverts to using the ANGLE_MAX parameter.
    void set_lean_angle_max_rad(float angle_max_rad) { _angle_max_override_rad = angle_max_rad; }

    // Overrides the maximum allowed roll/pitch angle in degrees.
    // See set_lean_angle_max_rad() for full details.
    void set_lean_angle_max_deg(const float angle_max_deg) { set_lean_angle_max_rad(radians(angle_max_deg)); }

    // Overrides the maximum allowed roll/pitch angle in centidegrees.
    // See set_lean_angle_max_rad() for full details.
    void set_lean_angle_max_cd(const float angle_max_cd) { set_lean_angle_max_rad(cd_to_rad(angle_max_cd)); }

    /// Other

    // Returns reference to the NE position P controller.
    AC_P_2D& get_pos_NE_p() { return _p_pos_ne_m; }

    // Returns reference to the U (vertical) position P controller.
    AC_P_1D& get_pos_U_p() { return _p_pos_u_m; }

    // Returns reference to the NE velocity PID controller.
    AC_PID_2D& get_vel_NE_pid() { return _pid_vel_ne_cm; }

    // Returns reference to the U (vertical) velocity PID controller.
    AC_PID_Basic& get_vel_U_pid() { return _pid_vel_u_cm; }

    // Returns reference to the U acceleration PID controller.
    AC_PID& get_accel_U_pid() { return _pid_accel_u_cm_to_kt; }

    // Marks that NE acceleration has been externally limited.
    // Prevents I-term windup by storing the current target direction.
    void set_externally_limited_NE() { _limit_vector_neu.x = _accel_target_neu_mss.x; _limit_vector_neu.y = _accel_target_neu_mss.y; }

    // Converts lean angles (rad) to NEU acceleration in cm/s².
    // See lean_angles_rad_to_accel_NEU_mss() for full details.
    Vector3f lean_angles_rad_to_accel_NEU_cmss(const Vector3f& att_target_euler_rad) const;

    // Converts lean angles (rad) to NEU acceleration in m/s².
    Vector3f lean_angles_rad_to_accel_NEU_mss(const Vector3f& att_target_euler_rad) const;

    // Writes position controller diagnostic logs (PSCN, PSCE, etc).
    void write_log();

    // Performs pre-arm checks for position control parameters and EKF readiness.
    // Returns false if failure_msg is populated.
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    // Enables or disables vibration compensation mode.
    // When enabled, disables use of horizontal velocity estimates.
    void set_vibe_comp(bool on_off) { _vibe_comp_enabled = on_off; }

    // Returns confidence (0–1) in vertical control authority based on output usage.
    // Used to assess throttle margin and PID effectiveness.
    float get_vel_U_control_ratio() const { return constrain_float(_vel_u_control_ratio, 0.0f, 1.0f); }

    // Returns lateral distance to closest point on active trajectory in meters.
    // Used to assess horizontal deviation from path.
    float crosstrack_error() const;

    // Resets NEU position controller state to prevent transients when exiting standby.
    // Zeros I-terms and aligns targets to current position.
    void standby_NEU_reset();

    // Returns measured vertical (Up) acceleration in cm/s² (Earth frame, gravity-compensated).
    // See get_measured_accel_U_mss() for full details.
    float get_measured_accel_U_cmss() const { return get_measured_accel_U_mss() * 100.0; }

    // Returns measured vertical (Up) acceleration in m/s² (Earth frame, gravity-compensated).
    // Positive = upward acceleration.
    float get_measured_accel_U_mss() const { return -(_ahrs.get_accel_ef().z + GRAVITY_MSS); }

    // Returns true if the requested forward pitch is limited by the configured tilt constraint.
    bool get_fwd_pitch_is_limited() const;
    
    // Sets artificial NE position disturbance in centimeters.
    // See set_disturb_pos_NE_m() for full details.
    void set_disturb_pos_NE_cm(const Vector2f& disturb_pos_cm) { set_disturb_pos_NE_m(disturb_pos_cm * 0.01); }

    // Sets artificial NE position disturbance in meters.
    void set_disturb_pos_NE_m(const Vector2f& disturb_pos_m) { _disturb_pos_ne_m = disturb_pos_m; }

    // Sets artificial NE velocity disturbance in cm/s.
    // See set_disturb_vel_NE_ms() for full details.
    void set_disturb_vel_NE_cms(const Vector2f& disturb_vel_cms) { set_disturb_vel_NE_ms(disturb_vel_cms * 0.01); }

    // Sets artificial NE velocity disturbance in m/s.
    void set_disturb_vel_NE_ms(const Vector2f& disturb_vel_ms) { _disturb_vel_ne_ms = disturb_vel_ms; }

    static const struct AP_Param::GroupInfo var_info[];

    // Logs position controller state along the North axis to PSCN..
    // Logs desired, target, and actual position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSCN(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss);

    // Logs position controller state along the East axis to PSCE.
    // Logs desired, target, and actual values for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSCE(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss);

    // Logs position controller state along the Down (vertical) axis to PSCD.
    // Logs desired, target, and actual values for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSCD(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss);

    // Logs offset tracking along the North axis to PSON.
    // Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSON(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);

    // Logs offset tracking along the East axis to PSOE.
    // Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSOE(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);

    // Logs offset tracking along the Down axis to PSOD.
    // Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSOD(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);

    // Logs terrain-following offset tracking along the Down axis to PSOT.
    // Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
    static void Write_PSOT(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);


    // Returns pointer to the global AC_PosControl singleton.
    static AC_PosControl *get_singleton(void) { return _singleton; }

protected:

    // Calculates vertical throttle using vibration-resistant feedforward estimation.
    // Returns throttle output using manual feedforward gain for vibration compensation mode.
    // Integrator is adjusted using velocity error when PID is being overridden.
    float get_throttle_with_vibration_override();

    // Converts horizontal acceleration (cm/s²) to roll/pitch lean angles in radians.
    // See accel_NE_mss_to_lean_angles_rad() for full details.
    void accel_NE_cmss_to_lean_angles_rad(float accel_n_cmss, float accel_e_cmss, float& roll_target_rad, float& pitch_target_rad) const;

    // Converts horizontal acceleration (m/s²) to roll/pitch lean angles in radians.
    void accel_NE_mss_to_lean_angles_rad(float accel_n_mss, float accel_e_mss, float& roll_target_rad, float& pitch_target_rad) const;

    // Converts current target lean angles to NE acceleration in cm/s².
    // See lean_angles_to_accel_NE_mss() for full details.
    void lean_angles_to_accel_NE_cmss(float& accel_n_cmss, float& accel_e_cmss) const;

    // Converts current target lean angles to NE acceleration in m/s².
    void lean_angles_to_accel_NE_mss(float& accel_n_mss, float& accel_e_mss) const;

    // Computes desired yaw and yaw rate based on the NE acceleration and velocity vectors.
    // Aligns yaw with the direction of travel if speed exceeds 5% of maximum.
    void calculate_yaw_and_rate_yaw();

    // Computes scaling factor to increase max vertical accel/jerk if vertical speed exceeds configured limits.
    float calculate_overspeed_gain();


    /// Terrain Following

    // Initializes terrain position, velocity, and acceleration to match the terrain target.
    void init_terrain();

    // Updates terrain estimate (_pos_terrain_u_m) toward target using filter time constants.
    void update_terrain();


    /// Offsets

    // Initializes NE position/velocity/acceleration offsets to match their respective targets.
    void init_offsets_NE();

    // Initializes vertical (U) offsets to match their respective targets.
    void init_offsets_U();

    // Updates NE offsets by gradually moving them toward their targets.
    void update_offsets_NE();

    // Updates vertical (U) offsets by gradually moving them toward their targets.
    void update_offsets_U();

    // Initializes tracking of NE EKF position resets.
    void init_ekf_NE_reset();

    // Handles NE position reset detection and response (e.g., clearing accumulated errors).
    void handle_ekf_NE_reset();

    // Initializes tracking of vertical (U) EKF resets.
    void init_ekf_U_reset();

    // Handles U EKF reset detection and response.
    void handle_ekf_U_reset();

    // references to inertial nav and ahrs libraries
    AP_AHRS_View&           _ahrs;
    const class AP_Motors&  _motors;
    AC_AttitudeControl&     _attitude_control;

    // parameters
    AP_Float        _lean_angle_max_deg;    // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AP_Float        _shaping_jerk_ne_msss;  // Jerk limit of the ne kinematic path generation in m/s³ used to determine how quickly the aircraft varies the acceleration target
    AP_Float        _shaping_jerk_u_msss;   // Jerk limit of the u kinematic path generation in m/s³ used to determine how quickly the aircraft varies the acceleration target
    AC_P_2D         _p_pos_ne_m;            // XY axis position controller to convert target distance (cm) to target velocity (cm/s)
    AC_P_1D         _p_pos_u_m;             // Z axis position controller to convert target altitude (cm) to target climb rate (cm/s)
    AC_PID_2D       _pid_vel_ne_cm;         // XY axis velocity controller to convert target velocity (cm/s) to target acceleration (cm/s²)
    AC_PID_Basic    _pid_vel_u_cm;          // Z axis velocity controller to convert target climb rate (cm/s) to target acceleration (cm/s²)
    AC_PID          _pid_accel_u_cm_to_kt;  // Z axis acceleration controller to convert target acceleration (cm/s²) to throttle output (0 to 1000)

    // internal variables
    float       _dt_s;                     // time difference (in seconds) since the last loop time
    uint32_t    _last_update_ne_ticks;     // ticks of last last update_NE_controller call
    uint32_t    _last_update_u_ticks;      // ticks of last update_z_controller call
    float       _vel_max_ne_ms;            // max horizontal speed in m/s used for kinematic shaping
    float       _vel_max_up_ms;            // max climb rate in m/s used for kinematic shaping
    float       _vel_max_down_ms;          // max descent rate in m/s used for kinematic shaping
    float       _accel_max_ne_mss;         // max horizontal acceleration in m/s² used for kinematic shaping
    float       _accel_max_u_mss;          // max vertical acceleration in m/s² used for kinematic shaping
    float       _jerk_max_ne_msss;         // Jerk limit of the ne kinematic path generation in m/s³ used to determine how quickly the aircraft varies the acceleration target
    float       _jerk_max_u_msss;          // Jerk limit of the z kinematic path generation in m/s³ used to determine how quickly the aircraft varies the acceleration target
    float       _vel_u_control_ratio = 2.0f;    // confidence that we have control in the vertical axis
    Vector2f    _disturb_pos_ne_m;         // position disturbance generated by system ID mode
    Vector2f    _disturb_vel_ne_ms;        // velocity disturbance generated by system ID mode
    float       _ne_control_scale_factor = 1.0; // single loop scale factor for XY control

    // output from controller
    float       _roll_target_rad;            // desired roll angle in radians calculated by position controller
    float       _pitch_target_rad;           // desired roll pitch in radians calculated by position controller
    float       _yaw_target_rad;             // desired yaw in radians calculated by position controller
    float       _yaw_rate_target_rads;       // desired yaw rate in radians per second calculated by position controller

    // position controller internal variables
    Vector3p    _pos_estimate_neu_m;
    Vector3p    _pos_desired_neu_m;        // desired location, frame NEU in m relative to the EKF origin. This is equal to the _pos_target minus offsets
    Vector3p    _pos_target_neu_m;         // target location, frame NEU in m relative to the EKF origin. This is equal to the _pos_desired_neu_m plus offsets
    Vector3f    _vel_estimate_neu_ms;
    Vector3f    _vel_desired_neu_ms;       // desired velocity in NEU m/s
    Vector3f    _vel_target_neu_ms;        // velocity target in NEU m/s calculated by pos_to_rate step
    Vector3f    _accel_desired_neu_mss;    // desired acceleration in NEU m/s² (feed forward)
    Vector3f    _accel_target_neu_mss;     // acceleration target in NEU m/s²
    // todo: seperate the limit vector into ne and u. ne is based on acceleration while u is set +-1 based on throttle saturation. Together they don't form a direction vector because the units are different.
    Vector3f    _limit_vector_neu;         // the direction that the position controller is limited, zero when not limited

    // terrain handling variables
    float    _pos_terrain_target_u_m;    // position terrain target in m relative to the EKF origin in NEU frame
    float    _pos_terrain_u_m;           // position terrain in m from the EKF origin in NEU frame. This terrain moves towards _pos_terrain_target_u_m
    float    _vel_terrain_u_ms;          // velocity terrain in NEU m/s calculated by pos_to_rate step. This terrain moves towards _vel_terrain_target
    float    _accel_terrain_u_mss;       // acceleration terrain in NEU m/s²

    // offset handling variables
    Vector3p    _pos_offset_target_neu_m;      // position offset target in m relative to the EKF origin in NEU frame
    Vector3p    _pos_offset_neu_m;             // position offset in m from the EKF origin in NEU frame. This offset moves towards _pos_offset_target_neu_m
    Vector3f    _vel_offset_target_neu_ms;     // velocity offset target in m/s in NEU frame
    Vector3f    _vel_offset_neu_ms;            // velocity offset in NEU m/s calculated by pos_to_rate step. This offset moves towards _vel_offset_target_neu_ms
    Vector3f    _accel_offset_target_neu_mss;  // acceleration offset target in m/s² in NEU frame
    Vector3f    _accel_offset_neu_mss;         // acceleration offset in NEU m/s²
    uint32_t    _posvelaccel_offset_target_ne_ms;   // system time that pos, vel, accel targets were set (used to implement timeouts)
    uint32_t    _posvelaccel_offset_target_u_ms;    // system time that pos, vel, accel targets were set (used to implement timeouts)

    // ekf reset handling
    uint32_t    _ekf_ne_reset_ms;       // system time of last recorded ekf ne position reset
    uint32_t    _ekf_u_reset_ms;        // system time of last recorded ekf altitude reset

    // high vibration handling
    bool        _vibe_comp_enabled;     // true when high vibration compensation is on

    // angle max override, if zero then use ANGLE_MAX parameter
    float       _angle_max_override_rad;

    // return true if on a real vehicle or SITL with lock-step scheduling
    bool has_good_timing(void) const;

private:
    // Internal log writer for PSCx (North, East, Down tracking).
    // Reduces duplication between Write_PSCN, PSCE, and PSCD.
    // Used for logging desired/target/actual position, velocity, and acceleration per axis.
    static void Write_PSCx(LogMessages ID, float pos_desired_m, float pos_target_m, float pos_m, 
                            float vel_desired_ms, float vel_target_ms, float vel_ms, 
                            float accel_desired_mss, float accel_target_mss, float accel_mss);

    // Internal log writer for PSOx (North, East, Down tracking).
    // Reduces duplication between Write_PSON, PSOE, and PSOD.
    // Used for logging the target/actual position, velocity, and acceleration offset per axis.
    static void Write_PSOx(LogMessages id, float pos_target_offset_m, float pos_offset_m,
                            float vel_target_offset_ms, float vel_offset_ms,
                            float accel_target_offset_mss, float accel_offset_mss);

    // singleton
    static AC_PosControl *_singleton;
};
