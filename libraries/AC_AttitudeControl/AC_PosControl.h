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
#define POSCONTROL_SPEED_DOWN_MS                1.5f    // default descent rate in m/s
#define POSCONTROL_SPEED_UP_MS                  2.5f    // default climb rate in m/s

#define POSCONTROL_ACCEL_D_MSS                  2.5f    // default vertical acceleration in m/s²
#define POSCONTROL_JERK_D_MSSS                  5.0f    // default vertical jerk m/s³

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

    // Updates internal NED position and velocity estimates from AHRS.
    // Falls back to vertical-only data if horizontal velocity or position is invalid or vibration forces it.
    // When high_vibes is true, forces use of vertical fallback for velocity.
    void update_estimates(bool high_vibes = false);

    // Returns the jerk limit for horizontal path shaping in m/s³.
    // Used to constrain acceleration changes in trajectory generation.
    float get_shaping_jerk_NE_msss() const { return _shaping_jerk_ne_msss; }


    ///
    /// 3D position shaper
    ///

    // Sets a new NED position target in meters and computes a jerk-limited trajectory.
    // Updates internal acceleration commands using a smooth kinematic path constrained
    // by configured acceleration and jerk limits. 
    // The path can be offset vertically to follow the terrain by providing the current 
    // terrain level in the NED frame and the terrain margin. Terrain margin is used to
    // constrain horizontal velocity to avoid vertical buffer violation.
    void input_pos_NED_m(const Vector3p& pos_ned_m, float pos_terrain_target_d_m, float terrain_margin_m);

    // Returns a scaling factor for horizontal velocity in m/s to ensure
    // the vertical controller maintains a safe distance above terrain.
    float terrain_scaler_D_m(float pos_terrain_d_m, float terrain_margin_m) const;

    ///
    /// Lateral position controller
    ///

    // Sets maximum horizontal speed (cm/s) and acceleration (cm/s²) for NE-axis shaping.
    // Can be called anytime; transitions are handled smoothly.
    // All arguments should be positive.
    // See NE_set_max_speed_accel_m() for full details.
    void NE_set_max_speed_accel_cm(float speed_cms, float accel_cmss);

    // Sets maximum horizontal speed (m/s) and acceleration (m/s²) for NE-axis shaping.
    // These values constrain the kinematic trajectory used by the lateral controller.
    // All arguments should be positive.
    void NE_set_max_speed_accel_m(float speed_ms, float accel_mss);

    // Sets horizontal correction limits for velocity (cm/s) and acceleration (cm/s²).
    // Should be called only during initialization to avoid control discontinuities.
    // All arguments should be positive.
    // See NE_set_correction_speed_accel_m() for full details.
    void NE_set_correction_speed_accel_cm(float speed_cms, float accel_cmss);

    // Sets horizontal correction limits for velocity (m/s) and acceleration (m/s²).
    // These values constrain the PID correction path, not the desired trajectory.
    // All arguments should be positive.
    void NE_set_correction_speed_accel_m(float speed_ms, float accel_mss);

    // Returns maximum horizontal speed in cm/s.
    // See NE_get_max_speed_ms() for full details.
    float NE_get_max_speed_cms() const { return NE_get_max_speed_ms() * 100.0; }

    // Returns maximum horizontal speed in m/s used for shaping the trajectory.
    float NE_get_max_speed_ms() const { return _vel_max_ne_ms; }

    // Returns maximum horizontal acceleration in m/s² used for trajectory shaping.
    float NE_get_max_accel_mss() const { return _accel_max_ne_mss; }

    // Sets maximum allowed horizontal position error in meters.
    // Used to constrain the output of the horizontal position P controller.
    void NE_set_pos_error_max_m(float error_max_m) { _p_pos_ne_m.set_error_max(error_max_m); }

    // Returns maximum allowed horizontal position error in meters.
    float NE_get_pos_error_max_m() const { return _p_pos_ne_m.get_error_max(); }

    // Initializes NE controller to a stationary stopping point with zero velocity and acceleration.
    // Use when the expected trajectory begins at rest but the starting position is unspecified.
    // The starting position can be retrieved with get_pos_target_NED_m().
    void NE_init_controller_stopping_point();

    // Smoothly decays NE acceleration over time to zero while maintaining current velocity and position.
    // Reduces output acceleration by ~95% over 0.5 seconds to avoid abrupt transitions.
    void NE_relax_velocity_controller();

    // Softens NE controller for landing by reducing position error and suppressing I-term windup.
    // Used to make descent behavior more stable near ground contact.
    void NE_soften_for_landing();

    // Fully initializes the NE controller with current position, velocity, acceleration, and attitude.
    // Intended for normal startup when the full state is known.
    // Private function shared by other NE initializers.
    void NE_init_controller();

    // Sets the desired NE-plane acceleration in m/s² using jerk-limited shaping.
    // Smoothly transitions to the specified acceleration from current kinematic state.
    // Constraints: max acceleration and jerk set via NE_set_max_speed_accel_m().
    void input_accel_NE_m(const Vector2f& accel_ne_msss);

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
    bool NE_is_active() const;

    // Disables NE position correction by setting the target position to the current position.
    // Useful to freeze positional control without disrupting velocity control.
    void NE_stop_pos_stabilisation();

    // Disables NE position and velocity correction by setting target values to current state.
    // Useful to prevent further corrections and freeze motion stabilization in NE axes.
    void NE_stop_vel_stabilisation();

    // Applies a scalar multiplier to the NE control loop.
    // Set to 0 to disable lateral control; 1 for full authority.
    void NE_set_control_scale_factor(float ne_control_scale_factor) {
        _ne_control_scale_factor = ne_control_scale_factor;
    }

    // Runs the NE-axis position controller, computing output acceleration from position and velocity errors.
    // Uses P and PID controllers to generate corrections which are added to feedforward velocity/acceleration.
    // Requires all desired targets to be pre-set using the input_* or set_* methods.
    void NE_update_controller();

    ///
    /// Vertical position controller
    ///

    // Sets maximum climb/descent rate (cm/s) and vertical acceleration (cm/s²) for the U-axis.
    // Descent rate may be positive or negative and is always interpreted as a descent.
    // See D_set_max_speed_accel_m() for full details.
    // All values must be positive.
    void D_set_max_speed_accel_cm(float decent_speed_max_cms, float climb_speed_max_cms, float accel_max_u_cmss);

    // Sets maximum climb/descent rate (m/s) and vertical acceleration (m/s²) for the U-axis.
    // These values are used for jerk-limited kinematic shaping of the vertical trajectory.
    // All values must be positive.
    void D_set_max_speed_accel_m(float decent_speed_max_ms, float climb_speed_max_ms, float accel_max_d_mss);

    // Sets vertical correction velocity and acceleration limits (cm/s, cm/s²).
    // Should only be called during initialization to avoid discontinuities.
    // See D_set_correction_speed_accel_m() for full details.
    // All values must be positive.
    void D_set_correction_speed_accel_cm(float decent_speed_max_cms, float climb_speed_max_cms, float accel_max_u_cmss);

    // Sets vertical correction velocity and acceleration limits (m/s, m/s²).
    // These values constrain the correction output of the PID controller.
    // All values must be positive.
    void D_set_correction_speed_accel_m(float decent_speed_max_ms, float climb_speed_max_ms, float accel_max_d_mss);

    // Returns maximum vertical acceleration in m/s² used for shaping the climb/descent trajectory.
    float D_get_max_accel_mss() const { return _accel_max_d_mss; }

    // Returns maximum allowed positive (upward) position error in meters.
    float get_pos_error_up_m() const { return _p_pos_d_m.get_error_min(); }

    // Returns maximum allowed negative (downward) position error in meters.
    float get_pos_error_down_m() const { return _p_pos_d_m.get_error_max(); }

    // Returns maximum climb rate in cm/s.
    // See get_max_speed_up_ms() for full details.
    float get_max_speed_up_cms() const { return get_max_speed_up_ms() * 100.0; }

    // Returns maximum climb rate in m/s used for shaping the vertical trajectory.
    float get_max_speed_up_ms() const { return _vel_max_up_ms; }

    /// Returns maximum descent rate in m/s (zero or positive).
    float get_max_speed_down_ms() const { return _vel_max_down_ms; }

    // Initializes U-axis controller to current position, velocity, and acceleration, disallowing descent.
    // Used for takeoff or hold scenarios where downward motion is prohibited.
    void D_init_controller_no_descent();

    // Initializes U-axis controller to a stationary stopping point with zero velocity and acceleration.
    // Used when the trajectory starts at rest but the initial altitude is unspecified.
    // The resulting position target can be retrieved with get_pos_target_NED_m().
    void D_init_controller_stopping_point();

    // Smoothly decays U-axis acceleration to zero over time while maintaining current vertical velocity.
    // Reduces requested acceleration by ~95% every 0.5 seconds to avoid abrupt transitions.
    // `throttle_setting` is used to determine whether to preserve positive acceleration in low-thrust cases.
    void D_relax_controller(float throttle_setting);

    // Fully initializes the U-axis controller with current position, velocity, acceleration, and attitude.
    // Used during standard controller activation when full state is known.
    // Private function shared by other vertical initializers.
    void D_init_controller();

    // Sets the desired vertical acceleration in m/s² using jerk-limited shaping.
    // Smoothly transitions to the target acceleration from current kinematic state.
    // Constraints: max acceleration and jerk set via D_set_max_speed_accel_m().
    void input_accel_D_m(float accel_d_mss);

    // Sets desired vertical velocity and acceleration (m/s, m/s²) using jerk-limited shaping.
    // Calculates required acceleration using current vertical kinematics.
    // If `limit_output` is true, limits apply to the combined (desired + correction) command.
    void input_vel_accel_D_m(float &vel_d_ms, float accel_d_mss, bool limit_output = true);

    // Generates a vertical trajectory using the given climb rate in cm/s and jerk-limited shaping.
    // Adjusts the internal target altitude based on integrated climb rate.
    // See D_set_pos_target_from_climb_rate_ms() for full details.
    void D_set_pos_target_from_climb_rate_cms(float climb_rate_cms);

    // Generates a vertical trajectory using the given climb rate in m/s and jerk-limited shaping.
    // Target altitude is updated over time by integrating the climb rate.
    void D_set_pos_target_from_climb_rate_ms(float climb_rate_ms, bool ignore_descent_limit = false);

    // Sets vertical position, velocity, and acceleration in cm using jerk-limited shaping.
    // See input_pos_vel_accel_D_m() for full details.
    void input_pos_vel_accel_U_cm(float &pos_u_cm, float &vel_u_cms, float accel_u_cmss, bool limit_output = true);

    // Sets vertical position, velocity, and acceleration in meters using jerk-limited shaping.
    // Calculates required acceleration using current state and constraints.
    // If `limit_output` is true, limits are applied to combined (desired + correction) command.
    void input_pos_vel_accel_D_m(float &pos_d_m, float &vel_d_ms, float accel_d_mss, bool limit_output = true);

    // Sets target altitude in cm using jerk-limited shaping to gradually move to the new position.
    // See D_set_alt_target_with_slew_m() for full details.
    void set_alt_target_with_slew_cm(float pos_u_cm);

    // Sets target altitude in meters using jerk-limited shaping.
    void D_set_alt_target_with_slew_m(float pos_u_m);

    // Returns true if the U-axis controller has run in the last 5 control loop cycles.
    bool D_is_active() const;

    // Runs the vertical (U-axis) position controller.
    // Computes output acceleration based on position and velocity errors using PID correction.
    // Feedforward velocity and acceleration are combined with corrections to produce a smooth vertical command.
    // Desired position, velocity, and acceleration must be set before calling.
    void D_update_controller();



    ///
    /// Accessors
    ///

    // Sets externally computed NED position, velocity, and acceleration in meters, m/s, and m/s².
    // Use when path planning or shaping is done outside this controller.
    void set_pos_vel_accel_NED_m(const Vector3p& pos_ned_m, const Vector3f& vel_ned_ms, const Vector3f& accel_ned_mss);

    // Sets externally computed NE position, velocity, and acceleration in meters, m/s, and m/s².
    // Use when path planning or shaping is done outside this controller.
    void set_pos_vel_accel_NE_m(const Vector2p& pos_ne_m, const Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss);


    /// Position

    // Returns the estimated position in NED frame, in meters relative to EKF origin.
    const Vector3p& get_pos_estimate_NED_m() const { return _pos_estimate_ned_m; }

    // Returns estimated altitude above EKF origin in meters.
    float get_pos_estimate_U_m() const { return -_pos_estimate_ned_m.z; }

    // Returns the target position in NED frame, in meters relative to EKF origin.
    const Vector3p& get_pos_target_NED_m() const { return _pos_target_ned_m; }

    // Sets the desired NE position in meters relative to EKF origin.
    void set_pos_desired_NE_m(const Vector2p& pos_desired_ne_m) { _pos_desired_ned_m.xy() = pos_desired_ne_m; }

    // Returns the desired position in NED frame, in meters relative to EKF origin.
    const Vector3p& get_pos_desired_NED_m() const { return _pos_desired_ned_m; }

    // Returns target altitude above EKF origin in centimeters.
    // See get_pos_target_U_m() for full details.
    float get_pos_target_U_cm() const { return get_pos_target_U_m() * 100.0; }

    // Returns target altitude above EKF origin in meters.
    float get_pos_target_U_m() const { return -_pos_target_ned_m.z; }

    // Sets desired altitude above EKF origin in centimeters.
    // See set_pos_desired_U_m() for full details.
    void set_pos_desired_U_cm(float pos_desired_u_cm) { set_pos_desired_U_m(pos_desired_u_cm * 0.01); }

    // Sets desired altitude above EKF origin in meters.
    void set_pos_desired_U_m(float pos_desired_u_m) { _pos_desired_ned_m.z = -pos_desired_u_m; }

    // Returns desired altitude above EKF origin in centimeters.
    // See get_pos_desired_U_m() for full details.
    float get_pos_desired_U_cm() const { return get_pos_desired_U_m() * 100.0; }

    // Returns desired altitude above EKF origin in meters.
    float get_pos_desired_U_m() const { return -_pos_desired_ned_m.z; }


    /// Stopping Point

    // Computes NE stopping point in meters based on current position, velocity, and acceleration.
    void get_stopping_point_NE_m(Vector2p &stopping_point_ne_m) const;

    // Computes vertical stopping point relative to EKF origin in meters, Down-positive. based on current velocity and acceleration.
    void get_stopping_point_D_m(postype_t &stopping_point_d_m) const;


    /// Position Error

    // Returns NED position error vector in meters between current and target positions.
    const Vector3f get_pos_error_NED_m() const { return Vector3f{_p_pos_ne_m.get_error().x, _p_pos_ne_m.get_error().y, _p_pos_d_m.get_error()}; }

    // Returns total NE-plane position error magnitude in meters.
    float get_pos_error_NE_m() const { return _p_pos_ne_m.get_error().length(); }

    // Returns vertical position error (altitude) in centimeters.
    // See get_pos_error_D_m() for full details.
    float get_pos_error_U_cm() const { return -get_pos_error_D_m() * 100.0; }

    // Returns vertical position error (altitude) in meters.
    float get_pos_error_D_m() const { return _p_pos_d_m.get_error(); }


    /// Velocity

    // Returns current velocity estimate in NED frame in m/s.
    const Vector3f& get_vel_estimate_NED_ms() const { return _vel_estimate_ned_ms; }

    // Returns current velocity estimate (Up) in m/s.
    float get_vel_estimate_U_ms() const { return -_vel_estimate_ned_ms.z; }

    // Sets desired velocity in NED frame in cm/s.
    // See set_vel_desired_NED_ms() for full details.
    void set_vel_desired_NEU_cms(const Vector3f &vel_desired_neu_cms) { set_vel_desired_NED_ms(Vector3f{vel_desired_neu_cms.x, vel_desired_neu_cms.y, -vel_desired_neu_cms.z} * 0.01); }

    // Sets desired velocity in NED frame in m/s.
    void set_vel_desired_NED_ms(const Vector3f &vel_desired_ned_ms) { _vel_desired_ned_ms = vel_desired_ned_ms; }

    // Sets desired horizontal (NE) velocity in m/s.
    void set_vel_desired_NE_ms(const Vector2f &vel_desired_ne_ms) { _vel_desired_ned_ms.xy() = vel_desired_ne_ms; }

    // Returns desired velocity in NEU frame in cm/s.
    // See get_vel_desired_NED_ms() for full details.
    const Vector3f get_vel_desired_NEU_cms() const { return Vector3f{_vel_desired_ned_ms.x, _vel_desired_ned_ms.y, -_vel_desired_ned_ms.z} * 100.0; }

    // Returns desired velocity in NED frame in m/s.
    const Vector3f& get_vel_desired_NED_ms() const { return _vel_desired_ned_ms; }

    // Returns desired velocity (Up) in m/s.
    float get_vel_desired_U_ms() const { return -_vel_desired_ned_ms.z; }

    // Returns velocity target in NEU frame in cm/s.
    // See get_vel_target_NED_ms() for full details.
    const Vector3f get_vel_target_NEU_cms() const { return Vector3f{_vel_target_ned_ms.x, _vel_target_ned_ms.y, -_vel_target_ned_ms.z} * 100.0; }

    // Returns velocity target in NED frame in m/s.
    const Vector3f& get_vel_target_NED_ms() const { return _vel_target_ned_ms; }

    // Sets desired vertical velocity (Up) in m/s.
    void set_vel_desired_D_ms(float vel_desired_d_ms) { _vel_desired_ned_ms.z = vel_desired_d_ms; }

    // Returns vertical velocity target (Up) in cm/s.
    // See get_vel_target_U_ms() for full details.
    float get_vel_target_U_cms() const { return get_vel_target_U_ms() * 100.0; }

    // Returns vertical velocity target (Up) in m/s.
    float get_vel_target_U_ms() const { return -_vel_target_ned_ms.z; }


    /// Acceleration

    // Sets desired horizontal acceleration in m/s².
    void set_accel_desired_NE_mss(const Vector2f &accel_desired_ne_mss) { _accel_desired_ned_mss.xy() = accel_desired_ne_mss; }

    // Returns target NED acceleration in m/s².
    const Vector3f& get_accel_target_NED_mss() const { return _accel_target_ned_mss; }


    /// Terrain

    // Sets both the terrain altitude and terrain target to the same value
    // (altitude above EKF origin in centimeters, Up-positive).
    // See set_pos_terrain_target_D_m() for full description.
    void set_pos_terrain_target_U_cm(float pos_terrain_target_u_cm) { set_pos_terrain_target_D_m(-pos_terrain_target_u_cm * 0.01); }

    // Sets both the terrain altitude and terrain target to the same value
    // (relative to EKF origin in meters, Down-positive).
    void set_pos_terrain_target_D_m(float pos_terrain_target_d_m) { _pos_terrain_target_d_m = pos_terrain_target_d_m; }

    // Initializes both the terrain altitude and terrain target to the same value
    // (altitude above EKF origin in centimeters, Up-positive).
    // See init_pos_terrain_D_m() for full description.
    void init_pos_terrain_U_cm(float pos_terrain_u_cm);

    // Initializes both the terrain altitude and terrain target to the same value
    // (relative to EKF origin in meters, Down-positive).
    void init_pos_terrain_D_m(float pos_terrain_d_m);

    // Returns the current terrain altitude (Down-positive, relative to EKF origin, in meters).
    float get_pos_terrain_D_m() const { return _pos_terrain_d_m; }


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

    // Sets NE offset targets in meters, m/s, and m/s².
    void set_posvelaccel_offset_target_NE_m(const Vector2p& pos_offset_target_ne_m, const Vector2f& vel_offset_target_ne_ms, const Vector2f& accel_offset_target_ne_mss);

    // Sets vertical offset targets (m, m/s, m/s²) relative to EKF origin in meters, Down-positive.
    void set_posvelaccel_offset_target_D_m(float pos_offset_target_d_m, float vel_offset_target_d_ms, float accel_offset_target_d_mss);

    // Returns current NED position offset in meters.
    const Vector3p& get_pos_offset_NED_m() const { return _pos_offset_ned_m; }

    // Returns current NED velocity offset in m/s.
    const Vector3f& get_vel_offset_NED_ms() const { return _vel_offset_ned_ms; }

    // Returns current NED acceleration offset in m/s².
    const Vector3f& get_accel_offset_NED_mss() const { return _accel_offset_ned_mss; }

    // Sets vertical position offset in meters relative to EKF origin in meters, Down-positive.
    void set_pos_offset_D_m(float pos_offset_d_m) { _pos_offset_ned_m.z = pos_offset_d_m; }

    // Returns vertical position offset in meters relative to EKF origin in meters, Down-positive.
    float get_pos_offset_U_m() const { return -_pos_offset_ned_m.z; }

    // Returns vertical velocity offset in m/s.
    float get_vel_offset_D_ms() const { return _vel_offset_ned_ms.z; }

    // Returns vertical acceleration offset in m/s².
    float get_accel_offset_D_mss() const { return _accel_offset_ned_mss.z; }

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
    AC_P_2D& NE_get_pos_p() { return _p_pos_ne_m; }

    // Returns reference to the D (vertical) position P controller.
    AC_P_1D& D_get_pos_p() { return _p_pos_d_m; }

    // Returns reference to the NE velocity PID controller.
    AC_PID_2D& NE_get_vel_pid() { return _pid_vel_ne_m; }

    // Returns reference to the D (vertical) velocity PID controller.
    AC_PID_Basic& D_get_vel_pid() { return _pid_vel_d_m; }

    // Returns reference to the D acceleration PID controller.
    AC_PID& D_get_accel_pid() { return _pid_accel_d_m; }

    // Marks that NE acceleration has been externally limited.
    // Prevents I-term windup by storing the current target direction.
    void NE_set_externally_limited() { _limit_vector_ned.x = _accel_target_ned_mss.x; _limit_vector_ned.y = _accel_target_ned_mss.y; }

    // Converts lean angles (rad) to NED acceleration in m/s².
    Vector3f lean_angles_rad_to_accel_NED_mss(const Vector3f& att_target_euler_rad) const;

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

    // Reset handling method
    enum class EKFResetMethod : uint8_t {
        MoveTarget = 0,     // the target position is reset so the vehicle does not physically move
        MoveVehicle = 1     // the target position is smoothly transitioned so the vehicle moves to its previous position coordinates
    };
    void set_reset_handling_method(EKFResetMethod reset_method) { _ekf_reset_method = reset_method; }

    // Returns confidence (0–1) in vertical control authority based on output usage.
    // Used to assess throttle margin and PID effectiveness.
    float get_vel_D_control_ratio() const { return constrain_float(_vel_d_control_ratio, 0.0f, 1.0f); }

    // Returns lateral distance to closest point on active trajectory in meters.
    // Used to assess horizontal deviation from path.
    float crosstrack_error_m() const;

    // Resets NED position controller state to prevent transients when exiting standby.
    // Zeros I-terms and aligns targets to current position.
    void NED_standby_reset();

    // Returns measured vertical (Down) acceleration in m/s² (Earth frame, gravity-compensated).
    // Positive = downward acceleration.
    float get_estimated_accel_D_mss() const { return _ahrs.get_accel_ef().z + GRAVITY_MSS; }

    // Returns measured vertical (Up) acceleration in m/s² (Earth frame, gravity-compensated).
    // Positive = upward acceleration.
    float get_estimated_accel_U_mss() const { return -get_estimated_accel_D_mss(); }

    // Returns true if the requested forward pitch is limited by the configured tilt constraint.
    bool get_fwd_pitch_is_limited() const;

    // Sets artificial NE position disturbance in meters.
    void set_disturb_pos_NE_m(const Vector2f& disturb_pos_m) { _disturb_pos_ne_m = disturb_pos_m; }

    // Sets artificial NE velocity disturbance in m/s.
    void set_disturb_vel_NE_ms(const Vector2f& disturb_vel_ms) { _disturb_vel_ne_ms = disturb_vel_ms; }

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

    // perform any required parameter conversions
    void convert_parameters();

    // Returns pointer to the global AC_PosControl singleton.
    static AC_PosControl *get_singleton(void) { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Calculates vertical throttle using vibration-resistant feedforward estimation.
    // Returns throttle output using manual feedforward gain for vibration compensation mode.
    // Integrator is adjusted using velocity error when PID is being overridden.
    float get_throttle_with_vibration_override();

    // Converts horizontal acceleration (m/s²) to roll/pitch lean angles in radians.
    void accel_NE_mss_to_lean_angles_rad(float accel_n_mss, float accel_e_mss, float& roll_target_rad, float& pitch_target_rad) const;

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

    // Updates terrain estimate (_pos_terrain_d_m) toward target using filter time constants.
    void update_terrain();


    /// Offsets

    // Initializes NE position/velocity/acceleration offsets to match their respective targets.
    void NE_init_offsets();

    // Initializes vertical (D) offsets to match their respective targets.
    void D_init_offsets();

    // Updates NE offsets by gradually moving them toward their targets.
    void NE_update_offsets();

    // Updates vertical (D) offsets by gradually moving them toward their targets.
    void D_update_offsets();

    // Initializes tracking of NE EKF position resets.
    void NE_init_ekf_reset();

    // Handles NE position reset detection and response (e.g., clearing accumulated errors).
    void NE_handle_ekf_reset();

    // Initializes tracking of vertical (D) EKF resets.
    void D_init_ekf_reset();

    // Handles the vertical (D) EKF reset detection and response.
    void D_handle_ekf_reset();

    // references to inertial nav and ahrs libraries
    AP_AHRS_View&           _ahrs;
    const class AP_Motors&  _motors;
    AC_AttitudeControl&     _attitude_control;

    // parameters
    AP_Float        _lean_angle_max_deg;    // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AP_Float        _shaping_jerk_ne_msss;  // Jerk limit of the ne kinematic path generation in m/s³ used to determine how quickly the aircraft varies the acceleration target
    AP_Float        _shaping_jerk_d_msss;   // Jerk limit of the u kinematic path generation in m/s³ used to determine how quickly the aircraft varies the acceleration target
    AC_P_2D         _p_pos_ne_m;            // XY axis position controller to convert target distance (m) to target velocity (m/s)
    AC_P_1D         _p_pos_d_m;             // Z axis position controller to convert target altitude (m) to target climb rate (m/s)
    AC_PID_2D       _pid_vel_ne_m;          // XY axis velocity controller to convert target velocity (m/s) to target acceleration (m/s²)
    AC_PID_Basic    _pid_vel_d_m;           // Z axis velocity controller to convert target climb rate (m/s) to target acceleration (m/s²)
    AC_PID          _pid_accel_d_m;         // Z axis acceleration controller to convert target acceleration (in units of gravity) to normalised throttle output

    // internal variables
    float       _dt_s;                      // time difference (in seconds) since the last loop time
    uint32_t    _last_update_ne_ticks;      // ticks of last NE_update_controller call
    uint32_t    _last_update_d_ticks;       // ticks of last update_z_controller call
    float       _vel_max_ne_ms;             // max horizontal speed in m/s used for kinematic shaping
    float       _vel_max_up_ms;             // max climb rate in m/s used for kinematic shaping
    float       _vel_max_down_ms;           // max descent rate in m/s used for kinematic shaping
    float       _accel_max_ne_mss;          // max horizontal acceleration in m/s² used for kinematic shaping
    float       _accel_max_d_mss;           // max vertical acceleration in m/s² used for kinematic shaping
    float       _jerk_max_ne_msss;          // Jerk limit of the ne kinematic path generation in m/s³ used to determine how quickly the aircraft varies the acceleration target
    float       _jerk_max_d_msss;           // Jerk limit of the z kinematic path generation in m/s³ used to determine how quickly the aircraft varies the acceleration target
    float       _vel_d_control_ratio = 2.0f;// confidence that we have control in the vertical axis
    Vector2f    _disturb_pos_ne_m;          // position disturbance generated by system ID mode
    Vector2f    _disturb_vel_ne_ms;         // velocity disturbance generated by system ID mode
    float       _ne_control_scale_factor = 1.0; // single loop scale factor for XY control

    // output from controller
    float       _roll_target_rad;           // desired roll angle in radians calculated by position controller
    float       _pitch_target_rad;          // desired roll pitch in radians calculated by position controller
    float       _yaw_target_rad;            // desired yaw in radians calculated by position controller
    float       _yaw_rate_target_rads;      // desired yaw rate in radians per second calculated by position controller

    // position controller internal variables
    Vector3p    _pos_estimate_ned_m;        // estimated location, frame NED in m relative to the EKF origin.
    Vector3p    _pos_desired_ned_m;         // desired location, frame NED in m relative to the EKF origin. This is equal to the _pos_target_ned_m minus offsets
    Vector3p    _pos_target_ned_m;          // target location, frame NED in m relative to the EKF origin. This is equal to the _pos_desired_ned_m plus offsets
    Vector3f    _vel_estimate_ned_ms;       // estimated velocity in NED m/s
    Vector3f    _vel_desired_ned_ms;        // desired velocity in NED m/s
    Vector3f    _vel_target_ned_ms;         // velocity target in NED m/s calculated by pos_to_rate step
    Vector3f    _accel_desired_ned_mss;     // desired acceleration in NED m/s² (feed forward)
    Vector3f    _accel_target_ned_mss;      // acceleration target in NED m/s²
    // todo: seperate the limit vector into ne and u. ne is based on acceleration while u is set +-1 based on throttle saturation. Together they don't form a direction vector because the units are different.
    Vector3f    _limit_vector_ned;          // the direction that the position controller is limited, zero when not limited

    // terrain handling variables
    float    _pos_terrain_target_d_m;   // position terrain target in m relative to the EKF origin in NED frame
    float    _pos_terrain_d_m;          // position terrain in m from the EKF origin in NED frame. This terrain moves towards _pos_terrain_target_d_m
    float    _vel_terrain_d_ms;         // velocity terrain in NED m/s calculated by pos_to_rate step. This terrain moves towards _vel_terrain_target
    float    _accel_terrain_d_mss;      // acceleration terrain in NED m/s²

    // offset handling variables
    Vector3p    _pos_offset_target_ned_m;       // position offset target in m relative to the EKF origin in NED frame
    Vector3p    _pos_offset_ned_m;              // position offset in m from the EKF origin in NED frame. This offset moves towards _pos_offset_target_ned_m
    Vector3f    _vel_offset_target_ned_ms;      // velocity offset target in m/s in NED frame
    Vector3f    _vel_offset_ned_ms;             // velocity offset in NED m/s calculated by pos_to_rate step. This offset moves towards _vel_offset_target_ned_ms
    Vector3f    _accel_offset_target_ned_mss;   // acceleration offset target in m/s² in NED frame
    Vector3f    _accel_offset_ned_mss;          // acceleration offset in NED m/s²
    uint32_t    _posvelaccel_offset_target_ne_ms;   // system time that pos, vel, accel targets were set (used to implement timeouts)
    uint32_t    _posvelaccel_offset_target_d_ms;    // system time that pos, vel, accel targets were set (used to implement timeouts)

    // ekf reset handling
    uint32_t    _ekf_ne_reset_ms;       // system time of last recorded ekf ne position reset
    uint32_t    _ekf_d_reset_ms;        // system time of last recorded ekf altitude reset
    EKFResetMethod _ekf_reset_method = EKFResetMethod::MoveTarget;  // EKF reset handling method.  Loiter should use MoveTarget, Auto should use MoveVehicle

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
