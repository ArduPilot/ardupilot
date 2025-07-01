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
#define POSCONTROL_ACCEL_NE_MSS                 1.0f    // default horizontal acceleration in m/s/s.  This is overwritten by waypoint and loiter controllers
#define POSCONTROL_JERK_NE_MSSS                 5.0f    // default horizontal jerk m/s/s/s

#define POSCONTROL_STOPPING_DIST_UP_MAX_M       3.0f    // max stopping distance (in m) vertically while climbing
#define POSCONTROL_STOPPING_DIST_DOWN_MAX_M     2.0f    // max stopping distance (in m) vertically while descending

#define POSCONTROL_SPEED_MS                     5.0f    // default horizontal speed in m/s
#define POSCONTROL_SPEED_DOWN_MS                -1.5f   // default descent rate in m/s
#define POSCONTROL_SPEED_UP_MS                  2.5f    // default climb rate in m/s

#define POSCONTROL_ACCEL_U_MSS                  2.5f    // default vertical acceleration in m/s/s.
#define POSCONTROL_JERK_U_MSSS                  5.0f    // default vertical jerk m/s/s/s

#define POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ      2.0f    // low-pass filter on acceleration error (unit: Hz)

#define POSCONTROL_OVERSPEED_GAIN_U             2.0f    // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range

#define POSCONTROL_RELAX_TC                     0.16f   // This is used to decay the I term to 5% in half a second.

class AC_PosControl
{
public:

    /// Constructor
    AC_PosControl(AP_AHRS_View& ahrs, const class AP_Motors& motors, AC_AttitudeControl& attitude_control);

    // do not allow copying
    CLASS_NO_COPY(AC_PosControl);

    /// set_dt / get_dt - dt is the time in seconds since the last time the position controllers were updated
    ///   _dt_s should be set based on the time of the last IMU read used by these controllers
    ///   the position controller should run updates for active controllers on each loop to ensure normal operation
    void set_dt_s(float dt) { _dt_s = dt; }
    float get_dt_s() const { return _dt_s; }

    // Updates internal position and velocity estimates in the NED frame.
    // Falls back to vertical-only estimates if full NED data is unavailable.
    // When high_vibes is true, forces use of vertical fallback for velocity.
    void update_estimates(bool high_vibes = false);

    /// get_shaping_jerk_NE_msss - gets the jerk limit of the ne kinematic path generation in cm/s/s/s
    float get_shaping_jerk_NE_cmsss() const { return get_shaping_jerk_NE_msss() * 100.0; }
    float get_shaping_jerk_NE_msss() const { return _shaping_jerk_ne_msss; }


    ///
    /// 3D position shaper
    ///

    /// input_pos_NEU_m - computes a jerk-limited trajectory from the current NEU position, velocity, and acceleration to a new position input (in cm).
    /// This function updates the desired acceleration using a smooth kinematic path constrained by acceleration and jerk limits.
    void input_pos_NEU_cm(const Vector3p& pos_neu_cm, float pos_terrain_target_alt_cm, float terrain_buffer_cm);
    void input_pos_NEU_m(const Vector3p& pos_neu_m, float pos_terrain_target_alt_m, float terrain_buffer_m);

    /// pos_terrain_U_scaler_m - computes a scaling factor applied to horizontal velocity limits to ensure the vertical position controller remains within its terrain buffer.
    float pos_terrain_U_scaler_cm(float pos_terrain_u_cm, float pos_terrain_u_buffer_cm) const;
    float pos_terrain_U_scaler_m(float pos_terrain_u_m, float pos_terrain_u_buffer_m) const;

    ///
    /// Lateral position controller
    ///

    /// set_max_speed_accel_NE_m - set the maximum horizontal speed in cm/s and acceleration in cm/s/s
    ///     This function only needs to be called if using the kinematic shaping.
    ///     This can be done at any time as changes in these parameters are handled smoothly
    ///     by the kinematic shaping.
    void set_max_speed_accel_NE_cm(float speed_cms, float accel_cmss);
    void set_max_speed_accel_NE_m(float speed_ms, float accel_mss);

    /// set_correction_speed_accel_NE_m - set the position controller correction velocity and acceleration limit
    ///     This should be done only during initialisation to avoid discontinuities
    void set_correction_speed_accel_NE_cm(float speed_cms, float accel_cmss);
    void set_correction_speed_accel_NE_m(float speed_ms, float accel_mss);

    /// get_max_speed_NE_ms - get the maximum horizontal speed in cm/s
    float get_max_speed_NE_cms() const { return get_max_speed_NE_ms() * 100.0; }
    float get_max_speed_NE_ms() const { return _vel_max_ne_ms; }

    /// get_max_accel_NE_mss - get the maximum horizontal acceleration in cm/s/s
    float get_max_accel_NE_cmss() const { return get_max_accel_NE_mss() * 100.0; }
    float get_max_accel_NE_mss() const { return _accel_max_ne_mss; }

    // set_pos_error_max_NE_m - set the maximum horizontal position error that will be allowed in the horizontal plane
    void set_pos_error_max_NE_cm(float error_max_cm) { set_pos_error_max_NE_m(error_max_cm * 0.01); }
    void set_pos_error_max_NE_m(float error_max_m) { _p_pos_ne_m.set_error_max(error_max_m); }

    // get_pos_error_max_NE_m - return the maximum horizontal position error that will be allowed in the horizontal plane
    float get_pos_error_max_NE_cm() { return get_pos_error_max_NE_m() * 100.0; }
    float get_pos_error_max_NE_m() { return _p_pos_ne_m.get_error_max(); }

    /// init_NE_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
    ///     This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
    ///     The starting position can be retrieved by getting the position target using get_pos_target_NEU_m() after calling this function.
    void init_NE_controller_stopping_point();

    // relax_velocity_controller_NE - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function exponentially decays the acceleration output by ~95% over 0.5 seconds to achieve a smooth transition to zero requested acceleration.
    void relax_velocity_controller_NE();

    /// Reduces controller response for landing by decaying position error and preventing I-term windup.
    void soften_for_landing_NE();

    // init_NE_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function is private and contains all the shared ne axis initialisation functions
    void init_NE_controller();

    /// input_accel_NE_m - computes a jerk-limited trajectory to smoothly reach the specified acceleration in the NE plane from the current position, velocity, and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_ne.
    ///     The jerk limit defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
    ///     The jerk limit also defines the time taken to achieve the maximum acceleration.
    void input_accel_NE_cm(const Vector3f& accel_neu_cmsss);
    void input_accel_NE_m(const Vector3f& accel_neu_msss);

    /// input_vel_accel_NE_m - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_ne.
    ///     The function modifies vel_ne_ms to follow the kinematic trajectory toward accel_mss.
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    void input_vel_accel_NE_cm(Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output = true);
    void input_vel_accel_NE_m(Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss, bool limit_output = true);

    /// input_pos_vel_accel_NE_m - calculate a jerk limited path from the current position, velocity and acceleration to an input position velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_ne.
    ///     The function modifies pos_ne_m and vel_ne_ms to follow the jerk-limited trajectory defined by accel_ne_mss.
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    void input_pos_vel_accel_NE_cm(Vector2p& pos_ne_cm, Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output = true);
    void input_pos_vel_accel_NE_m(Vector2p& pos_ne_m, Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss, bool limit_output = true);

    // is_active_NE - returns true if the ne position controller has been run in the previous 5 loop times
    bool is_active_NE() const;

    /// stop_pos_NE_stabilisation - sets the target to the current position to remove any position corrections from the system
    void stop_pos_NE_stabilisation();

    /// stop_vel_NE_stabilisation - sets the target to the current position and velocity to the current velocity to remove any position and velocity corrections from the system
    void stop_vel_NE_stabilisation();

    /// set a single loop ne control scale factor. Set to zero to disable
    void set_NE_control_scale_factor(float ne_control_scale_factor) {
        _ne_control_scale_factor = ne_control_scale_factor;
    }

    /// update_NE_controller - runs the horizontal position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_NE_controller();

    ///
    /// Vertical position controller
    ///

    /// set_max_speed_accel_U_mss - set the maximum vertical speed in cm/s and acceleration in cm/s/s
    ///     speed_down_ms may be positive or negative, but it is always interpreted as a descent rate.
    ///     This can be done at any time as changes in these parameters are handled smoothly
    ///     by the kinematic shaping.
    void set_max_speed_accel_U_cm(float speed_down_cms, float speed_up_cms, float accel_cmss);
    void set_max_speed_accel_U_m(float speed_down_ms, float speed_up_ms, float accel_mss);

    /// set_correction_speed_accel_U_mss - set the position controller correction velocity and acceleration limit
    ///     speed_down_ms may be positive or negative, but it is always interpreted as a descent rate.
    ///     This should be done only during initialisation to avoid discontinuities
    void set_correction_speed_accel_U_cmss(float speed_down_cms, float speed_up_cms, float accel_cmss);
    void set_correction_speed_accel_U_mss(float speed_down_ms, float speed_up_ms, float accel_mss);

    /// get_max_accel_U_mss - get the maximum vertical acceleration in cm/s/s
    float get_max_accel_U_cmss() const { return get_max_accel_U_mss() * 100.0; }
    float get_max_accel_U_mss() const { return _accel_max_u_mss; }

    // get_pos_error_up_m - get the allowed upper bound of vertical position error (positive direction)
    float get_pos_error_up_cm() { return get_pos_error_up_m() * 100.0; }
    float get_pos_error_up_m() { return _p_pos_u_m.get_error_max(); }

    // get_pos_error_down_m - get the allowed lower bound of vertical position error (negative direction)
    float get_pos_error_down_cm() { return get_pos_error_down_m() * 100.0; }
    float get_pos_error_down_m() { return _p_pos_u_m.get_error_min(); }

    /// get_max_speed_up_ms - accessors for current maximum up speed in cm/s
    float get_max_speed_up_cms() const { return get_max_speed_up_ms() * 100.0; }
    float get_max_speed_up_ms() const { return _vel_max_up_ms; }

    /// get_max_speed_down_ms - accessors for current maximum down speed in cm/s.  Will be a negative number
    float get_max_speed_down_cms() const { return get_max_speed_down_ms() * 100.0; }
    float get_max_speed_down_ms() const { return _vel_max_down_ms; }

    /// init_U_controller_no_descent - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function does not allow any negative velocity or acceleration
    void init_U_controller_no_descent();

    /// init_U_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
    ///     This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
    ///     The starting position can be retrieved by getting the position target using get_pos_target_NEU_m() after calling this function.
    void init_U_controller_stopping_point();

    // relax_U_controller - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
    void relax_U_controller(float throttle_setting);

    // init_U_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function is private and contains all the shared z axis initialisation functions
    void init_U_controller();

    /// input_accel_U_m - calculate a jerk limited path from the current position, velocity and acceleration to an input acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_z.
    virtual void input_accel_U_cm(float accel_u_cmss);
    virtual void input_accel_U_m(float accel_u_mss);

    /// input_vel_accel_U_m - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_z.
    ///     The function modifies vel_u_ms to follow the jerk-limited trajectory defined by accel_u_mss.
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    virtual void input_vel_accel_U_cm(float &vel_u_cms, float accel_u_cmss, bool limit_output = true);
    virtual void input_vel_accel_U_m(float &vel_u_ms, float accel_u_mss, bool limit_output = true);

    /// set_pos_target_U_from_climb_rate_m - adjusts target up or down using a commanded climb rate in cm/s
    ///     using the default jerk-limited kinematic shaping method.
    ///     The zero target altitude is varied to follow pos_offset_u
    void set_pos_target_U_from_climb_rate_cm(float vel_u_cms);
    void set_pos_target_U_from_climb_rate_m(float vel_u_ms);

    /// land_at_climb_rate_m - adjusts target up or down using a commanded climb rate in cm/s
    ///     using the default jerk-limited kinematic shaping method.
    ///     ignore_descent_limit turns off output saturation handling to aid in landing detection. ignore_descent_limit should be true unless landing.
    void land_at_climb_rate_cm(float vel_u_cms, bool ignore_descent_limit);
    void land_at_climb_rate_m(float vel_u_ms, bool ignore_descent_limit);

    /// input_pos_vel_accel_U_m - calculate a jerk limited path from the current position, velocity and acceleration to an input position velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The function alters the pos_u_m and vel_u_ms to be the kinematic path based on accel_u_mss
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    void input_pos_vel_accel_U_cm(float &pos_u_cm, float &vel_u_cms, float accel_u_cmss, bool limit_output = true);
    void input_pos_vel_accel_U_m(float &pos_u_m, float &vel_u_ms, float accel_u_mss, bool limit_output = true);

    /// set_alt_target_with_slew_m - adjusts target up or down using a commanded altitude in cm
    ///     using the default jerk-limited kinematic shaping method.
    void set_alt_target_with_slew_cm(float pos_u_cm);
    void set_alt_target_with_slew_m(float pos_u_m);

    // is_active_U - returns true if the z position controller has been run in the previous 5 loop times
    bool is_active_U() const;

    /// update_U_controller - runs the vertical position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_U_controller();



    ///
    /// Accessors
    ///

    /// set commanded position (cm), velocity (cm/s) and acceleration (cm/s/s) inputs when the path is created externally.
    void set_pos_vel_accel_NEU_cm(const Vector3p& pos_neu_cm, const Vector3f& vel_neu_cms, const Vector3f& accel_neu_cmss);
    void set_pos_vel_accel_NEU_m(const Vector3p& pos_neu_m, const Vector3f& vel_neu_ms, const Vector3f& accel_neu_mss);
    void set_pos_vel_accel_NE_cm(const Vector2p& pos_ne_cm, const Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss);
    void set_pos_vel_accel_NE_m(const Vector2p& pos_ne_m, const Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss);


    /// Position

    /// get_pos_estimate_NEU_m - returns the current position estimate, frame NEU in cm relative to the EKF origin
    const Vector3p get_pos_estimate_NEU_cm() const { return get_pos_estimate_NEU_m() * 100.0; }
    const Vector3p& get_pos_estimate_NEU_m() const { return _pos_estimate_neu_m; }

    /// get_pos_target_NEU_m - returns the position target, frame NEU in cm relative to the EKF origin
    const Vector3p get_pos_target_NEU_cm() const { return get_pos_target_NEU_m() * 100.0; }
    const Vector3p& get_pos_target_NEU_m() const { return _pos_target_neu_m; }

    /// set_pos_desired_NE_m - sets the position target, frame NEU in cm relative to the EKF origin
    void set_pos_desired_NE_cm(const Vector2f& pos_desired_ne_cm) { set_pos_desired_NE_m(pos_desired_ne_cm * 0.01); }
    void set_pos_desired_NE_m(const Vector2f& pos_desired_ne_m) { _pos_desired_neu_m.xy() = pos_desired_ne_m.topostype(); }

    /// get_pos_desired_NEU_m - returns the position desired, frame NEU in cm relative to the EKF origin
    const Vector3p get_pos_desired_NEU_cm() const { return get_pos_desired_NEU_m() * 100.0; }
    const Vector3p& get_pos_desired_NEU_m() const { return _pos_desired_neu_m; }

    /// get_pos_target_U_m - get target altitude (in cm above the EKF origin)
    float get_pos_target_U_cm() const { return get_pos_target_U_m() * 100.0; }
    float get_pos_target_U_m() const { return _pos_target_neu_m.z; }

    /// set_pos_desired_U_m - set altitude target in cm above the EKF origin
    void set_pos_desired_U_cm(float pos_desired_u_cm) { set_pos_desired_U_m(pos_desired_u_cm * 0.01); }
    void set_pos_desired_U_m(float pos_desired_u_m) { _pos_desired_neu_m.z = pos_desired_u_m; }

    /// get_pos_desired_U_m - get target altitude (in cm above the EKF origin)
    float get_pos_desired_U_cm() const { return get_pos_desired_U_m() * 100.0; }
    float get_pos_desired_U_m() const { return _pos_desired_neu_m.z; }


    /// Stopping Point

    /// get_stopping_point_NE_m - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration
    void get_stopping_point_NE_cm(Vector2p &stopping_point_neu_cm) const;
    void get_stopping_point_NE_m(Vector2p &stopping_point_neu_m) const;

    /// get_stopping_point_U_m - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration
    void get_stopping_point_U_cm(postype_t &stopping_point_u_cm) const;
    void get_stopping_point_U_m(postype_t &stopping_point_u_m) const;


    /// Position Error

    /// get_pos_error_NEU_m - returns the 3D position error vector between the current and target NEU positions.
    const Vector3f get_pos_error_NEU_cm() const { return get_pos_error_NEU_m() * 100.0; }
    const Vector3f get_pos_error_NEU_m() const { return Vector3f(_p_pos_ne_m.get_error().x, _p_pos_ne_m.get_error().y, _p_pos_u_m.get_error()); }

    /// get_pos_error_NE_m - get the length of the position error vector in the ne plane
    float get_pos_error_NE_cm() const { return get_pos_error_NE_m() * 100.0; }
    float get_pos_error_NE_m() const { return _p_pos_ne_m.get_error().length(); }

    /// get_pos_error_U_m - returns altitude error in cm
    float get_pos_error_U_cm() const { return get_pos_error_U_m() * 100.0; }
    float get_pos_error_U_m() const { return _p_pos_u_m.get_error(); }


    /// Velocity

    /// get_vel_estimate_NEU_ms - returns current velocity estimate in cm/s in NEU
    const Vector3f get_vel_estimate_NEU_cms() const { return get_vel_estimate_NEU_ms() * 100.0; }
    const Vector3f& get_vel_estimate_NEU_ms() const { return _vel_estimate_neu_ms; }

    /// set_vel_desired_NEU_ms - sets desired velocity in NEU cm/s
    void set_vel_desired_NEU_cms(const Vector3f &vel_desired_neu_cms) { set_vel_desired_NEU_ms(vel_desired_neu_cms * 0.01); }
    void set_vel_desired_NEU_ms(const Vector3f &vel_desired_neu_ms) { _vel_desired_neu_ms = vel_desired_neu_ms; }

    /// set_vel_desired_NE_ms - sets the desired horizontal velocity (NE only) in cm/s.
    void set_vel_desired_NE_cms(const Vector2f &vel_desired_ne_cms) { set_vel_desired_NE_ms(vel_desired_ne_cms * 0.01); }
    void set_vel_desired_NE_ms(const Vector2f &vel_desired_ne_ms) { _vel_desired_neu_ms.xy() = vel_desired_ne_ms; }

    /// get_vel_desired_NEU_ms - returns desired velocity in cm/s in NEU
    const Vector3f get_vel_desired_NEU_cms() const { return get_vel_desired_NEU_ms() * 100.0; }
    const Vector3f& get_vel_desired_NEU_ms() const { return _vel_desired_neu_ms; }

    // get_vel_target_NEU_ms - returns the target velocity in NEU cm/s
    const Vector3f get_vel_target_NEU_cms() const { return get_vel_target_NEU_ms() * 100.0; }
    const Vector3f& get_vel_target_NEU_ms() const { return _vel_target_neu_ms; }

    /// set_vel_desired_U_ms - sets desired velocity in cm/s in z axis
    void set_vel_desired_U_cms(float vel_desired_u_cms) { set_vel_desired_U_ms(vel_desired_u_cms * 0.01); }

    void set_vel_desired_U_ms(float vel_desired_u_ms) { _vel_desired_neu_ms.z = vel_desired_u_ms; }

    /// get_vel_target_U_ms - returns target vertical speed in cm/s
    float get_vel_target_U_cms() const { return get_vel_target_U_ms() * 100.0; }
    float get_vel_target_U_ms() const { return _vel_target_neu_ms.z; }


    /// Acceleration

    // set_accel_desired_NE_mss - set desired acceleration in cm/s in ne axis
    void set_accel_desired_NE_cmss(const Vector2f &accel_desired_neu_cmss) { set_accel_desired_NE_mss(accel_desired_neu_cmss * 0.01); }
    void set_accel_desired_NE_mss(const Vector2f &accel_desired_neu_mss) { _accel_desired_neu_mss.xy() = accel_desired_neu_mss; }

    // get_accel_target_NEU_mss - returns the target acceleration in NEU cm/s/s
    const Vector3f get_accel_target_NEU_cmss() const { return get_accel_target_NEU_mss() * 100.0; }
    const Vector3f& get_accel_target_NEU_mss() const { return _accel_target_neu_mss; }


    /// Terrain

    // set_pos_terrain_target_U_m - set target terrain altitude in cm
    void set_pos_terrain_target_U_cm(float pos_terrain_target_u_cm) { set_pos_terrain_target_U_m(pos_terrain_target_u_cm * 0.01); }
    void set_pos_terrain_target_U_m(float pos_terrain_target_u_m) { _pos_terrain_target_u_m = pos_terrain_target_u_m; }

    // init_pos_terrain_U_m - initialises the current terrain altitude and target altitude to pos_offset_terrain_m
    void init_pos_terrain_U_cm(float pos_terrain_u_cm);
    void init_pos_terrain_U_m(float pos_terrain_u_m);

    // get_pos_terrain_U_m - returns the current terrain altitude in cm
    float get_pos_terrain_U_cm() const { return get_pos_terrain_U_m() * 100.0; }
    float get_pos_terrain_U_m() const { return _pos_terrain_u_m; }


    /// Offset

#if AP_SCRIPTING_ENABLED
    // position, velocity and acceleration offset target (only used by scripting)
    // gets or sets an additional offset to the vehicle's target position, velocity and acceleration
    // units are m, m/s and m/s/s in NED frame
    bool set_posvelaccel_offset(const Vector3f &pos_offset_NED_m, const Vector3f &vel_offset_NED_ms, const Vector3f &accel_offset_NED_mss);

    bool get_posvelaccel_offset(Vector3f &pos_offset_NED_m, Vector3f &vel_offset_NED_ms, Vector3f &accel_offset_NED_mss);

    // get target velocity in m/s in NED frame
    bool get_vel_target(Vector3f &vel_target_NED_ms);

    // get target acceleration in m/s/s in NED frame
    bool get_accel_target(Vector3f &accel_target_NED_mss);
#endif

    /// set the horizontal position, velocity and acceleration offset targets in cm, cms and cm/s/s from EKF origin in NE frame.
    /// These offsets must be updated at least every 3 seconds or they will timeout and revert to zero.
    void set_posvelaccel_offset_target_NE_cm(const Vector2p& pos_offset_target_ne_cm, const Vector2f& vel_offset_target_ne_cms, const Vector2f& accel_offset_target_ne_cmss);
    void set_posvelaccel_offset_target_NE_m(const Vector2p& pos_offset_target_ne_m, const Vector2f& vel_offset_target_ne_ms, const Vector2f& accel_offset_target_ne_mss);
    
    void set_posvelaccel_offset_target_U_cm(float pos_offset_target_u_cm, float vel_offset_target_u_cms, float accel_offset_target_u_cmss);
    void set_posvelaccel_offset_target_U_m(float pos_offset_target_u_m, float vel_offset_target_u_ms, float accel_offset_target_u_mss);

    /// get the position, velocity or acceleration offets in cm from EKF origin in NEU frame
    const Vector3p get_pos_offset_NEU_cm() const { return get_pos_offset_NEU_m() * 100.0; }
    const Vector3p& get_pos_offset_NEU_m() const { return _pos_offset_neu_m; }
    
    const Vector3f get_vel_offset_NEU_cms() const { return get_vel_offset_NEU_ms() * 100.0; }
    const Vector3f& get_vel_offset_NEU_ms() const { return _vel_offset_neu_ms; }
    
    const Vector3f get_accel_offset_NEU_cmss() const { return get_accel_offset_NEU_mss() * 100.0; }
    const Vector3f& get_accel_offset_NEU_mss() const { return _accel_offset_neu_mss; }

    /// set_pos_offset_U_m - set altitude offset in cm above the EKF origin
    void set_pos_offset_U_m(float pos_offset_u_m) { _pos_offset_neu_m.z = pos_offset_u_m; }

    /// get_pos_offset_U_m - returns altitude offset in cm above the EKF origin
    float get_pos_offset_U_cm() const { return get_pos_offset_U_m() * 100.0; }
    float get_pos_offset_U_m() const { return _pos_offset_neu_m.z; }

    /// get_vel_offset_U_ms - returns current vertical offset speed in cm/s
    float get_vel_offset_U_cms() const { return get_vel_offset_U_ms() * 100.0; }
    float get_vel_offset_U_ms() const { return _vel_offset_neu_ms.z; }

    /// get_accel_offset_U_mss - returns current vertical offset acceleration in cm/s/s
    float get_accel_offset_U_cmss() const { return get_accel_offset_U_mss() * 100.0; }
    float get_accel_offset_U_mss() const { return _accel_offset_neu_mss.z; }

    /// Outputs

    /// Returns the desired roll angle in radians for the attitude controller.
    float get_roll_rad() const { return _roll_target_rad; }

    /// Returns the desired pitch angle in radians for the attitude controller.
    float get_pitch_rad() const { return _pitch_target_rad; }

    /// Returns the desired yaw angle in radians for the attitude controller.
    float get_yaw_rad() const { return _yaw_target_rad; }

    /// Returns the desired yaw rate in radians/second for the attitude controller.
    float get_yaw_rate_rads() const { return _yaw_rate_target_rads; }

    /// Returns the desired roll angle in centidegrees for the attitude controller.
    float get_roll_cd() const { return rad_to_cd(_roll_target_rad); }

    /// Returns the desired pitch angle in centidegrees for the attitude controller.
    float get_pitch_cd() const { return rad_to_cd(_pitch_target_rad); }

    /// Returns the desired yaw angle in centidegrees for the attitude controller.
    float get_yaw_cd() const { return rad_to_cd(_yaw_target_rad); }

    /// Returns the desired yaw rate in centidegrees/second for the attitude controller.
    float get_yaw_rate_cds() const { return rad_to_cd(_yaw_rate_target_rads); }

    /// Returns the desired thrust direction vector in the body frame.
    Vector3f get_thrust_vector() const;

    /// Returns the bearing to the position target in centidegrees (0 = North, CW positive).
    float get_bearing_to_target_rad() const;

    /// Returns the maximum allowed lean angle in radians (roll/pitch) for the attitude controller.
    float get_lean_angle_max_rad() const;

    /// Sets the maximum allowed lean angle in radians (roll/pitch) for the attitude controller.
    /// A value of zero means to use the ANGLE_MAX parameter. This is reset to zero by init_NE_controller().
    void set_lean_angle_max_rad(float angle_max_rad) { _angle_max_override_rad = angle_max_rad; }

    /// Sets the maximum allowed lean angle in degrees (roll/pitch) for the attitude controller.
    /// See set_lean_angle_max_rad() for full details.
    void set_lean_angle_max_deg(float angle_max_deg) { set_lean_angle_max_rad(radians(angle_max_deg)); }

    /// Sets the maximum allowed lean angle in centidegrees (roll/pitch) for the attitude controller.
    /// See set_lean_angle_max_rad() for full details.
    void set_lean_angle_max_cd(float angle_max_cd) { set_lean_angle_max_rad(cd_to_rad(angle_max_cd)); }

    /// Other

    /// get pid controllers
    AC_P_2D& get_pos_NE_p() { return _p_pos_ne_m; }
    AC_P_1D& get_pos_U_p() { return _p_pos_u_m; }
    AC_PID_2D& get_vel_NE_pid() { return _pid_vel_ne_cm; }
    AC_PID_Basic& get_vel_U_pid() { return _pid_vel_u_cm; }
    AC_PID& get_accel_U_pid() { return _pid_accel_u_cm_to_kt; }

    /// set_externally_limited_NE - mark that accel has been limited
    ///     this prevents integrator windup during external acceleration saturation
    void set_externally_limited_NE() { _limit_vector_neu.x = _accel_target_neu_mss.x; _limit_vector_neu.y = _accel_target_neu_mss.y; }

    // lean_angles_rad_to_accel_NEU_mss - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    Vector3f lean_angles_rad_to_accel_NEU_cmss(const Vector3f& att_target_euler_rad) const;
    Vector3f lean_angles_rad_to_accel_NEU_mss(const Vector3f& att_target_euler_rad) const;

    // write PSC and/or PSCZ logs
    void write_log();

    // provide feedback on whether arming would be a good idea right now:
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    // enable or disable high vibration compensation
    void set_vibe_comp(bool on_off) { _vibe_comp_enabled = on_off; }

    /// get_vel_U_control_ratio - returns the proportion of control authority used on the vertical axis (0 to 1)
    float get_vel_U_control_ratio() const { return constrain_float(_vel_u_control_ratio, 0.0f, 1.0f); }

    /// crosstrack_error - returns horizontal error to the closest point to the current track
    float crosstrack_error() const;

    /// standby_NEU_reset - resets I terms and removes position error
    ///     This function will let Loiter and Alt Hold continue to operate
    ///     in the event that the flight controller is in control of the
    ///     aircraft when in standby.
    void standby_NEU_reset();

    // get_measured_accel_U_mss - returns the vertical (Up) acceleration in the earth frame, gravity-compensated, in cm/s/s (+ve = upward)
    float get_measured_accel_U_cmss() const { return get_measured_accel_U_mss() * 100.0; }
    float get_measured_accel_U_mss() const { return -(_ahrs.get_accel_ef().z + GRAVITY_MSS); }

    /// returns true when the forward pitch demand is limited by the maximum allowed tilt
    bool get_fwd_pitch_is_limited() const;
    
    // set_disturb_pos_NE_m - set the position disturbance in the north east plane
    void set_disturb_pos_NE_cm(Vector2f disturb_pos_cm) { set_disturb_pos_NE_m(disturb_pos_cm * 0.01); }
    void set_disturb_pos_NE_m(Vector2f disturb_pos_m) { _disturb_pos_ne_m = disturb_pos_m; }

    // set_disturb_vel_NE_ms - set the velocity disturbance in the north east plane
    void set_disturb_vel_NE_cms(Vector2f disturb_vel_cms) { set_disturb_vel_NE_ms(disturb_vel_cms * 0.01); }
    void set_disturb_vel_NE_ms(Vector2f disturb_vel_ms) { _disturb_vel_ne_ms = disturb_vel_ms; }

    static const struct AP_Param::GroupInfo var_info[];

    static void Write_PSCN(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss);
    static void Write_PSCE(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss);
    static void Write_PSCD(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss);
    static void Write_PSON(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);
    static void Write_PSOE(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);
    static void Write_PSOD(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);
    static void Write_PSOT(float pos_target_offset_m, float pos_offset_m, float vel_target_offset_ms, float vel_offset_ms, float accel_target_offset_mss, float accel_offset_mss);

    // singleton
    static AC_PosControl *get_singleton(void) { return _singleton; }

protected:

    // get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain)
    float get_throttle_with_vibration_override();

    // accel_NE_mss_to_lean_angles_rad - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void accel_NE_cmss_to_lean_angles_rad(float accel_n_cmss, float accel_e_cmss, float& roll_target_rad, float& pitch_target_rad) const;
    void accel_NE_mss_to_lean_angles_rad(float accel_n_mss, float accel_e_mss, float& roll_target_rad, float& pitch_target_rad) const;

    // lean_angles_to_accel_NE_mss - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void lean_angles_to_accel_NE_cmss(float& accel_n_cmss, float& accel_e_cmss) const;
    void lean_angles_to_accel_NE_mss(float& accel_n_mss, float& accel_e_mss) const;

    // calculate_yaw_and_rate_yaw - calculate the vehicle yaw and rate of yaw.
    void calculate_yaw_and_rate_yaw();

    // calculate_overspeed_gain - calculated increased maximum acceleration and jerk if over speed condition is detected
    float calculate_overspeed_gain();


    /// Terrain Following

    /// set the position, velocity and acceleration offsets in cm, cms and cm/s/s from EKF origin in NE frame
    /// this is used to initiate the offsets when initialise the position controller or do an offset reset
    /// note that this sets the actual offsets, not the offset targets
    void init_terrain();

    /// update_terrain - updates the terrain position, velocity and acceleration estimation
    /// this moves the estimated terrain position _pos_terrain_u_m towards the target _pos_terrain_target_u_m
    void update_terrain();


    /// Offsets

    /// init_offsets - set the position, velocity and acceleration offsets in cm, cms and cm/s/s from EKF origin in NE frame
    /// this is used to initiate the offsets when initialise the position controller or do an offset reset
    /// note that this sets the actual offsets, not the offset targets
    void init_offsets_NE();
    void init_offsets_U();

    /// update_offsets - update the position and velocity offsets
    /// this moves the offsets (e.g _pos_offset_neu_m, _vel_offset_neu_ms, _accel_offset_neu_mss) towards the targets (e.g. _pos_offset_target_neu_m or _vel_offset_target_neu_ms)
    void update_offsets_NE();
    void update_offsets_U();

    /// initialise and check for ekf position resets
    void init_ekf_NE_reset();
    void handle_ekf_NE_reset();
    void init_ekf_U_reset();
    void handle_ekf_U_reset();

    // references to inertial nav and ahrs libraries
    AP_AHRS_View&           _ahrs;
    const class AP_Motors&  _motors;
    AC_AttitudeControl&     _attitude_control;

    // parameters
    AP_Float        _lean_angle_max_deg;    // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AP_Float        _shaping_jerk_ne_msss;  // Jerk limit of the ne kinematic path generation in m/s^3 used to determine how quickly the aircraft varies the acceleration target
    AP_Float        _shaping_jerk_u_msss;   // Jerk limit of the u kinematic path generation in m/s^3 used to determine how quickly the aircraft varies the acceleration target
    AC_P_2D         _p_pos_ne_m;           // XY axis position controller to convert target distance (cm) to target velocity (cm/s)
    AC_P_1D         _p_pos_u_m;            // Z axis position controller to convert target altitude (cm) to target climb rate (cm/s)
    AC_PID_2D       _pid_vel_ne_cm;         // XY axis velocity controller to convert target velocity (cm/s) to target acceleration (cm/s^2)
    AC_PID_Basic    _pid_vel_u_cm;          // Z axis velocity controller to convert target climb rate (cm/s) to target acceleration (cm/s^2)
    AC_PID          _pid_accel_u_cm_to_kt;  // Z axis acceleration controller to convert target acceleration (cm/s^2) to throttle output (0 to 1000)

    // internal variables
    float       _dt_s;                     // time difference (in seconds) since the last loop time
    uint32_t    _last_update_ne_ticks;     // ticks of last last update_NE_controller call
    uint32_t    _last_update_u_ticks;      // ticks of last update_z_controller call
    float       _vel_max_ne_ms;            // max horizontal speed in cm/s used for kinematic shaping
    float       _vel_max_up_ms;            // max climb rate in cm/s used for kinematic shaping
    float       _vel_max_down_ms;          // max descent rate in cm/s used for kinematic shaping
    float       _accel_max_ne_mss;         // max horizontal acceleration in cm/s/s used for kinematic shaping
    float       _accel_max_u_mss;          // max vertical acceleration in cm/s/s used for kinematic shaping
    float       _jerk_max_ne_msss;         // Jerk limit of the ne kinematic path generation in cm/s^3 used to determine how quickly the aircraft varies the acceleration target
    float       _jerk_max_u_msss;          // Jerk limit of the z kinematic path generation in cm/s^3 used to determine how quickly the aircraft varies the acceleration target
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
    Vector3p    _pos_desired_neu_m;        // desired location, frame NEU in cm relative to the EKF origin.  This is equal to the _pos_target minus offsets
    Vector3p    _pos_target_neu_m;         // target location, frame NEU in cm relative to the EKF origin.  This is equal to the _pos_desired_neu_m plus offsets
    Vector3f    _vel_estimate_neu_ms;
    Vector3f    _vel_desired_neu_ms;       // desired velocity in NEU cm/s
    Vector3f    _vel_target_neu_ms;        // velocity target in NEU cm/s calculated by pos_to_rate step
    Vector3f    _accel_desired_neu_mss;    // desired acceleration in NEU cm/s/s (feed forward)
    Vector3f    _accel_target_neu_mss;     // acceleration target in NEU cm/s/s
    // todo: seperate the limit vector into ne and u. ne is based on acceleration while u is set +-1 based on throttle saturation. Together they don't form a direction vector because the units are different.
    Vector3f    _limit_vector_neu;         // the direction that the position controller is limited, zero when not limited

    // terrain handling variables
    float    _pos_terrain_target_u_m;    // position terrain target in cm relative to the EKF origin in NEU frame
    float    _pos_terrain_u_m;           // position terrain in cm from the EKF origin in NEU frame.  this terrain moves towards _pos_terrain_target_u_m
    float    _vel_terrain_u_ms;          // velocity terrain in NEU cm/s calculated by pos_to_rate step.  this terrain moves towards _vel_terrain_target
    float    _accel_terrain_u_mss;       // acceleration terrain in NEU cm/s/s

    // offset handling variables
    Vector3p    _pos_offset_target_neu_m;      // position offset target in cm relative to the EKF origin in NEU frame
    Vector3p    _pos_offset_neu_m;             // position offset in cm from the EKF origin in NEU frame.  this offset moves towards _pos_offset_target_neu_m
    Vector3f    _vel_offset_target_neu_ms;     // velocity offset target in cm/s in NEU frame
    Vector3f    _vel_offset_neu_ms;            // velocity offset in NEU cm/s calculated by pos_to_rate step.  this offset moves towards _vel_offset_target_neu_ms
    Vector3f    _accel_offset_target_neu_mss;  // acceleration offset target in cm/s/s in NEU frame
    Vector3f    _accel_offset_neu_mss;         // acceleration offset in NEU cm/s/s
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
    // convenience method for writing PSCE, PSCN, and PSCD logs, to reduce code duplication
    static void Write_PSCx(LogMessages ID, float pos_desired_m, float pos_target_m, float pos_m, 
                            float vel_desired_ms, float vel_target_ms, float vel_ms, 
                            float accel_desired_mss, float accel_target_mss, float accel_mss);

    // a convenience function for writing out the position controller offsets
    static void Write_PSOx(LogMessages id, float pos_target_offset_m, float pos_offset_m,
                            float vel_target_offset_ms, float vel_offset_ms,
                            float accel_target_offset_mss, float accel_offset_mss);

    // singleton
    static AC_PosControl *_singleton;
};
