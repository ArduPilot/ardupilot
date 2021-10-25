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
#include <AP_InertialNav/AP_InertialNav.h>  // Inertial Navigation library
#include "AC_AttitudeControl.h"     // Attitude control library
#include <AP_Motors/AP_Motors.h>    // motors library
#include <AP_Vehicle/AP_Vehicle.h>  // common vehicle parameters


// position controller default definitions
#define POSCONTROL_ACCEL_XY                     100.0f  // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
#define POSCONTROL_JERK_XY                      5.0f    // default horizontal jerk m/s/s/s

#define POSCONTROL_STOPPING_DIST_UP_MAX         300.0f  // max stopping distance (in cm) vertically while climbing
#define POSCONTROL_STOPPING_DIST_DOWN_MAX       200.0f  // max stopping distance (in cm) vertically while descending

#define POSCONTROL_SPEED                        500.0f  // default horizontal speed in cm/s
#define POSCONTROL_SPEED_DOWN                  -150.0f  // default descent rate in cm/s
#define POSCONTROL_SPEED_UP                     250.0f  // default climb rate in cm/s

#define POSCONTROL_ACCEL_Z                      250.0f  // default vertical acceleration in cm/s/s.
#define POSCONTROL_JERK_Z                       5.0f    // default vertical jerk m/s/s/s

#define POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ      2.0f    // low-pass filter on acceleration error (unit: Hz)

#define POSCONTROL_OVERSPEED_GAIN_Z             2.0f    // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range

#define POSCONTROL_RELAX_TC                     0.16f   // This is used to decay the relevant variable to 5% in half a second.

class AC_PosControl
{
public:

    /// Constructor
    AC_PosControl(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                  const AP_Motors& motors, AC_AttitudeControl& attitude_control, float dt);

    /// get_dt - gets time delta in seconds for all position controllers
    float get_dt() const { return _dt; }

    /// get_shaping_jerk_xy_cmsss - gets the jerk limit of the xy kinematic path generation in cm/s/s/s
    float get_shaping_jerk_xy_cmsss() const { return _shaping_jerk_xy*100.0; }


    ///
    /// 3D position shaper
    ///

    /// input_pos_xyz - calculate a jerk limited path from the current position, velocity and acceleration to an input position.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_xy.
    void input_pos_xyz(const Vector3p& pos, float pos_offset_z, float pos_offset_z_buffer);

    /// pos_offset_z_scaler - calculates a multiplier used to reduce the horizontal velocity to allow the z position controller to stay within the provided buffer range
    float pos_offset_z_scaler(float pos_offset_z, float pos_offset_z_buffer) const;

    ///
    /// Lateral position controller
    ///

    /// set_max_speed_accel_xy - set the maximum horizontal speed in cm/s and acceleration in cm/s/s
    ///     This function only needs to be called if using the kinematic shaping.
    ///     This can be done at any time as changes in these parameters are handled smoothly
    ///     by the kinematic shaping.
    void set_max_speed_accel_xy(float speed_cms, float accel_cmss);

    /// set_max_speed_accel_xy - set the position controller correction velocity and acceleration limit
    ///     This should be done only during initialisation to avoid discontinuities
    void set_correction_speed_accel_xy(float speed_cms, float accel_cmss);

    /// get_max_speed_xy_cms - get the maximum horizontal speed in cm/s
    float get_max_speed_xy_cms() const { return _vel_max_xy_cms; }

    /// get_max_accel_xy_cmss - get the maximum horizontal acceleration in cm/s/s
    float get_max_accel_xy_cmss() const { return _accel_max_xy_cmss; }

    // set the maximum horizontal position error that will be allowed in the horizontal plane
    void set_pos_error_max_xy_cm(float error_max) { _p_pos_xy.set_error_max(error_max); }
    float get_pos_error_max_xy_cm() { return _p_pos_xy.get_error_max(); }

    /// init_xy_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    void init_xy_controller();

    /// init_xy_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
    ///     This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
    ///     The starting position can be retrieved by getting the position target using get_pos_target_cm() after calling this function.
    void init_xy_controller_stopping_point();

    // relax_velocity_controller_xy - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
    void relax_velocity_controller_xy();

    /// input_accel_xy - calculate a jerk limited path from the current position, velocity and acceleration to an input acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_xy.
    ///     The jerk limit defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
    ///     The jerk limit also defines the time taken to achieve the maximum acceleration.
    void input_accel_xy(const Vector3f& accel);

    /// input_vel_accel_xy - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_xy.
    ///     The function alters the vel to be the kinematic path based on accel
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    void input_vel_accel_xy(Vector2f& vel, const Vector2f& accel, bool limit_output = true);

    /// input_pos_vel_accel_xy - calculate a jerk limited path from the current position, velocity and acceleration to an input position velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_xy.
    ///     The function alters the pos and vel to be the kinematic path based on accel
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    void input_pos_vel_accel_xy(Vector2p& pos, Vector2f& vel, const Vector2f& accel, bool limit_output = true);

    // is_active_xy - returns true if the xy position controller has been run in the previous 5 loop times
    bool is_active_xy() const;

    /// stop_pos_xy_stabilisation - sets the target to the current position to remove any position corrections from the system
    void stop_pos_xy_stabilisation();

    /// stop_vel_xy_stabilisation - sets the target to the current position and velocity to the current velocity to remove any position and velocity corrections from the system
    void stop_vel_xy_stabilisation();

    /// update_xy_controller - runs the horizontal position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_xy_controller();

    ///
    /// Vertical position controller
    ///

    /// set_max_speed_accel_z - set the maximum vertical speed in cm/s and acceleration in cm/s/s
    ///     speed_down can be positive or negative but will always be interpreted as a descent speed
    ///     This can be done at any time as changes in these parameters are handled smoothly
    ///     by the kinematic shaping.
    void set_max_speed_accel_z(float speed_down, float speed_up, float accel_cmss);

    /// set_correction_speed_accel_z - set the position controller correction velocity and acceleration limit
    ///     speed_down can be positive or negative but will always be interpreted as a descent speed
    ///     This should be done only during initialisation to avoid discontinuities
    void set_correction_speed_accel_z(float speed_down, float speed_up, float accel_cmss);

    /// get_max_accel_z_cmss - get the maximum vertical acceleration in cm/s/s
    float get_max_accel_z_cmss() const { return _accel_max_z_cmss; }

    // get_pos_error_z_up_cm - get the maximum vertical position error up that will be allowed
    float get_pos_error_z_up_cm() { return _p_pos_z.get_error_max(); }

    // get_pos_error_z_down_cm - get the maximum vertical position error down that will be allowed
    float get_pos_error_z_down_cm() { return _p_pos_z.get_error_min(); }

    /// get_max_speed_up_cms - accessors for current maximum up speed in cm/s
    float get_max_speed_up_cms() const { return _vel_max_up_cms; }

    /// get_max_speed_down_cms - accessors for current maximum down speed in cm/s.  Will be a negative number
    float get_max_speed_down_cms() const { return _vel_max_down_cms; }

    /// init_z_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    void init_z_controller();

    /// init_z_controller_no_descent - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function does not allow any negative velocity or acceleration
    void init_z_controller_no_descent();

    /// init_z_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
    ///     This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
    ///     The starting position can be retrieved by getting the position target using get_pos_target_cm() after calling this function.
    void init_z_controller_stopping_point();

    // relax_z_controller - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
    void relax_z_controller(float throttle_setting);

    /// input_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_z.
    virtual void input_accel_z(float accel);

    /// input_vel_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_z.
    ///     The function alters the vel to be the kinematic path based on accel
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    virtual void input_vel_accel_z(float &vel, float accel, bool ignore_descent_limit, bool limit_output = true);

    /// set_pos_target_z_from_climb_rate_cm - adjusts target up or down using a commanded climb rate in cm/s
    ///     using the default position control kinematic path.
    ///     The zero target altitude is varied to follow pos_offset_z
    void set_pos_target_z_from_climb_rate_cm(float vel);

    /// land_at_climb_rate_cm - adjusts target up or down using a commanded climb rate in cm/s
    ///     using the default position control kinematic path.
    ///     ignore_descent_limit turns off output saturation handling to aid in landing detection. ignore_descent_limit should be true unless landing.
    void land_at_climb_rate_cm(float vel, bool ignore_descent_limit);

    /// input_pos_vel_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input position velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The function alters the pos and vel to be the kinematic path based on accel
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    void input_pos_vel_accel_z(float &pos, float &vel, float accel, bool limit_output = true);

    /// set_alt_target_with_slew - adjusts target up or down using a commanded altitude in cm
    ///     using the default position control kinematic path.
    void set_alt_target_with_slew(float pos);

    /// update_pos_offset_z - updates the vertical offsets used by terrain following
    void update_pos_offset_z(float pos_offset);

    // is_active_z - returns true if the z position controller has been run in the previous 5 loop times
    bool is_active_z() const;

    /// update_z_controller - runs the vertical position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_z_controller();



    ///
    /// Accessors
    ///

    /// set commanded position (cm), velocity (cm/s) and acceleration (cm/s/s) inputs when the path is created externally.
    void set_pos_vel_accel(const Vector3p& pos, const Vector3f& vel, const Vector3f& accel);
    void set_pos_vel_accel_xy(const Vector2p& pos, const Vector2f& vel, const Vector2f& accel);


    /// Position

    /// set_pos_target_xy_cm - sets the position target in NEU cm from home
    void set_pos_target_xy_cm(float pos_x, float pos_y) { _pos_target.x = pos_x; _pos_target.y = pos_y; }

    /// get_pos_target_cm - returns the position target in NEU cm from home
    const Vector3p& get_pos_target_cm() const { return _pos_target; }

    /// set_pos_target_z_cm - set altitude target in cm above home
    void set_pos_target_z_cm(float pos_target) { _pos_target.z = pos_target; }

    /// get_pos_target_z_cm - get target altitude (in cm above home)
    float get_pos_target_z_cm() const { return _pos_target.z; }

    /// get_stopping_point_xy_cm - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration
    void get_stopping_point_xy_cm(Vector2p &stopping_point) const;

    /// get_stopping_point_z_cm - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration
    void get_stopping_point_z_cm(postype_t &stopping_point) const;

    /// get_pos_error_cm - get position error vector between the current and target position
    const Vector3f get_pos_error_cm() const { return (_pos_target - _inav.get_position().topostype()).tofloat(); }

    /// get_pos_error_xy_cm - get the length of the position error vector in the xy plane
    float get_pos_error_xy_cm() const { return norm(_pos_target.x - _inav.get_position().x, _pos_target.y - _inav.get_position().y); }

    /// get_pos_error_z_cm - returns altitude error in cm
    float get_pos_error_z_cm() const { return (_pos_target.z - _inav.get_position().z); }


    /// Velocity

    /// set_vel_desired_cms - sets desired velocity in NEU cm/s
    void set_vel_desired_cms(const Vector3f &des_vel) { _vel_desired = des_vel; }

    /// set_vel_desired_xy_cms - sets horizontal desired velocity in NEU cm/s
    void set_vel_desired_xy_cms(const Vector2f &vel) {_vel_desired.xy() = vel; }

    /// get_vel_desired_cms - returns desired velocity (i.e. feed forward) in cm/s in NEU
    const Vector3f& get_vel_desired_cms() { return _vel_desired; }

    // get_vel_target_cms - returns the target velocity in NEU cm/s
    const Vector3f& get_vel_target_cms() const { return _vel_target; }

    /// set_vel_desired_z_cms - sets desired velocity in cm/s in z axis
    void set_vel_desired_z_cms(float vel_z_cms) {_vel_desired.z = vel_z_cms;}

    /// get_vel_target_z_cms - returns current vertical speed in cm/s
    float get_vel_target_z_cms() const { return _vel_target.z; }


    /// Acceleration

    // set_accel_desired_xy_cmss set desired acceleration in cm/s in xy axis
    void set_accel_desired_xy_cmss(const Vector2f &accel_cms) { _accel_desired.xy() = accel_cms; }

    // get_accel_target_cmss - returns the target acceleration in NEU cm/s/s
    const Vector3f& get_accel_target_cmss() const { return _accel_target; }


    /// Offset

    /// set_pos_offset_target_z_cm - set altitude offset target in cm above home
    void set_pos_offset_target_z_cm(float pos_offset_target_z) { _pos_offset_target_z = pos_offset_target_z; }

    /// set_pos_offset_z_cm - set altitude offset in cm above home
    void set_pos_offset_z_cm(float pos_offset_z) { _pos_offset_z = pos_offset_z; }

    /// get_pos_offset_z_cm - returns altitude offset in cm above home
    float get_pos_offset_z_cm() const { return _pos_offset_z; }

    /// get_vel_offset_z_cm - returns current vertical offset speed in cm/s
    float get_vel_offset_z_cms() const { return _vel_offset_z; }

    /// get_accel_offset_z_cm - returns current vertical offset acceleration in cm/s/s
    float get_accel_offset_z_cmss() const { return _accel_offset_z; }


    /// Outputs

    /// get desired roll and pitch to be passed to the attitude controller
    float get_roll_cd() const { return _roll_target; }
    float get_pitch_cd() const { return _pitch_target; }

    /// get desired yaw to be passed to the attitude controller
    float get_yaw_cd() const { return _yaw_target; }

    /// get desired yaw rate to be passed to the attitude controller
    float get_yaw_rate_cds() const { return _yaw_rate_target; }

    /// get desired roll and pitch to be passed to the attitude controller
    Vector3f get_thrust_vector() const;

    /// get_bearing_to_target_cd - get bearing to target position in centi-degrees
    int32_t get_bearing_to_target_cd() const;

    /// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request
    float get_lean_angle_max_cd() const;


    /// Other

    /// get pid controllers
    AC_P_2D& get_pos_xy_p() { return _p_pos_xy; }
    AC_P_1D& get_pos_z_p() { return _p_pos_z; }
    AC_PID_2D& get_vel_xy_pid() { return _pid_vel_xy; }
    AC_PID_Basic& get_vel_z_pid() { return _pid_vel_z; }
    AC_PID& get_accel_z_pid() { return _pid_accel_z; }

    /// set_limit_accel_xy - mark that accel has been limited
    ///     this prevents integrator buildup
    void set_externally_limited_xy() { _limit_vector.x = _accel_target.x; _limit_vector.y = _accel_target.y; }

    // overrides the velocity process variable for one timestep
    void override_vehicle_velocity_xy(const Vector2f& vel_xy) { _vehicle_horiz_vel = vel_xy; _flags.vehicle_horiz_vel_override = true; }

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    Vector3f lean_angles_to_accel(const Vector3f& att_target_euler) const;

    // write PSC and/or PSCZ logs
    void write_log();

    // provide feedback on whether arming would be a good idea right now:
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    // enable or disable high vibration compensation
    void set_vibe_comp(bool on_off) { _vibe_comp_enabled = on_off; }

    /// get_vel_z_error_ratio - returns the proportion of error relative to the maximum request
    float get_vel_z_control_ratio() const { return constrain_float(_vel_z_control_ratio, 0.0f, 1.0f); }

    /// crosstrack_error - returns horizontal error to the closest point to the current track
    float crosstrack_error() const;

    /// standby_xyz_reset - resets I terms and removes position error
    ///     This function will let Loiter and Alt Hold continue to operate
    ///     in the event that the flight controller is in control of the
    ///     aircraft when in standby.
    void standby_xyz_reset();

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // general purpose flags
    struct poscontrol_flags {
            uint16_t vehicle_horiz_vel_override : 1; // 1 if we should use _vehicle_horiz_vel as our velocity process variable for one timestep
    } _flags;

    // limit flags structure
    struct poscontrol_limit_flags {
        bool pos_xy;        // true if we have hit a horizontal position limit
        bool pos_up;        // true if we have hit a vertical position limit while going up
        bool pos_down;      // true if we have hit a vertical position limit while going down
    } _limit;

    /// init_xy - initialise the position controller to the current position, velocity and acceleration.
    ///     This function is private and contains all the shared xy axis initialisation functions
    void init_xy();

    /// init_z - initialise the position controller to the current position, velocity and acceleration.
    ///     This function is private and contains all the shared z axis initialisation functions
    void init_z();

    // get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain)
    float get_throttle_with_vibration_override();

    // get earth-frame Z-axis acceleration with gravity removed in cm/s/s with +ve being up
    float get_z_accel_cmss() const { return -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f; }

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const;

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void lean_angles_to_accel_xy(float& accel_x_cmss, float& accel_y_cmss) const;

    // calculate_yaw_and_rate_yaw - calculate the vehicle yaw and rate of yaw.
    bool calculate_yaw_and_rate_yaw();

    /// initialise and check for ekf position resets
    void init_ekf_xy_reset();
    void handle_ekf_xy_reset();
    void init_ekf_z_reset();
    void handle_ekf_z_reset();

    // references to inertial nav and ahrs libraries
    AP_AHRS_View&           _ahrs;
    const AP_InertialNav&   _inav;
    const AP_Motors&        _motors;
    AC_AttitudeControl&     _attitude_control;

    // parameters
    AP_Float        _lean_angle_max;    // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AP_Float        _shaping_jerk_xy;   // Jerk limit of the xy kinematic path generation in m/s^3 used to determine how quickly the aircraft varies the acceleration target
    AP_Float        _shaping_jerk_z;    // Jerk limit of the z kinematic path generation in m/s^3 used to determine how quickly the aircraft varies the acceleration target
    AC_P_2D         _p_pos_xy;          // XY axis position controller to convert distance error to desired velocity
    AC_P_1D         _p_pos_z;           // Z axis position controller to convert altitude error to desired climb rate
    AC_PID_2D       _pid_vel_xy;        // XY axis velocity controller to convert velocity error to desired acceleration
    AC_PID_Basic    _pid_vel_z;         // Z axis velocity controller to convert climb rate error to desired acceleration
    AC_PID          _pid_accel_z;       // Z axis acceleration controller to convert desired acceleration to throttle output

    // internal variables
    float       _dt;                    // time difference (in seconds) between calls from the main program
    uint64_t    _last_update_xy_us;     // system time (in microseconds) since last update_xy_controller call
    uint64_t    _last_update_z_us;      // system time (in microseconds) since last update_z_controller call
    float       _vel_max_xy_cms;        // max horizontal speed in cm/s used for kinematic shaping
    float       _vel_max_up_cms;        // max climb rate in cm/s used for kinematic shaping
    float       _vel_max_down_cms;      // max descent rate in cm/s used for kinematic shaping
    float       _accel_max_xy_cmss;     // max horizontal acceleration in cm/s/s used for kinematic shaping
    float       _accel_max_z_cmss;      // max vertical acceleration in cm/s/s used for kinematic shaping
    float       _jerk_max_xy_cmsss;       // Jerk limit of the xy kinematic path generation in cm/s^3 used to determine how quickly the aircraft varies the acceleration target
    float       _jerk_max_z_cmsss;        // Jerk limit of the z kinematic path generation in cm/s^3 used to determine how quickly the aircraft varies the acceleration target
    float       _vel_z_control_ratio = 2.0f;    // confidence that we have control in the vertical axis

    // output from controller
    float       _roll_target;           // desired roll angle in centi-degrees calculated by position controller
    float       _pitch_target;          // desired roll pitch in centi-degrees calculated by position controller
    float       _yaw_target;            // desired yaw in centi-degrees calculated by position controller
    float       _yaw_rate_target;       // desired yaw rate in centi-degrees per second calculated by position controller

    // position controller internal variables
    Vector3p    _pos_target;            // target location in NEU cm from home
    Vector3f    _vel_desired;           // desired velocity in NEU cm/s
    Vector3f    _vel_target;            // velocity target in NEU cm/s calculated by pos_to_rate step
    Vector3f    _accel_desired;         // desired acceleration in NEU cm/s/s (feed forward)
    Vector3f    _accel_target;          // acceleration target in NEU cm/s/s
    Vector3f    _limit_vector;          // the direction that the position controller is limited, zero when not limited
    Vector2f    _vehicle_horiz_vel;     // velocity to use if _flags.vehicle_horiz_vel_override is set
    float       _pos_offset_target_z;   // vertical position offset target in NEU cm from home
    float       _pos_offset_z;          // vertical position offset in NEU cm from home
    float       _vel_offset_z;          // vertical velocity offset in NEU cm/s calculated by pos_to_rate step
    float       _accel_offset_z;        // vertical acceleration offset in NEU cm/s/s

    // ekf reset handling
    uint32_t    _ekf_xy_reset_ms;       // system time of last recorded ekf xy position reset
    uint32_t    _ekf_z_reset_ms;        // system time of last recorded ekf altitude reset

    // high vibration handling
    bool        _vibe_comp_enabled;     // true when high vibration compensation is on

    // return true if on a real vehicle or SITL with lock-step scheduling
    bool has_good_timing(void) const;
};
