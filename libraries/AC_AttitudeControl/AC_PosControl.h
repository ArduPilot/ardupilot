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
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PDNN Controller~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <AC_PID/AC_PDNN_3D.h>  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <AP_InertialNav/AP_InertialNav.h>  // Inertial Navigation library
#include <AP_Scripting/AP_Scripting_config.h>
#include "AC_AttitudeControl.h"     // Attitude control library

#include <AP_Logger/LogStructure.h>

// position controller default definitions               //在这里可以设定各种速度和加速度限制!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

#define POSCONTROL_RELAX_TC                     0.16f   // This is used to decay the I term to 5% in half a second.

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~添加pdnn~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define POSCONTROL_PDNN_XY_P                  5.0f    // horizontal pdnn controller P gain default
#define POSCONTROL_PDNN_XY_D                  5.0f    // horizontal pdnn controller D gain default
#define POSCONTROL_PDNN_Z_P                   5.0f    // veritical pdnn controller P gain default
#define POSCONTROL_PDNN_Z_D                   5.0f    // veritical pdnn controller D gain default
//在头文件中定义pdnn控制器构造函数的初始化默认值，是因为cpp文件中if编译需要工作空间先build copter
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class AC_PosControl
{
public:

    /// Constructor
    AC_PosControl(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                  const class AP_Motors& motors, AC_AttitudeControl& attitude_control);

    // do not allow copying
    CLASS_NO_COPY(AC_PosControl);

    /// set_dt / get_dt - dt is the time since the last time the position controllers were updated
    ///   _dt should be set based on the time of the last IMU read used by these controllers
    ///   the position controller should run updates for active controllers on each loop to ensure normal operation
    void set_dt(float dt) { _dt = dt; }
    float get_dt() const { return _dt; }

    /// get_shaping_jerk_xy_cmsss - gets the jerk limit of the xy kinematic path generation in cm/s/s/s
    float get_shaping_jerk_xy_cmsss() const { return _shaping_jerk_xy * 100.0; }


    ///
    /// 3D position shaper
    ///

    /// input_pos_xyz - calculate a jerk limited path from the current position, velocity and acceleration to an input position.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_xy.
    void input_pos_xyz(const Vector3p& pos, float pos_terrain_target, float terrain_buffer);

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

    /// init_xy_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
    ///     This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
    ///     The starting position can be retrieved by getting the position target using get_pos_target_cm() after calling this function.
    void init_xy_controller_stopping_point();

    // relax_velocity_controller_xy - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
    void relax_velocity_controller_xy();

    /// reduce response for landing
    void soften_for_landing_xy();

    // init_xy_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function is private and contains all the shared xy axis initialisation functions
    void init_xy_controller();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~初始化期望旋转矩阵Rc~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void init_Rc();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~平滑期望X位置函数~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    float pos_desired_x_set_update(float x_final, float max_pos_x, float rate, float frequency);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~平滑期望Y位置函数~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    float pos_desired_y_set_update(float y_final, float max_pos_y, float rate, float frequency);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~平滑期望高度函数~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    float pos_desired_z_set_update(float z_final, float max_alt, float rate, float frequency);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~扰动函数~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    float disturb(float frequency);
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~is_active_Rc~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    bool is_active_Rc() const;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

    /// stop_pos_xy_stabilisation - sets the target to the current position to remove any position corrections from the system
    void stop_pos_xy_stabilisation();

    /// stop_vel_xy_stabilisation - sets the target to the current position and velocity to the current velocity to remove any position and velocity corrections from the system
    void stop_vel_xy_stabilisation();

    /// update_xy_controller - runs the horizontal position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_xy_controller();
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~期望旋转矩阵Rc更新函数~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void update_Rc();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

    // init_z_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function is private and contains all the shared z axis initialisation functions
    void init_z_controller();

    /// input_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_z.
    virtual void input_accel_z(float accel);

    /// input_vel_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_z.
    ///     The function alters the vel to be the kinematic path based on accel
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    virtual void input_vel_accel_z(float &vel, float accel, bool limit_output = true);

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

    /// get_pos_target_cm - returns the position target, frame NEU in cm relative to the EKF origin
    const Vector3p& get_pos_target_cm() const { return _pos_target; }  

    /// set_pos_desired_xy_cm - sets the position target, frame NEU in cm relative to the EKF origin //AC_WPNav模块通过这个set访问器，设置期望位置
    void set_pos_desired_xy_cm(const Vector2f& pos) { _pos_desired.xy() = pos.topostype(); }

    /// get_pos_desired_cm - returns the position desired, frame NEU in cm relative to the EKF origin
    const Vector3p& get_pos_desired_cm() const { return _pos_desired; } //moed_guided.cpp通过这里的getter，传输期望位置信息到poscontrol

    /// get_pos_target_z_cm - get target altitude (in cm above the EKF origin)
    float get_pos_target_z_cm() const { return _pos_target.z; }

    /// set_pos_desired_z_cm - set altitude target in cm above the EKF origin
    void set_pos_desired_z_cm(float pos_z) { _pos_desired.z = pos_z; }

    /// get_pos_desired_z_cm - get target altitude (in cm above the EKF origin)
    float get_pos_desired_z_cm() const { return _pos_desired.z; }


    /// Stopping Point

    /// get_stopping_point_xy_cm - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration
    void get_stopping_point_xy_cm(Vector2p &stopping_point) const;

    /// get_stopping_point_z_cm - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration
    void get_stopping_point_z_cm(postype_t &stopping_point) const;


    /// Position Error //这里的get访问通过调用pid的访问器get_error来实现“串联访问”

    /// get_pos_error_cm - get position error vector between the current and target position
    const Vector3f get_pos_error_cm() const { return Vector3f(_p_pos_xy.get_error().x, _p_pos_xy.get_error().y, _p_pos_z.get_error()); }

    /// get_pos_error_xy_cm - get the length of the position error vector in the xy plane
    float get_pos_error_xy_cm() const { return _p_pos_xy.get_error().length(); }

    /// get_pos_error_z_cm - returns altitude error in cm
    float get_pos_error_z_cm() const { return _p_pos_z.get_error(); }


    /// Velocity

    /// set_vel_desired_cms - sets desired velocity in NEU cm/s
    void set_vel_desired_cms(const Vector3f &des_vel) { _vel_desired = des_vel; }

    /// set_vel_desired_xy_cms - sets horizontal desired velocity in NEU cm/s
    void set_vel_desired_xy_cms(const Vector2f &vel) {_vel_desired.xy() = vel; }

    /// get_vel_desired_cms - returns desired velocity in cm/s in NEU
    const Vector3f& get_vel_desired_cms() { return _vel_desired; }

    // get_vel_target_cms - returns the target velocity in NEU cm/s
    const Vector3f& get_vel_target_cms() const { return _vel_target; }

    /// set_vel_desired_z_cms - sets desired velocity in cm/s in z axis
    void set_vel_desired_z_cms(float vel_z_cms) {_vel_desired.z = vel_z_cms;}

    /// get_vel_target_z_cms - returns target vertical speed in cm/s
    float get_vel_target_z_cms() const { return _vel_target.z; }


    /// Acceleration

    // set_accel_desired_xy_cmss - set desired acceleration in cm/s in xy axis
    void set_accel_desired_xy_cmss(const Vector2f &accel_cms) { _accel_desired.xy() = accel_cms; }

    // get_accel_target_cmss - returns the target acceleration in NEU cm/s/s
    const Vector3f& get_accel_target_cmss() const { return _accel_target; }


    /// Terrain

    // set_pos_terrain_target_cm - set target terrain altitude in cm
    void set_pos_terrain_target_cm(float pos_terrain_target) {_pos_terrain_target = pos_terrain_target;}

    // init_pos_terrain_cm - initialises the current terrain altitude and target altitude to pos_offset_terrain_cm
    void init_pos_terrain_cm(float pos_offset_terrain_cm);

    // get_pos_terrain_cm - returns the current terrain altitude in cm
    float get_pos_terrain_cm() { return _pos_terrain; }


    /// Offset

#if AP_SCRIPTING_ENABLED
    // position, velocity and acceleration offset target (only used by scripting)
    // gets or sets an additional offset to the vehicle's target position, velocity and acceleration
    // units are m, m/s and m/s/s in NED frame
    bool set_posvelaccel_offset(const Vector3f &pos_offset_NED, const Vector3f &vel_offset_NED, const Vector3f &accel_offset_NED);
    bool get_posvelaccel_offset(Vector3f &pos_offset_NED, Vector3f &vel_offset_NED, Vector3f &accel_offset_NED);
#endif

    /// set the horizontal position, velocity and acceleration offset targets in cm, cms and cm/s/s from EKF origin in NE frame
    /// these must be set every 3 seconds (or less) or they will timeout and return to zero
    void set_posvelaccel_offset_target_xy_cm(const Vector2p& pos_offset_target_xy_cm, const Vector2f& vel_offset_target_xy_cms, const Vector2f& accel_offset_target_xy_cmss);
    void set_posvelaccel_offset_target_z_cm(float pos_offset_target_z_cm, float vel_offset_target_z_cms, float accel_offset_target_z_cmss);

    /// get the position, velocity or acceleration offets in cm from EKF origin in NEU frame
    const Vector3p& get_pos_offset_cm() const { return _pos_offset; }
    const Vector3f& get_vel_offset_cms() const { return _vel_offset; }
    const Vector3f& get_accel_offset_cmss() const { return _accel_offset; }

    /// set_pos_offset_z_cm - set altitude offset in cm above the EKF origin
    void set_pos_offset_z_cm(float pos_offset_z) { _pos_offset.z = pos_offset_z; }

    /// get_pos_offset_z_cm - returns altitude offset in cm above the EKF origin
    float get_pos_offset_z_cm() const { return _pos_offset.z; }

    /// get_vel_offset_z_cm - returns current vertical offset speed in cm/s
    float get_vel_offset_z_cms() const { return _vel_offset.z; }

    /// get_accel_offset_z_cm - returns current vertical offset acceleration in cm/s/s
    float get_accel_offset_z_cmss() const { return _accel_offset.z; }

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

    /*
      set_lean_angle_max_cd - set the maximum lean angle. A value of zero means to use the ANGLE_MAX parameter.
      This is reset to zero on init_xy_controller()
    */
    void set_lean_angle_max_cd(float angle_max_cd) { _angle_max_override_cd = angle_max_cd; }
    

    /// Other

    /// get pid controllers 为封装外部提供访问位置控制中pid控制器的实时数据
    AC_P_2D& get_pos_xy_p() { return _p_pos_xy; }
    AC_P_1D& get_pos_z_p() { return _p_pos_z; }
    AC_PID_2D& get_vel_xy_pid() { return _pid_vel_xy; }
    AC_PID_Basic& get_vel_z_pid() { return _pid_vel_z; }
    AC_PID& get_accel_z_pid() { return _pid_accel_z; }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~添加pdnn的getter~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    AC_PDNN_3D& get_pos_pdnn() { return _pdnn_pos; }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
    /// set_limit_accel_xy - mark that accel has been limited
    ///     this prevents integrator buildup
    void set_externally_limited_xy() { _limit_vector.x = _accel_target.x; _limit_vector.y = _accel_target.y; }

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

    // get earth-frame Z-axis acceleration with gravity removed in cm/s/s with +ve being up
    float get_z_accel_cmss() const { return -(_ahrs.get_accel_ef().z + GRAVITY_MSS) * 100.0f; }

    /// returns true when the forward pitch demand is limited by the maximum allowed tilt
    bool get_fwd_pitch_is_limited() const { return _fwd_pitch_is_limited; }
    
    // set disturbance north
    void set_disturb_pos_cm(Vector2f disturb_pos) {_disturb_pos = disturb_pos;}

    // set disturbance north
    void set_disturb_vel_cms(Vector2f disturb_vel) {_disturb_vel = disturb_vel;}

    static const struct AP_Param::GroupInfo var_info[];

    static void Write_PSCN(float pos_desired, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel);
    static void Write_PSCE(float pos_desired, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel);
    static void Write_PSCD(float pos_desired, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel);
    static void Write_PSON(float pos_target_offset_cm, float pos_offset_cm, float vel_target_offset_cms, float vel_offset_cms, float accel_target_offset_cmss, float accel_offset_cmss);
    static void Write_PSOE(float pos_target_offset_cm, float pos_offset_cm, float vel_target_offset_cms, float vel_offset_cms, float accel_target_offset_cmss, float accel_offset_cmss);
    static void Write_PSOD(float pos_target_offset_cm, float pos_offset_cm, float vel_target_offset_cms, float vel_offset_cms, float accel_target_offset_cmss, float accel_offset_cmss);
    static void Write_PSOT(float pos_target_offset_cm, float pos_offset_cm, float vel_target_offset_cms, float vel_offset_cms, float accel_target_offset_cmss, float accel_offset_cmss);

    // singleton
    static AC_PosControl *get_singleton(void) { return _singleton; }

protected:

    // get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain)
    float get_throttle_with_vibration_override();

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const;

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void lean_angles_to_accel_xy(float& accel_x_cmss, float& accel_y_cmss) const;

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
    /// this moves the estimated terrain position _pos_terrain towards the target _pos_terrain_target
    void update_terrain();


    /// Offsets

    /// init_offsets - set the position, velocity and acceleration offsets in cm, cms and cm/s/s from EKF origin in NE frame
    /// this is used to initiate the offsets when initialise the position controller or do an offset reset
    /// note that this sets the actual offsets, not the offset targets
    void init_offsets_xy();
    void init_offsets_z();

    /// update_offsets - update the position and velocity offsets
    /// this moves the offsets (e.g _pos_offset, _vel_offset, _accel_offset) towards the targets (e.g. _pos_offset_target or _vel_offset_target)
    void update_offsets_xy();
    void update_offsets_z();

    /// initialise and check for ekf position resets
    void init_ekf_xy_reset();
    void handle_ekf_xy_reset();
    void init_ekf_z_reset();
    void handle_ekf_z_reset();

    // references to inertial nav and ahrs libraries
    AP_AHRS_View&     _ahrs;
    const AP_InertialNav&   _inav;
    const class AP_Motors&  _motors;
    AC_AttitudeControl&     _attitude_control;

    // parameters  //用各种PID类创建位置控制pid对象
    AP_Float        _lean_angle_max;    // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AP_Float        _shaping_jerk_xy;   // Jerk limit of the xy kinematic path generation in m/s^3 used to determine how quickly the aircraft varies the acceleration target
    AP_Float        _shaping_jerk_z;    // Jerk limit of the z kinematic path generation in m/s^3 used to determine how quickly the aircraft varies the acceleration target
    AC_P_2D         _p_pos_xy;          // XY axis position controller to convert distance error to desired velocity
    AC_P_1D         _p_pos_z;           // Z axis position controller to convert altitude error to desired climb rate
    AC_PID_2D       _pid_vel_xy;        // XY axis velocity controller to convert velocity error to desired acceleration
    AC_PID_Basic    _pid_vel_z;         // Z axis velocity controller to convert climb rate error to desired acceleration
    AC_PID          _pid_accel_z;       // Z axis acceleration controller to convert desired acceleration to throttle output
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~创建PDNN控制器对象~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    AC_PDNN_3D      _pdnn_pos;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // internal variables
    float       _dt;                    // time difference (in seconds) since the last loop time
    uint32_t    _last_update_xy_ticks;  // ticks of last last update_xy_controller call
    uint32_t    _last_update_Rc_ticks;  //新建用于记录Rc_update的时间戳
    uint32_t    _last_update_z_ticks;   // ticks of last update_z_controller call
    float       _vel_max_xy_cms;        // max horizontal speed in cm/s used for kinematic shaping
    float       _vel_max_up_cms;        // max climb rate in cm/s used for kinematic shaping
    float       _vel_max_down_cms;      // max descent rate in cm/s used for kinematic shaping
    float       _accel_max_xy_cmss;     // max horizontal acceleration in cm/s/s used for kinematic shaping
    float       _accel_max_z_cmss;      // max vertical acceleration in cm/s/s used for kinematic shaping
    float       _jerk_max_xy_cmsss;       // Jerk limit of the xy kinematic path generation in cm/s^3 used to determine how quickly the aircraft varies the acceleration target
    float       _jerk_max_z_cmsss;        // Jerk limit of the z kinematic path generation in cm/s^3 used to determine how quickly the aircraft varies the acceleration target
    float       _vel_z_control_ratio = 2.0f;    // confidence that we have control in the vertical axis
    Vector2f    _disturb_pos;           // position disturbance generated by system ID mode
    Vector2f    _disturb_vel;           // velocity disturbance generated by system ID mode

    // output from controller
    float       _roll_target;           // desired roll angle in centi-degrees calculated by position controller
    float       _pitch_target;          // desired roll pitch in centi-degrees calculated by position controller
    float       _yaw_target;            // desired yaw in centi-degrees calculated by position controller
    float       _yaw_rate_target;       // desired yaw rate in centi-degrees per second calculated by position controller

    // position controller internal variables
    Vector3p    _pos_desired;           // desired location, frame NEU in cm relative to the EKF origin.  This is equal to the _pos_target minus offsets
    Vector3p    _pos_target;            // target location, frame NEU in cm relative to the EKF origin.  This is equal to the _pos_desired plus offsets
    Vector3f    _vel_desired;           // desired velocity in NEU cm/s
    Vector3f    _vel_target;            // velocity target in NEU cm/s calculated by pos_to_rate step
    Vector3f    _accel_desired;         // desired acceleration in NEU cm/s/s (feed forward)
    Vector3f    _accel_target;          // acceleration target in NEU cm/s/s
    Vector3f    _limit_vector;          // the direction that the position controller is limited, zero when not limited

    bool        _fwd_pitch_is_limited;     // true when the forward pitch demand is being limited to meet acceleration limits

    // terrain handling variables
    float    _pos_terrain_target;       // position terrain target in cm relative to the EKF origin in NEU frame
    float    _pos_terrain;              // position terrain in cm from the EKF origin in NEU frame.  this terrain moves towards _pos_terrain_target
    float    _vel_terrain;              // velocity terrain in NEU cm/s calculated by pos_to_rate step.  this terrain moves towards _vel_terrain_target
    float    _accel_terrain;            // acceleration terrain in NEU cm/s/s

    // offset handling variables
    Vector3p    _pos_offset_target;     // position offset target in cm relative to the EKF origin in NEU frame
    Vector3p    _pos_offset;            // position offset in cm from the EKF origin in NEU frame.  this offset moves towards _pos_offset_target
    Vector3f    _vel_offset_target;     // velocity offset target in cm/s in NEU frame
    Vector3f    _vel_offset;            // velocity offset in NEU cm/s calculated by pos_to_rate step.  this offset moves towards _vel_offset_target
    Vector3f    _accel_offset_target;   // acceleration offset target in cm/s/s in NEU frame
    Vector3f    _accel_offset;          // acceleration offset in NEU cm/s/s
    uint32_t    _posvelaccel_offset_target_xy_ms;   // system time that pos, vel, accel targets were set (used to implement timeouts)
    uint32_t    _posvelaccel_offset_target_z_ms;    // system time that pos, vel, accel targets were set (used to implement timeouts)

    // ekf reset handling
    uint32_t    _ekf_xy_reset_ms;       // system time of last recorded ekf xy position reset
    uint32_t    _ekf_z_reset_ms;        // system time of last recorded ekf altitude reset

    // high vibration handling
    bool        _vibe_comp_enabled;     // true when high vibration compensation is on

    // angle max override, if zero then use ANGLE_MAX parameter
    float       _angle_max_override_cd;

    // return true if on a real vehicle or SITL with lock-step scheduling
    bool has_good_timing(void) const;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~声明内部成员变量~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Vector3f    _U_x;     //PDNN位移控制器输出
    Vector3f    _b_1d;    //desired b1_axis
    Vector3f    _b_1c;
    Vector3f    _b_2c;
    Vector3f    _b_3c;
    Matrix3f    _Rc;      //解算后的期望旋转矩阵
    Matrix3f    _R_body_to_ned_meas; //旋转矩阵测量值 body to NED
    Matrix3f    _R_body_to_neu_meas; //旋转矩阵测量值 body to NEU
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

private:
    // convenience method for writing out the identical PSCE, PSCN, PSCD - and
    // to save bytes
    static void Write_PSCx(LogMessages ID, float pos_desired, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel);

    // a convenience function for writing out the position controller offsets
    static void Write_PSOx(LogMessages id, float pos_target_offset_cm, float pos_offset_cm,
                           float vel_target_offset_cms, float vel_offset_cms,
                           float accel_target_offset_cmss, float accel_offset_cmss);

    // singleton
    static AC_PosControl *_singleton;
};
 ///~~~~~~~~~~~~~~~~~~~~~~~~~~~DIY New DDS Topic output get_Wrench~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Wrench struct

struct DIYWrench
{
Vector3f force; // 力的三维向量
Vector3f torque; // 力矩的三维向量
// 默认构造函数，初始化为 0
DIYWrench() : force(), torque() {}
// 参数化构造函数
DIYWrench(const Vector3f& f, const Vector3f& t) : force(f), torque(t) {}
};

extern DIYWrench get_DIYwrench(float test_msg_1, float test_msg_2);
extern DIYWrench current_DIYwrench;
extern DIYWrench get_current_DIYwrench();
///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~DIY end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
