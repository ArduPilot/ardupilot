#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P.h>               // P library
#include <AC_PID/AC_PID.h>             // PID library
#include <AC_PID/AC_PI_2D.h>           // PI library (2-axis)
#include <AC_PID/AC_PID_2D.h>          // PID library (2-axis)
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include "AC_AttitudeControl.h" // Attitude control library
#include <AP_Motors/AP_Motors.h>          // motors library
#include <AP_Vehicle/AP_Vehicle.h>         // common vehicle parameters


// position controller default definitions
#define POSCONTROL_ACCELERATION_MIN             50.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
#define POSCONTROL_ACCEL_XY                     100.0f  // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
#define POSCONTROL_ACCEL_XY_MAX                 980.0f  // max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
#define POSCONTROL_STOPPING_DIST_UP_MAX         300.0f  // max stopping distance (in cm) vertically while climbing
#define POSCONTROL_STOPPING_DIST_DOWN_MAX       200.0f  // max stopping distance (in cm) vertically while descending

#define POSCONTROL_SPEED                        500.0f  // default horizontal speed in cm/s
#define POSCONTROL_SPEED_DOWN                  -150.0f  // default descent rate in cm/s
#define POSCONTROL_SPEED_UP                     250.0f  // default climb rate in cm/s

#define POSCONTROL_ACCEL_Z                      250.0f  // default vertical acceleration in cm/s/s.

#define POSCONTROL_LEASH_LENGTH_MIN             100.0f  // minimum leash lengths in cm

#define POSCONTROL_DT_50HZ                      0.02f   // time difference in seconds for 50hz update rate
#define POSCONTROL_DT_400HZ                     0.0025f // time difference in seconds for 400hz update rate

#define POSCONTROL_ACTIVE_TIMEOUT_MS            200     // position controller is considered active if it has been called within the past 0.2 seconds

#define POSCONTROL_VEL_ERROR_CUTOFF_FREQ        4.0f    // low-pass filter on velocity error (unit: hz)
#define POSCONTROL_THROTTLE_CUTOFF_FREQ         2.0f    // low-pass filter on accel error (unit: hz)
#define POSCONTROL_ACCEL_FILTER_HZ              2.0f    // low-pass filter on acceleration (unit: hz)
#define POSCONTROL_JERK_RATIO                   1.0f    // Defines the time it takes to reach the requested acceleration

#define POSCONTROL_OVERSPEED_GAIN_Z             2.0f    // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range

class AC_PosControl
{
public:

    /// Constructor
    AC_PosControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                  const AP_Motors& motors, AC_AttitudeControl& attitude_control);

    ///
    /// initialisation functions
    ///

    /// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    ///     updates z axis accel controller's D term filter
    void set_dt(float delta_sec);
    float get_dt() const { return _dt; }

    ///
    /// z position controller
    ///

    /// set_speed_z - sets maximum climb and descent rates
    ///     speed_down can be positive or negative but will always be interpreted as a descent speed
    ///     leash length will be recalculated the next time update_z_controller() is called
    void set_speed_z(float speed_down, float speed_up);

    /// get_speed_up - accessor for current up speed in cm/s
    float get_speed_up() const { return _speed_up_cms; }

    /// get_speed_down - accessors for current down speed in cm/s.  Will be a negative number
    float get_speed_down() const { return _speed_down_cms; }

    /// get_vel_target_z - returns current vertical speed in cm/s
    float get_vel_target_z() const { return _vel_target.z; }

    /// set_accel_z - set vertical acceleration in cm/s/s
    ///     leash length will be recalculated the next time update_z_controller() is called
    void set_accel_z(float accel_cmss);

    /// get_accel_z - returns current vertical acceleration in cm/s/s
    float get_accel_z() const { return _accel_z_cms; }

    /// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
    ///     called by update_z_controller if z-axis speed or accelerations are changed
    void calc_leash_length_z();

    /// set_alt_target - set altitude target in cm above home
    void set_alt_target(float alt_cm) { _pos_target.z = alt_cm; }

    /// set_alt_target_with_slew - adjusts target towards a final altitude target
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    void set_alt_target_with_slew(float alt_cm, float dt);

    /// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    ///     set force_descend to true during landing to allow target to move low enough to slow the motors
    virtual void set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend);

    /// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    ///     set force_descend to true during landing to allow target to move low enough to slow the motors
    virtual void set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend);

    /// add_takeoff_climb_rate - adjusts alt target up or down using a climb rate in cm/s
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     almost no checks are performed on the input
    void add_takeoff_climb_rate(float climb_rate_cms, float dt);

    /// set_alt_target_to_current_alt - set altitude target to current altitude
    void set_alt_target_to_current_alt() { _pos_target.z = _inav.get_altitude(); }

    /// shift altitude target (positive means move altitude up)
    void shift_alt_target(float z_cm);

    /// relax_alt_hold_controllers - set all desired and targets to measured
    void relax_alt_hold_controllers(float throttle_setting);

    /// get_alt_target, get_desired_alt - get desired altitude (in cm above home) from loiter or wp controller which should be fed into throttle controller
    /// To-Do: remove one of the two functions below
    float get_alt_target() const { return _pos_target.z; }

    /// get_alt_error - returns altitude error in cm
    float get_alt_error() const;
    
    // returns horizontal error in cm
    float get_horizontal_error() const;

    /// set_target_to_stopping_point_z - sets altitude target to reasonable stopping altitude in cm above home
    void set_target_to_stopping_point_z();

    /// get_stopping_point_z - calculates stopping point based on current position, velocity, vehicle acceleration
    void get_stopping_point_z(Vector3f& stopping_point) const;

    /// init_takeoff - initialises target altitude if we are taking off
    void init_takeoff();

    // is_active - returns true if the z-axis position controller has been run very recently
    bool is_active_z() const;

    /// update_z_controller - fly to altitude in cm above home
    void update_z_controller();

    // get_leash_down_z, get_leash_up_z - returns vertical leash lengths in cm
    float get_leash_down_z() const { return _leash_down_z; }
    float get_leash_up_z() const { return _leash_up_z; }

    ///
    /// xy position controller
    ///

    /// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request
    float get_lean_angle_max_cd() const;

    /// init_xy_controller - initialise the xy controller
    ///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
    ///     should be called once whenever significant changes to the position target are made
    ///     this does not update the xy target
    void init_xy_controller();

    /// set_accel_xy - set horizontal acceleration in cm/s/s
    ///     leash length will be recalculated the next time update_xy_controller() is called
    void set_accel_xy(float accel_cmss);
    float get_accel_xy() const { return _accel_cms; }

    /// set_speed_xy - set horizontal speed maximum in cm/s
    ///     leash length will be recalculated the next time update_xy_controller() is called
    void set_speed_xy(float speed_cms);
    float get_speed_xy() const { return _speed_cms; }

    /// set_limit_accel_xy - mark that accel has been limited
    ///     this prevents integrator buildup
    void set_limit_accel_xy(void) { _limit.accel_xy = true; }
    
    /// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
    ///     should be called whenever the speed, acceleration or position kP is modified
    void calc_leash_length_xy();

    /// set the horizontal leash length
    void set_leash_length_xy(float leash) { _leash = leash; _flags.recalc_leash_xy = false; }

    /// get_pos_target - get target as position vector (from home in cm)
    const Vector3f& get_pos_target() const { return _pos_target; }

    /// set_pos_target in cm from home
    void set_pos_target(const Vector3f& position);

    /// set_xy_target in cm from home
    void set_xy_target(float x, float y);

    /// shift position target target in x, y axis
    void shift_pos_xy_target(float x_cm, float y_cm);

    /// get_desired_velocity - returns xy desired velocity (i.e. feed forward) in cm/s in lat and lon direction
    const Vector3f& get_desired_velocity() { return _vel_desired; }

    /// set_desired_velocity_z - sets desired velocity in cm/s in z axis
    void set_desired_velocity_z(float vel_z_cms) {_vel_desired.z = vel_z_cms;}

    // clear desired velocity feed-forward in z axis
    void clear_desired_velocity_ff_z() { _flags.use_desvel_ff_z = false; }

    // set desired acceleration in cm/s in xy axis
    void set_desired_accel_xy(float accel_lat_cms, float accel_lon_cms) { _accel_desired.x = accel_lat_cms; _accel_desired.y = accel_lon_cms; }

    /// set_desired_velocity_xy - sets desired velocity in cm/s in lat and lon directions
    ///     when update_xy_controller is next called the position target is moved based on the desired velocity and
    ///     the desired velocities are fed forward into the rate_to_accel step
    void set_desired_velocity_xy(float vel_lat_cms, float vel_lon_cms) {_vel_desired.x = vel_lat_cms; _vel_desired.y = vel_lon_cms; }

    /// set_desired_velocity - sets desired velocity in cm/s in all 3 axis
    ///     when update_vel_controller_xyz is next called the position target is moved based on the desired velocity
    void set_desired_velocity(const Vector3f &des_vel) { _vel_desired = des_vel; }

    // overrides the velocity process variable for one timestep
    void override_vehicle_velocity_xy(const Vector2f& vel_xy) { _vehicle_horiz_vel = vel_xy; _flags.vehicle_horiz_vel_override = true; }

    /// freeze_ff_z - used to stop the feed forward being calculated during a known discontinuity
    void freeze_ff_z() { _flags.freeze_ff_z = true; }

    // is_active_xy - returns true if the xy position controller has been run very recently
    bool is_active_xy() const;

    /// update_xy_controller - run the horizontal position controller - should be called at 100hz or higher
    ///     when use_desired_velocity is true the desired velocity (i.e. feed forward) is incorporated at the pos_to_rate step
    void update_xy_controller(float ekfNavVelGainScaler);

    /// set_target_to_stopping_point_xy - sets horizontal target to reasonable stopping position in cm from home
    void set_target_to_stopping_point_xy();

    /// get_stopping_point_xy - calculates stopping point based on current position, velocity, vehicle acceleration
    ///     distance_max allows limiting distance to stopping point
    ///     results placed in stopping_position vector
    ///     set_accel_xy() should be called before this method to set vehicle acceleration
    ///     set_leash_length() should have been called before this method
    void get_stopping_point_xy(Vector3f &stopping_point) const;

    /// get_distance_to_target - get horizontal distance to position target in cm (used for reporting)
    float get_distance_to_target() const;

    /// get_bearing_to_target - get bearing to target position in centi-degrees
    int32_t get_bearing_to_target() const;

    /// xyz velocity controller

    /// init_vel_controller_xyz - initialise the velocity controller - should be called once before the caller attempts to use the controller
    void init_vel_controller_xyz();

    /// update_velocity_controller_xy - run the XY velocity controller - should be called at 100hz or higher
    ///     velocity targets should we set using set_desired_velocity_xy() method
    ///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
    ///     throttle targets will be sent directly to the motors
    void update_vel_controller_xy(float ekfNavVelGainScaler);
    
    /// update_velocity_controller_xyz - run the velocity controller - should be called at 100hz or higher
    ///     velocity targets should we set using set_desired_velocity_xyz() method
    ///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
    ///     throttle targets will be sent directly to the motors
    void update_vel_controller_xyz(float ekfNavVelGainScaler);

    /// get desired roll, pitch which should be fed into stabilize controllers
    float get_roll() const { return _roll_target; }
    float get_pitch() const { return _pitch_target; }

    // get_leash_xy - returns horizontal leash length in cm
    float get_leash_xy() const { return _leash; }

    /// get pid controllers
    AC_P& get_pos_z_p() { return _p_pos_z; }
    AC_P& get_vel_z_p() { return _p_vel_z; }
    AC_PID& get_accel_z_pid() { return _pid_accel_z; }
    AC_P& get_pos_xy_p() { return _p_pos_xy; }
    AC_PID_2D& get_vel_xy_pid() { return _pid_vel_xy; }

    /// accessors for reporting
    const Vector3f& get_vel_target() const { return _vel_target; }
    const Vector3f& get_accel_target() const { return _accel_target; }

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const;

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void lean_angles_to_accel(float& accel_x_cmss, float& accel_y_cmss) const;

    // time_since_last_xy_update - returns time in seconds since the horizontal position controller was last run
    float time_since_last_xy_update() const;

    // write log to dataflash
    void write_log();

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // general purpose flags
    struct poscontrol_flags {
            uint16_t recalc_leash_z     : 1;    // 1 if we should recalculate the z axis leash length
            uint16_t recalc_leash_xy    : 1;    // 1 if we should recalculate the xy axis leash length
            uint16_t reset_desired_vel_to_pos   : 1;    // 1 if we should reset the rate_to_accel_xy step
            uint16_t reset_accel_to_lean_xy     : 1;    // 1 if we should reset the accel to lean angle step
            uint16_t reset_rate_to_accel_z      : 1;    // 1 if we should reset the rate_to_accel_z step
            uint16_t reset_accel_to_throttle    : 1;    // 1 if we should reset the accel_to_throttle step of the z-axis controller
            uint16_t freeze_ff_z        : 1;    // 1 used to freeze velocity to accel feed forward for one iteration
            uint16_t use_desvel_ff_z    : 1;    // 1 to use z-axis desired velocity as feed forward into velocity step
            uint16_t vehicle_horiz_vel_override : 1; // 1 if we should use _vehicle_horiz_vel as our velocity process variable for one timestep
    } _flags;

    // limit flags structure
    struct poscontrol_limit_flags {
        uint8_t pos_up      : 1;    // 1 if we have hit the vertical position leash limit while going up
        uint8_t pos_down    : 1;    // 1 if we have hit the vertical position leash limit while going down
        uint8_t vel_up      : 1;    // 1 if we have hit the vertical velocity limit going up
        uint8_t vel_down    : 1;    // 1 if we have hit the vertical velocity limit going down
        uint8_t accel_xy    : 1;    // 1 if we have hit the horizontal accel limit
    } _limit;

    ///
    /// z controller private methods
    ///

    // run position control for Z axis
    // target altitude should be set with one of these functions
    //          set_alt_target
    //          set_target_to_stopping_point_z
    //          init_takeoff
    void run_z_controller();

    ///
    /// xy controller private methods
    ///

    /// move velocity target using desired acceleration
    void desired_accel_to_vel(float nav_dt);

    /// desired_vel_to_pos - move position target using desired velocities
    void desired_vel_to_pos(float nav_dt);

    /// run horizontal position controller correcting position and velocity
    ///     converts position (_pos_target) to target velocity (_vel_target)
    ///     desired velocity (_vel_desired) is combined into final target velocity
    ///     converts desired velocities in lat/lon directions to accelerations in lat/lon frame
    ///     converts desired accelerations provided in lat/lon frame to roll/pitch angles
    void run_xy_controller(float dt, float ekfNavVelGainScaler);

    /// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
    float calc_leash_length(float speed_cms, float accel_cms, float kP) const;

    /// limit vector to a given length, returns true if vector was limited
    static bool limit_vector_length(float& vector_x, float& vector_y, float max_length);

    /// Proportional controller with piecewise sqrt sections to constrain second derivative
    static Vector3f sqrt_controller(const Vector3f& error, float p, float second_ord_lim);

    /// initialise and check for ekf position resets
    void init_ekf_xy_reset();
    void check_for_ekf_xy_reset();
    void init_ekf_z_reset();
    void check_for_ekf_z_reset();

    // references to inertial nav and ahrs libraries
    const AP_AHRS_View &        _ahrs;
    const AP_InertialNav&       _inav;
    const AP_Motors&            _motors;
    AC_AttitudeControl&         _attitude_control;

    // parameters
    AP_Float    _accel_xy_filt_hz;      // XY acceleration filter cutoff frequency
    AP_Float    _lean_angle_max;        // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AC_P        _p_pos_z;
    AC_P        _p_vel_z;
    AC_PID      _pid_accel_z;
    AC_P        _p_pos_xy;
    AC_PID_2D   _pid_vel_xy;

    // internal variables
    float       _dt;                    // time difference (in seconds) between calls from the main program
    uint32_t    _last_update_xy_ms;     // system time of last update_xy_controller call
    uint32_t    _last_update_z_ms;      // system time of last update_z_controller call
    float       _speed_down_cms;        // max descent rate in cm/s
    float       _speed_up_cms;          // max climb rate in cm/s
    float       _speed_cms;             // max horizontal speed in cm/s
    float       _accel_z_cms;           // max vertical acceleration in cm/s/s
    float       _accel_last_z_cms;      // max vertical acceleration in cm/s/s
    float       _accel_cms;             // max horizontal acceleration in cm/s/s
    float       _leash;                 // horizontal leash length in cm.  target will never be further than this distance from the vehicle
    float       _leash_down_z;          // vertical leash down in cm.  target will never be further than this distance below the vehicle
    float       _leash_up_z;            // vertical leash up in cm.  target will never be further than this distance above the vehicle

    // output from controller
    float       _roll_target;           // desired roll angle in centi-degrees calculated by position controller
    float       _pitch_target;          // desired roll pitch in centi-degrees calculated by position controller

    // position controller internal variables
    Vector3f    _pos_target;            // target location in cm from home
    Vector3f    _pos_error;             // error between desired and actual position in cm
    Vector3f    _vel_desired;           // desired velocity in cm/s
    Vector3f    _vel_target;            // velocity target in cm/s calculated by pos_to_rate step
    Vector3f    _vel_error;             // error between desired and actual acceleration in cm/s
    Vector3f    _vel_last;              // previous iterations velocity in cm/s
    Vector3f    _accel_desired;         // desired acceleration in cm/s/s (feed forward)
    Vector3f    _accel_target;          // acceleration target in cm/s/s
    Vector3f    _accel_error;           // acceleration error in cm/s/s
    Vector2f    _vehicle_horiz_vel;     // velocity to use if _flags.vehicle_horiz_vel_override is set
    float       _distance_to_target;    // distance to position target - for reporting only
    LowPassFilterFloat _vel_error_filter;   // low-pass-filter on z-axis velocity error

    LowPassFilterVector2f _accel_target_filter; // acceleration target filter

    // ekf reset handling
    uint32_t    _ekf_xy_reset_ms;      // system time of last recorded ekf xy position reset
    uint32_t    _ekf_z_reset_ms;       // system time of last recorded ekf altitude reset
};
