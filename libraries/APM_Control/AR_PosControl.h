#pragma once

#include <AP_Common/AP_Common.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <AC_PID/AC_P_2D.h>            // P library (2-axis)
#include <AC_PID/AC_PID_2D.h>          // PID library (2-axis)

class AR_PosControl {
public:

    // constructor
    AR_PosControl(AR_AttitudeControl& atc);

    // do not allow copying
    CLASS_NO_COPY(AR_PosControl);

    static AR_PosControl *get_singleton() { return _singleton; }

    // update navigation
    void update(float dt);

    // true if update has been called recently
    bool is_active() const;

    // set speed, acceleration and jerk limits
    void set_limits(float speed_max, float accel_max, float lat_accel_max, float jerk_max);

    // setter to allow vehicle code to provide turn related param values to this library (should be updated regularly)
    void set_turn_params(float turn_radius, bool pivot_possible);

    // set reversed
    void set_reversed(bool reversed) { _reversed = reversed; }

    // accessor for _reversed
    bool get_reversed() { return _reversed; }

    // get limits
    float get_speed_max() const { return _speed_max; }
    float get_accel_max() const { return _accel_max; }
    float get_lat_accel_max() const { return _lat_accel_max; }
    float get_jerk_max() const { return _jerk_max; }

    // initialise the position controller to the current position, velocity, acceleration and attitude
    // this should be called before the input shaping methods are used
    // return true on success, false if targets cannot be initialised
    bool init();

    // adjust position, velocity and acceleration targets smoothly using input shaping
    // pos is the target position as an offset from the EKF origin (in meters)
    // vel is the target velocity in m/s. accel is the target acceleration in m/s/s
    // dt should be the update rate in seconds
    // init should be called once before starting to use these methods
    void input_pos_target(const Vector2p &pos, float dt);
    void input_pos_vel_target(const Vector2p &pos, const Vector2f &vel, float dt);
    void input_pos_vel_accel_target(const Vector2p &pos, const Vector2f &vel, const Vector2f &accel, float dt);

    // set target position, desired velocity and acceleration.  These should be from an externally created path and are not "input shaped"
    void set_pos_vel_accel_target(const Vector2p &pos, const Vector2f &vel, const Vector2f &accel);

    // get outputs for forward-back speed (in m/s), lateral speed (in m/s) and turn rate (in rad/sec)
    float get_desired_speed() const { return _desired_speed; }
    float get_desired_turn_rate_rads() const { return _desired_turn_rate_rads; }
    float get_desired_lat_accel() const { return _desired_lat_accel; }

    // get position target
    const Vector2p& get_pos_target() const { return _pos_target; }

    // returns desired velocity vector (i.e. feed forward) in m/s in NE frame
    Vector2f get_desired_velocity() const;

    // return desired acceleration vector in m/s in NE frame
    Vector2f get_desired_accel() const;

    /// get position error as a vector from the current position to the target position
    Vector2p get_pos_error() const;

    // get pid controllers
    AC_P_2D& get_pos_p() { return _p_pos; }
    AC_PID_2D& get_vel_pid() { return _pid_vel; }

    // write PSC logs
    void write_log();

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AR_PosControl *_singleton;

    // initialise and check for ekf position resets
    void init_ekf_xy_reset();
    void handle_ekf_xy_reset();

    // references
    AR_AttitudeControl &_atc;       // rover attitude control library

    // parameters
    AC_P_2D   _p_pos;               // position P controller to convert distance error to desired velocity
    AC_PID_2D _pid_vel;             // velocity PID controller to convert velocity error to desired acceleration

    // limits
    float _speed_max;               // maximum forward speed in m/s
    float _accel_max;               // maximum forward/back acceleration in m/s/s
    float _lat_accel_max;           // lateral acceleration maximum in m/s/s
    float _jerk_max;                // maximum jerk in m/s/s/s (used for both forward and lateral input shaping)
    float _turn_radius;             // vehicle turn radius in meters

    // position and velocity targets
    Vector2p _pos_target;           // position target as an offset (in meters) from the EKF origin
    Vector2f _vel_desired;          // desired velocity in m/s in NE frame.  This is the "feed forward" provided by SCurves
    Vector2f _vel_target;           // velocity target in m/s in NE frame
    Vector2f _accel_desired;        // desired accel in m/s/s in NE frame.  This is the "feed forward" provided by SCurves
    Vector2f _accel_target;         // accel target in m/s/s in NE frame
    bool _pos_target_valid;         // true if _pos_target is valid
    bool _vel_desired_valid;        // true if _vel_desired is valid
    bool _accel_desired_valid;      // true if _accel_desired is valid

    // variables for navigation
    uint32_t _last_update_ms;       // system time of last call to update
    bool _reversed;                 // true if vehicle should move in reverse towards target

    // main outputs
    float _desired_speed;           // desired forward_back speed in m/s
    float _desired_turn_rate_rads;  // desired turn-rate in rad/sec (negative is counter clockwise, positive is clockwise)
    float _desired_lat_accel;       // desired lateral acceleration (for reporting only)

    // ekf reset handling
    uint32_t _ekf_xy_reset_ms;      // system time of last recorded ekf xy position reset
};
