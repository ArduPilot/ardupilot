#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

class AR_AttitudeControl {
public:

    // constructor
    AR_AttitudeControl();

    // do not allow copying
    CLASS_NO_COPY(AR_AttitudeControl);

    static AR_AttitudeControl *get_singleton() { return _singleton; }

    //
    // steering controller
    //

    // return a steering servo output given a desired lateral acceleration rate in m/s/s.
    // positive lateral acceleration is to the right.  dt should normally be the main loop rate.
    // return value is normally in range -1.0 to +1.0 but can be higher or lower
    float get_steering_out_lat_accel(float desired_accel, bool motor_limit_left, bool motor_limit_right, float dt);

    // return a steering servo output given a heading in radians
    // set rate_max_rads to a non-zero number to apply a limit on the desired turn rate
    // return value is normally in range -1.0 to +1.0 but can be higher or lower
    float get_steering_out_heading(float heading_rad, float rate_max_rads, bool motor_limit_left, bool motor_limit_right, float dt);

    // return a desired turn-rate given a desired heading in radians
    // normally the results are later passed into get_steering_out_rate
    float get_turn_rate_from_heading(float heading_rad, float rate_max_rads) const;

    // return a steering servo output given a desired yaw rate in radians/sec.
    // positive yaw is to the right
    // return value is normally in range -1.0 to +1.0 but can be higher or lower
    // also sets steering_limit_left and steering_limit_right flags
    float get_steering_out_rate(float desired_rate, bool motor_limit_left, bool motor_limit_right, float dt);

    // get latest desired turn rate in rad/sec recorded during calls to get_steering_out_rate.  For reporting purposes only
    float get_desired_turn_rate() const;

    // get latest desired lateral acceleration in m/s/s recorded during calls to get_steering_out_lat_accel.  For reporting purposes only
    float get_desired_lat_accel() const;

    // get actual lateral acceleration in m/s/s.  returns true on success.  For reporting purposes only
    bool get_lat_accel(float &lat_accel) const;

    // calculate the turn rate in rad/sec given a lateral acceleration (in m/s/s) and speed (in m/s)
    float get_turn_rate_from_lat_accel(float lat_accel, float speed) const;

    // get the lateral acceleration limit (in m/s/s).  Returns at least 0.1G or approximately 1 m/s/s
    float get_turn_lat_accel_max() const { return MAX(_turn_lateral_G_max, 0.1f) * GRAVITY_MSS; }

    // returns true if the steering has been limited which can be caused by the physical steering surface
    // reaching its physical limits (aka motor limits) or acceleration or turn rate limits being applied
    bool steering_limit_left() const { return _steering_limit_left; }
    bool steering_limit_right() const { return _steering_limit_right; }

    //
    // throttle / speed controller
    //

    // set limits used by throttle controller
    //   forward/back acceleration max in m/s/s
    //   forward/back deceleartion max in m/s/s
    void set_throttle_limits(float throttle_accel_max, float throttle_decel_max);

    // return a throttle output from -1 to +1 given a desired speed in m/s (use negative speeds to travel backwards)
    //   desired_speed argument should already have been passed through get_desired_speed_accel_limited function
    //   motor_limit should be true if motors have hit their upper or lower limits
    //   cruise speed should be in m/s, cruise throttle should be a number from -1 to +1
    float get_throttle_out_speed(float desired_speed, bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt);

    // return a throttle output from -1 to +1 to perform a controlled stop.  stopped is set to true once stop has been completed
    float get_throttle_out_stop(bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt, bool &stopped);

    // balancebot pitch to throttle controller
    // returns a throttle output from -1 to +1 given a desired pitch angle (in radians)
    // pitch_max should be the user defined max pitch angle (in radians)
    // motor_limit should be true if the motors have hit their upper or lower limit
    float get_throttle_out_from_pitch(float desired_pitch, float pitch_max, bool motor_limit, float dt);

    // returns true if the pitch angle has been limited to prevent falling over
    // pitch limit protection is implemented within get_throttle_out_from_pitch
    bool pitch_limited() const { return _pitch_limited; }

    // get latest desired pitch in radians for reporting purposes
    float get_desired_pitch() const;

    // Sailboat heel(roll) angle contorller, release sail to keep at maximum heel angle
    float get_sail_out_from_heel(float desired_heel, float dt);

    // low level control accessors for reporting and logging
    AC_P& get_steering_angle_p() { return _steer_angle_p; }
    AC_PID& get_steering_rate_pid() { return _steer_rate_pid; }
    AC_PID& get_pitch_to_throttle_pid() { return _pitch_to_throttle_pid; }
    AC_PID& get_sailboat_heel_pid() { return _sailboat_heel_pid; }
    const AP_PIDInfo& get_throttle_speed_pid_info() const { return _throttle_speed_pid_info; }

    // get the slew rate value for speed and steering for oscillation detection in lua scripts
    void get_srate(float &steering_srate, float &speed_srate);

    // get forward speed in m/s (earth-frame horizontal velocity but only along vehicle x-axis).  returns true on success
    bool get_forward_speed(float &speed) const;

    // get throttle/speed controller maximum acceleration (also used for deceleration)
    float get_accel_max() const { return MAX(_throttle_accel_max, 0.0f); }

    // get throttle/speed controller maximum deceleration
    float get_decel_max() const;

    // check if speed controller active
    bool speed_control_active() const;

    // get latest desired speed recorded during call to get_throttle_out_speed.  For reporting purposes only
    float get_desired_speed() const;

    // get acceleration limited desired speed
    float get_desired_speed_accel_limited(float desired_speed, float dt) const;

    // get minimum stopping distance (in meters) given a speed (in m/s)
    float get_stopping_distance(float speed) const;

    // get speed below which vehicle is considered stopped (in m/s)
    float get_stop_speed() const { return MAX(_stop_speed, 0.0f); }

    // relax I terms of throttle and steering controllers
    void relax_I();

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AR_AttitudeControl *_singleton;

    // parameters
    AC_P     _steer_angle_p;        // steering angle controller
    AC_PID   _steer_rate_pid;       // steering rate controller
    AC_PID   _throttle_speed_pid;   // throttle speed controller
    AC_PID   _pitch_to_throttle_pid;// balancebot pitch controller
    AP_Float _pitch_to_throttle_ff; // balancebot feed forward from current pitch angle
    AP_Float _pitch_limit_tc;       // balancebot pitch limit protection time constant
    AP_Float _pitch_limit_throttle_thresh;  // balancebot pitch limit throttle threshold (in the range 0 to 1.0)

    AP_Float _throttle_accel_max;   // speed/throttle control acceleration (and deceleration) maximum in m/s/s.  0 to disable limits
    AP_Float _throttle_decel_max;    // speed/throttle control deceleration maximum in m/s/s. 0 to use ATC_ACCEL_MAX for deceleration
    AP_Int8  _brake_enable;         // speed control brake enable/disable. if set to 1 a reversed output to the motors to slow the vehicle.
    AP_Float _stop_speed;           // speed control stop speed.  Motor outputs to zero once vehicle speed falls below this value
    AP_Float _steer_accel_max;      // steering angle acceleration max in deg/s/s
    AP_Float _steer_rate_max;       // steering rate control maximum rate in deg/s
    AP_Float _turn_lateral_G_max;   // sterring maximum lateral acceleration limit in 'G'

    // steering control
    uint32_t _steer_lat_accel_last_ms;  // system time of last call to lateral acceleration controller (i.e. get_steering_out_lat_accel)
    uint32_t _steer_turn_last_ms;   // system time of last call to steering rate controller
    float    _desired_lat_accel;    // desired lateral acceleration (in m/s/s) from latest call to get_steering_out_lat_accel (for reporting purposes)
    float    _desired_turn_rate;    // desired turn rate (in radians/sec) either from external caller or from lateral acceleration controller
    bool     _steering_limit_left;  // true when the steering control has reached its left limit (e.g. motor has reached limits or accel or turn rate limits applied)
    bool     _steering_limit_right; // true when the steering control has reached its right limit (e.g. motor has reached limits or accel or turn rate limits applied)

    // throttle control
    uint32_t _speed_last_ms;        // system time of last call to get_throttle_out_speed
    float    _desired_speed;        // last recorded desired speed
    uint32_t _stop_last_ms;         // system time the vehicle was at a complete stop
    bool     _throttle_limit_low;   // throttle output was limited from going too low (used to reduce i-term buildup)
    bool     _throttle_limit_high;  // throttle output was limited from going too high (used to reduce i-term buildup)
    AP_PIDInfo _throttle_speed_pid_info;   // local copy of throttle_speed controller's PID info to allow reporting of unusual FF

    // balancebot pitch control
    uint32_t _balance_last_ms = 0;  // system time that get_throttle_out_from_pitch was last called
    float _pitch_limit_low = 0;     // min desired pitch (in radians) used to protect against falling over
    float _pitch_limit_high = 0;    // max desired pitch (in radians) used to protect against falling over
    bool _pitch_limited = false;    // true if pitch was limited on last call to get_throttle_out_from_pitch

    // Sailboat heel control
    AC_PID   _sailboat_heel_pid;    // Sailboat heel angle pid controller
    uint32_t _heel_controller_last_ms = 0;
};
