#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

// attitude control default definition
#define AR_ATTCONTROL_STEER_ANG_P       2.50f
#define AR_ATTCONTROL_STEER_RATE_FF     0.20f
#define AR_ATTCONTROL_STEER_RATE_P      0.20f
#define AR_ATTCONTROL_STEER_RATE_I      0.20f
#define AR_ATTCONTROL_STEER_RATE_IMAX   1.00f
#define AR_ATTCONTROL_STEER_RATE_D      0.00f
#define AR_ATTCONTROL_STEER_RATE_FILT   10.00f
#define AR_ATTCONTROL_STEER_RATE_MAX    360.0f
#define AR_ATTCONTROL_STEER_ACCEL_MAX   180.0f
#define AR_ATTCONTROL_THR_SPEED_P       0.20f
#define AR_ATTCONTROL_THR_SPEED_I       0.20f
#define AR_ATTCONTROL_THR_SPEED_IMAX    1.00f
#define AR_ATTCONTROL_THR_SPEED_D       0.00f
#define AR_ATTCONTROL_THR_SPEED_FILT    10.00f
#define AR_ATTCONTROL_PITCH_THR_P       100.0f
#define AR_ATTCONTROL_PITCH_THR_I       80.0f
#define AR_ATTCONTROL_PITCH_THR_D       1.0f
#define AR_ATTCONTROL_PITCH_THR_IMAX    1.0f
#define AR_ATTCONTROL_PITCH_THR_FILT    10.0f
#define AR_ATTCONTROL_DT                0.02f
#define AR_ATTCONTROL_TIMEOUT_MS        200

// throttle/speed control maximum acceleration/deceleration (in m/s) (_ACCEL_MAX parameter default)
#define AR_ATTCONTROL_THR_ACCEL_MAX     2.00f

// minimum speed in m/s
#define AR_ATTCONTROL_STEER_SPEED_MIN   1.0f

// speed (in m/s) at or below which vehicle is considered stopped (_STOP_SPEED parameter default)
#define AR_ATTCONTROL_STOP_SPEED_DEFAULT    0.1f


class AR_AttitudeControl {
public:

    // constructor
    AR_AttitudeControl(AP_AHRS &ahrs);

    //
    // steering controller
    //

    // return a steering servo output given a desired lateral acceleration rate in m/s/s.
    // positive lateral acceleration is to the right.  dt should normally be the main loop rate.
    // return value is normally in range -1.0 to +1.0 but can be higher or lower
    float get_steering_out_lat_accel(float desired_accel, bool motor_limit_left, bool motor_limit_right, float dt);

    // return a steering servo output given a heading in radians
    // return value is normally in range -1.0 to +1.0 but can be higher or lower
    float get_steering_out_heading(float heading_rad, float rate_max, bool motor_limit_left, bool motor_limit_right, float dt);

    // return a steering servo output given a desired yaw rate in radians/sec.
    // positive yaw is to the right
    // return value is normally in range -1.0 to +1.0 but can be higher or lower
    float get_steering_out_rate(float desired_rate, bool motor_limit_left, bool motor_limit_right, float dt);

    // get latest desired turn rate in rad/sec recorded during calls to get_steering_out_rate.  For reporting purposes only
    float get_desired_turn_rate() const;

    // get latest desired lateral acceleration in m/s/s recorded during calls to get_steering_out_lat_accel.  For reporting purposes only
    float get_desired_lat_accel() const;

    // get actual lateral acceleration in m/s/s.  returns true on success.  For reporting purposes only
    bool get_lat_accel(float &lat_accel) const;

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

    // for balancebot
    // return a throttle output from -1 to +1 given a desired pitch angle
    // desired_pitch is in radians
    float get_throttle_out_from_pitch(float desired_pitch, bool armed, float dt);

    // low level control accessors for reporting and logging
    AC_P& get_steering_angle_p() { return _steer_angle_p; }
    AC_PID& get_steering_rate_pid() { return _steer_rate_pid; }
    AC_PID& get_throttle_speed_pid() { return _throttle_speed_pid; }
    AC_PID& get_pitch_to_throttle_pid() { return _pitch_to_throttle_pid; }

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

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // external references
    const AP_AHRS &_ahrs;

    // parameters
    AC_P     _steer_angle_p;        // steering angle controller
    AC_PID   _steer_rate_pid;       // steering rate controller
    AC_PID   _throttle_speed_pid;   // throttle speed controller
    AC_PID   _pitch_to_throttle_pid;// balancebot pitch controller

    AP_Float _throttle_accel_max;   // speed/throttle control acceleration (and deceleration) maximum in m/s/s.  0 to disable limits
    AP_Float _throttle_decel_max;    // speed/throttle control deceleration maximum in m/s/s. 0 to use ATC_ACCEL_MAX for deceleration
    AP_Int8  _brake_enable;         // speed control brake enable/disable. if set to 1 a reversed output to the motors to slow the vehicle.
    AP_Float _stop_speed;           // speed control stop speed.  Motor outputs to zero once vehicle speed falls below this value
    AP_Float _steer_accel_max;      // steering angle acceleration max in deg/s/s
    AP_Float _steer_rate_max;       // steering rate control maximum rate in deg/s

    // steering control
    uint32_t _steer_lat_accel_last_ms;  // system time of last call to lateral acceleration controller (i.e. get_steering_out_lat_accel)
    uint32_t _steer_turn_last_ms;   // system time of last call to steering rate controller
    float    _desired_lat_accel;    // desired lateral acceleration (in m/s/s) from latest call to get_steering_out_lat_accel (for reporting purposes)
    float    _desired_turn_rate;    // desired turn rate (in radians/sec) either from external caller or from lateral acceleration controller

    // throttle control
    uint32_t _speed_last_ms;        // system time of last call to get_throttle_out_speed
    float    _desired_speed;        // last recorded desired speed
    uint32_t _stop_last_ms;         // system time the vehicle was at a complete stop
    bool     _throttle_limit_low;   // throttle output was limited from going too low (used to reduce i-term buildup)
    bool     _throttle_limit_high;  // throttle output was limited from going too high (used to reduce i-term buildup)

    // balancebot pitch control
    uint32_t _balance_last_ms = 0;
};
