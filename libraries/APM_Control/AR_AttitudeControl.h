#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

// attitude control default definition
#define AR_ATTCONTROL_STEER_ANG_P       1.00f
#define AR_ATTCONTROL_STEER_RATE_P      1.00f
#define AR_ATTCONTROL_STEER_RATE_I      0.50f
#define AR_ATTCONTROL_STEER_RATE_IMAX   1.00f
#define AR_ATTCONTROL_STEER_RATE_D      0.00f
#define AR_ATTCONTROL_STEER_RATE_FILT   5.00f
#define AR_ATTCONTROL_THR_SPEED_P       0.20f
#define AR_ATTCONTROL_THR_SPEED_I       0.20f
#define AR_ATTCONTROL_THR_SPEED_IMAX    1.00f
#define AR_ATTCONTROL_THR_SPEED_D       0.00f
#define AR_ATTCONTROL_THR_SPEED_FILT    5.00f
#define AR_ATTCONTROL_DT                0.02f

// throttle/speed control maximum acceleration/deceleration (in m/s) (_ACCEL_MAX parameter default)
#define AR_ATTCONTROL_THR_ACCEL_MAX     5.00f

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

    // return a steering servo output from -1.0 to +1.0 given a desired lateral acceleration rate in m/s/s.
    // positive lateral acceleration is to the right.
	float get_steering_out_lat_accel(float desired_accel, bool skid_steering, bool motor_limit_left, bool motor_limit_right);

    // return a steering servo output from -1 to +1 given a yaw error in radians
    float get_steering_out_angle_error(float angle_err, bool skid_steering, bool motor_limit_left, bool motor_limit_right);

	// return a steering servo output from -1 to +1 given a
    // desired yaw rate in radians/sec. Positive yaw is to the right.
	float get_steering_out_rate(float desired_rate, bool skid_steering, bool motor_limit_left, bool motor_limit_right);

	//
	// throttle / speed controller
	//

    // set limits used by throttle controller
    //   forward/back acceleration max in m/s/s
    //   forward/back deceleartion max in m/s/s
    void set_throttle_limits(float throttle_accel_max, float throttle_decel_max);

    // return a throttle output from -1 to +1 given a desired speed in m/s (use negative speeds to travel backwards)
    //   skid_steering should be true for skid-steer vehicles
    //   motor_limit should be true if motors have hit their upper or lower limits
    //   cruise speed should be in m/s, cruise throttle should be a number from -1 to +1
    float get_throttle_out_speed(float desired_speed, bool skid_steering, bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle);

    // return a throttle output from -1 to +1 to perform a controlled stop.  stopped is set to true once stop has been completed
    float get_throttle_out_stop(bool skid_steering, bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, bool &stopped);

    // low level control accessors for reporting and logging
    AC_P& get_steering_angle_p() { return _steer_angle_p; }
    AC_PID& get_steering_rate_pid() { return _steer_rate_pid; }
    AC_PID& get_throttle_speed_pid() { return _throttle_speed_pid; }

    // get forward speed in m/s (earth-frame horizontal velocity but only along vehicle x-axis).  returns true on success
    bool get_forward_speed(float &speed) const;

    // get throttle/speed controller maximum acceleration (also used for deceleration)
    float get_accel_max() const { return MAX(_throttle_accel_max, 0.0f); }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // external references
    AP_AHRS &_ahrs;

    // parameters
    AC_P     _steer_angle_p;        // steering angle controller
	AC_PID   _steer_rate_pid;       // steering rate controller
	AC_PID   _throttle_speed_pid;   // throttle speed controller
	AP_Float _throttle_accel_max;   // speed/throttle control acceleration (and deceleration) maximum in m/s/s.  0 to disable limits
	AP_Int8  _brake_enable;         // speed control brake enable/disable. if set to 1 a reversed output to the motors to slow the vehicle.
	AP_Float _stop_speed;           // speed control stop speed.  Motor outputs to zero once vehicle speed falls below this value

    // steering control
    uint32_t _steer_turn_last_ms;   // system time of last call to steering rate controller

    // throttle control
    uint32_t _speed_last_ms;        // system time of last call to get_throttle_out_speed
    float    _desired_speed;        // last recorded desired speed
    uint32_t _stop_last_ms;         // system time the vehicle was at a complete stop
    bool     _throttle_limit_low;   // throttle output was limited from going too low (used to reduce i-term buildup)
    bool     _throttle_limit_high;  // throttle output was limited from going too high (used to reduce i-term buildup)
};
