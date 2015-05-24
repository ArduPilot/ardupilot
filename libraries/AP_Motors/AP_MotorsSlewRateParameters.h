/// @file	AP_MotorsSlewRateParameters.h
/// @brief	Parameters for AP_MotorsMatrixSlewRateLimited

#pragma once

#include <AP_Param/AP_Param.h>

// Default normal slew rate (change in PWM output / sec)
#define AP_MOTORS_NORMAL_SLEW_RATE_DEFAULT 2400

// Default recovery slew rate (change in PWM output / sec)
#define AP_MOTORS_RECOVERY_SLEW_RATE_DEFAULT 3600

// Default motor recovery output level
#define AP_MOTORS_RECOVERY_OUTPUT_DEFAULT 0.5f

// Default logging interval
#define AP_MOTORS_LOG_INTERVAL_DEFAULT 50

/*****
* Motor failure detection parameters
*/
#define AP_MOTORS_RECOVERY_HIGH_MOTOR_THRESHOLD_DEFAULT 0.95f
#define AP_MOTORS_RECOVERY_LOW_AVG_THRESHOLD_DEFAULT 0.77f
#define AP_MOTORS_RECOVERY_DETECTION_TIME_DEFAULT 200
#define AP_MOTORS_RECOVERY_POST_DELAY_DEFAULT 750
#define AP_MOTORS_RECOVERY_MOT_FILT_HZ_DEFAULT 5.0f
#define AP_MOTORS_SLEW_RATE_LIMITER_LOW_THRESHOLD_DEFAULT 0.36f


/// @class      AP_MotorsSlewRateParameters
class AP_MotorsSlewRateParameters {
public:
    AP_MotorsSlewRateParameters();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];
    
   /****
     * Parameter values
     */

    // slew rate limiter enabled
    AP_Int8 _enabled;

    // the slew rate used during normal motor operation (PWM us/sec)
    AP_Float _normal_slew_rate;

    // the slew rate used when a motor recovery is underway (PWM us/sec)
    AP_Float _recovery_slew_rate;

    // the initial motor output level when a recovery is started (proportion of full output, 0-1.0)
    AP_Float _recovery_output;

    // motor failure detection high threshold (proportion of full output, 0-1.0)
    AP_Float _detection_high_threshold;

    // motor failure detection low threshold (proportion of full output, 0-1.0)
    AP_Float _detection_low_threshold;

    // amount of time output imbalance must be present before triggering failure recovery
    AP_Int32 _detection_time_ms;

    // amount of time after a failure recovery before a new failure recovery can be initated
    AP_Int32 _post_recovery_time_ms;

    // failure detection low pass filter frequncy (Hz)
    AP_Float _detection_filter_frequency;

    // output power at which slew rate limiter becomes active
    AP_Float _limiter_low_threshold;

    // minimum time between log messages
    AP_Int32 _log_interval_ms;

    // debug slew rate limiter
    AP_Int8 _debug;
};
