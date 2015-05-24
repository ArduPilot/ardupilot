/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
/**
 *
 * AP_MotorsSlewRateParameters.cpp - ArduCopter motors library
 *
 * Description:
 *
 * As there doesn't seem to be a straightforward way to define 
 * parameters for subclasses, this class provides a parameter "container"
 * holding all the parameters for AP_MotorsMatrixSlewRateLimited, which
 * is a subclass of AP_MotorsMatrix.
 *
 */
 
#include "AP_MotorsSlewRateParameters.h"

// parameters for the AP_MotorsMatrixSlewRateLimited class
const AP_Param::GroupInfo AP_MotorsSlewRateParameters::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Slew rate limiter enabled or disabled
    // @Description: Slew rate limiter enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED", 0, AP_MotorsSlewRateParameters, _enabled, 0, AP_PARAM_FLAG_ENABLE),
    
    // @Param: RT_NORM
    // @DisplayName: Normal Motor Slew Rate
    // @Description: Maximum allowed change in motor PWM output per second, during normal operation
    // @Units: PWM/s
    // @Range: 0 25000
    // @User: Advanced
    AP_GROUPINFO("RT_NORM", 1, AP_MotorsSlewRateParameters, _normal_slew_rate, AP_MOTORS_NORMAL_SLEW_RATE_DEFAULT),

    // @Param: RT_RCOV
    // @DisplayName: Recovery Motor Slew Rate
    // @Description: Maximum allowed change in motor PWM output per second, during recovery / restart
    // @Units: PWM/s
    // @Range: 0 25000
    // @User: Advanced
    AP_GROUPINFO("RT_RCOV", 2, AP_MotorsSlewRateParameters, _recovery_slew_rate, AP_MOTORS_RECOVERY_SLEW_RATE_DEFAULT),

    // @Param: RCOV_OUT
    // @DisplayName: Motor Recovery Output Power
    // @Description: Motor output level to use when starting motor recovery (proportion of max pwm output)
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("RCOV_OUT", 3, AP_MotorsSlewRateParameters, _recovery_output, AP_MOTORS_RECOVERY_OUTPUT_DEFAULT),

    // @Param: DET_HIGH
    // @DisplayName: Motor Failure Detection High Threshold
    // @Description: Motor output level high threshold for failure detection (proportion of max pwm output)
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DET_HIGH", 4, AP_MotorsSlewRateParameters, _detection_high_threshold, AP_MOTORS_RECOVERY_HIGH_MOTOR_THRESHOLD_DEFAULT),

    // @Param: DET_LOW
    // @DisplayName: Motor Failure Detection Low Threshold
    // @Description: Motor output level low threshold for failure detection (proportion of max pwm output)
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DET_LOW", 5, AP_MotorsSlewRateParameters, _detection_low_threshold, AP_MOTORS_RECOVERY_LOW_AVG_THRESHOLD_DEFAULT),

    // @Param: DET_TIME
    // @DisplayName: Motor Failure Detection Time
    // @Description: Minimum time exceeding threshold to trigger failure detection
    // @Units: ms
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("DET_TIME", 6, AP_MotorsSlewRateParameters, _detection_time_ms, AP_MOTORS_RECOVERY_DETECTION_TIME_DEFAULT),

    // @Param: DET_FREQ
    // @DisplayName: Motor Failure Detection Filter Frequency
    // @Description: Frequency used in failure detection low pass filter
    // @Units: Hz
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("DET_FREQ", 7, AP_MotorsSlewRateParameters, _detection_filter_frequency, AP_MOTORS_RECOVERY_MOT_FILT_HZ_DEFAULT),

    // @Param: DLY_TIME
    // @DisplayName: Motor Failure Post Recovery Delay Time
    // @Description: Time after motor recovery during which failure detection is disabled
    // @Units: ms
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("DLY_TIME", 8, AP_MotorsSlewRateParameters, _post_recovery_time_ms, AP_MOTORS_RECOVERY_POST_DELAY_DEFAULT),

    // @Param: LIM_LOW
    // @DisplayName: Lower Output Power Threshold for Limiter
    // @Description: Output power above which the slew rate limiter becomes active.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("LIM_LOW", 9, AP_MotorsSlewRateParameters, _limiter_low_threshold, AP_MOTORS_SLEW_RATE_LIMITER_LOW_THRESHOLD_DEFAULT),

    // @Param: LOG_INT
    // @DisplayName: Slew Rate Limiter Logging Interval
    // @Description: Minimum time between slew rate limiter log messages.
    // @Units: ms
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("LOG_INT", 10, AP_MotorsSlewRateParameters, _log_interval_ms, AP_MOTORS_LOG_INTERVAL_DEFAULT),

    // @Param: DEBUG
    // @DisplayName: Slew Rate Limiter Debug Enable
    // @Description: Enable debugging for slew rate limiter.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 11, AP_MotorsSlewRateParameters, _debug, 0),

    AP_GROUPEND
};


/**
 * Constructor
 */
AP_MotorsSlewRateParameters::AP_MotorsSlewRateParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
