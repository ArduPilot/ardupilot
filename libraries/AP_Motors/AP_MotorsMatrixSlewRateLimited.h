/// @file	AP_MotorsMatrixSlewRateLimited.h
/// @brief	Motor control class for Matrixcopters with output slew rate limiting
#pragma once

#include <DataFlash/DataFlash.h>
#include "AP_MotorsMatrix.h"
#include "AP_MotorsSlewRateParameters.h"

/// @class      AP_MotorsMatrixSlewRateLimited
class AP_MotorsMatrixSlewRateLimited : public AP_MotorsMatrix {
public:
    AP_MotorsMatrixSlewRateLimited(AP_MotorsSlewRateParameters& parameters, uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // override of init from parent class
    void init(motor_frame_class frame_class, motor_frame_type frame_type);

    // override of output_to_motors
    void output_to_motors();

    // override of rc_write
    void rc_write(uint8_t chan, uint16_t pwm);

protected:
    // Motor recovery states
    enum motor_recovery_state {
        MOTOR_STATE_NORMAL = 0,
        MOTOR_STATE_RECOVERY = 1,
        MOTOR_STATE_POST_RECOVERY = 2
    };

    // check and update the current recovery state
    void update_recovery_state();

    // checks for motor failure, returns true of a failure has been detected
    bool detect_motor_failure(uint64_t current_time);

    // resets the recovery state
    void reset_failure_detector(uint64_t current_time);

    // start a motor recovery
    void start_motor_recovery(uint64_t current_time);

    // return the requested output level for the given motor as a
    // proportion of full output (0.0 - 1.0)
    float get_requested_motor_out(uint8_t mot);

    // convert the given pwm value to a proportion of full output (0.0 - 1.0)
    float pwm_to_out(uint16_t pwm);

    // convert a value representing the proportion of full output (0.0 - 1.0)
    // to a pwm value
    float out_to_pwm(float output);

    // set the slew rate per interation value based on a slew per second rate
    void set_slew_rate(float slew_per_second);

    AP_MotorsSlewRateParameters& _params;

    // Next time that the system should check for a state change. 0 indicates right away.
    uint64_t _next_state_transition_time = 0;

    // Current state
    motor_recovery_state _recovery_state = MOTOR_STATE_NORMAL;

    // Current slew rate in PWM change / loop iteration
    float _slew_per_iteration = 0;

    /***
     * Motor state information
     */

    // last pwm output sent to motor
    float _motor_out_pwm_actual[AP_MOTORS_MAX_NUM_MOTORS];

    // current maximum allowed pwm output - adjusted up and down by slew rate limiter
    float _motor_out_pwm_max[AP_MOTORS_MAX_NUM_MOTORS];

    // last requested pwm output (not always equal to actual about, if slew rate is limited)
    uint16_t _motor_out_pwm_requested[AP_MOTORS_MAX_NUM_MOTORS];

    /*****
    * Motor failure detector state
    */
    uint8_t _last_highest_motor = 0;
    uint64_t _last_motor_good_time = 0;
    float _motor_out_filtered[AP_MOTORS_MAX_NUM_MOTORS];

    // track total number of failure events since start
    uint16_t _failure_count = 0;

    // time of next log message for log message throttling
    uint64_t _next_log_time = 0;

private:
    // set the motor recovery state
    void set_state(motor_recovery_state new_state);

    // log the current state of the slew rate limiter
    void log_state(bool force);
};
