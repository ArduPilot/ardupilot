/// @file	AP_MotorsMatrixSlewRateLimited.h
/// @brief	Motor control class for Matrixcopters with output slew rate limiting
#pragma once

#include <DataFlash/DataFlash.h>
#include "AP_MotorsMatrix.h"

// Default normal slew rate (change in PWM output / sec)
#define AP_MOTORS_NORMAL_SLEW_RATE_DEFAULT 2400

// Default normal slew rate (change in PWM output / sec)
#define AP_MOTORS_RECOVERY_SLEW_RATE_DEFAULT 3600

// Default motor recovery output level
#define AP_MOTORS_RECOVERY_OUTPUT_DEFAULT 0.5f

/*****
* Motor failure detection parameters
*/
#define AP_MOTORS_RECOVERY_HIGH_MOTOR_THRESHOLD_DEFAULT 0.95f
#define AP_MOTORS_RECOVERY_LOW_AVG_THRESHOLD_DEFAULT 0.77f
#define AP_MOTORS_RECOVERY_DETECTION_TIME_DEFAULT 200
#define AP_MOTORS_RECOVERY_POST_DELAY_DEFAULT 750
#define AP_MOTORS_RECOVERY_MOT_FILT_HZ_DEFAULT 5.0f

/// @class      AP_MotorsMatrixSlewRateLimited
class AP_MotorsMatrixSlewRateLimited : public AP_MotorsMatrix {
public:

    // Constructor
    AP_MotorsMatrixSlewRateLimited(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // rc_write is where we hook in the slew rate limter
    void rc_write(uint8_t chan, uint16_t pwm);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // Motor recovery states
    enum motor_recovery_state {
        MOTOR_STATE_NORMAL = 0,
        MOTOR_STATE_RECOVERY = 1,
        MOTOR_STATE_POST_RECOVERY = 2
    };

    void init(motor_frame_class frame_class, motor_frame_type frame_type);

    // check and update the current recovery state
    void update_recovery_state();

    // sets the current recovery state
    void set_recovery_state(uint32_t tnow_ms, motor_recovery_state new_recovery_state);

    // checks for motor failure, returns true of a failure has been detected
    bool detect_motor_failure(uint32_t tnow_ms);

    // resets the recovery state
    void reset_failure_detector(uint32_t tnow_ms);

    // start a motor recovery
    void start_motor_recovery(uint32_t tnow_ms);

    // return the current output level for the given motor as a
    // proportion of full output (0.0 - 1.0)
    float get_motor_out(uint8_t mot);

    // convert the given pwm value to a proportion of full output (0.0 - 1.0)
    float pwm_to_out(uint16_t pwm);

    // convert a value representing the proportion of full output (0.0 - 1.0)
    // to a pwm value
    float out_to_pwm(float output);

    void set_slew_rate(float slew_per_second);

    void set_filter_alpha();

    /****
     * Parameter values
     */
    AP_Float _normal_slew_rate;

    AP_Float _recovery_slew_rate;

    AP_Float _recovery_output;

    AP_Float _detection_high_threshold;

    AP_Float _detection_low_threshold;

    AP_Int32 _detection_time;

    AP_Int32 _post_recovery_time;

    AP_Float _detection_filter_frequency;

    /**
     * Other member variables
     */

    // Next time that the system should check for a state change. 0 indicates right away.
    uint32_t _next_state_transition_time = 0;

    // Current state
    motor_recovery_state _recovery_state;

    // Current slew rate in PWM change / loop iteration
    float _slew_per_iteration;

    /***
     * Motor state information
     */

    // last motor output, used as refererence point when calculating new output
    float _motor_out_pwm[AP_MOTORS_MAX_NUM_MOTORS];

    /*****
    * Motor failure detector state
    */
    uint8_t _last_highest_motor = 0;
    uint32_t _last_motor_good_time = 0;
    float _motor_out_filtered[AP_MOTORS_MAX_NUM_MOTORS];

    // track total number of failure events since start
    uint16_t _failure_count = 0;

    // calculated alpha value for motor failure detection low pass filter
    float _motor_filt_alpha;
};
