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
 * AP_MotorsMatrixSlewRateLimited.cpp - ArduCopter motors library
 *
 * Original code by Jonathan Challinger
 *
 * Refactored / adapted to AC3.5 by Hugh Eaves
 *
 * Description:
 *
 * AP_MotorsMatrixSlewRateLimited is an AP_MotorsMatrix, but with the
 * additional capability of limiting the rate of change of the motor
 * outputs. (i.e. slew rate limiting) This is primarily to workaround
 * an issue with signal propogation with the 3DR Solo ESC controllers,
 * but could be applied to other applications where limiting the
 * output slew rate could be useful.
 *
 */
 
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrixSlewRateLimited.h"

/**
 * Constructor
 */
AP_MotorsMatrixSlewRateLimited::AP_MotorsMatrixSlewRateLimited(AP_MotorsSlewRateParameters& parameters, uint16_t loop_rate, uint16_t speed_hz) :
     AP_MotorsMatrix(loop_rate, speed_hz),
    _recovery_state(MOTOR_STATE_NORMAL),
    _params(parameters)
{

    memset(_motor_out_pwm_actual, 0, sizeof(_motor_out_pwm_actual));
    memset(_motor_out_pwm_max, 0, sizeof(_motor_out_pwm_max));
    memset(_motor_out_pwm_requested, 0, sizeof(_motor_out_pwm_requested));
    memset(_motor_out_filtered, 0, sizeof(_motor_out_filtered));
}

/**
 * Re-initialize some state variables after the parameters have been set
 */
void AP_MotorsMatrixSlewRateLimited::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    AP_MotorsMatrix::init(frame_class, frame_type);
    _next_state_transition_time = 0;
    _recovery_state = MOTOR_STATE_NORMAL;
    set_slew_rate(_params._normal_slew_rate);
}

/**
 * Override output_to_motors, just so we can call update_recovery_state()
 * once per loop iteration.
 */
void AP_MotorsMatrixSlewRateLimited::output_to_motors()
{
    AP_MotorsMatrix::output_to_motors();
    update_recovery_state();
    log_state(false);
}

/**
 * Override rc_write so we can constrain the output
 * based on the slew rate (if needed),
 * and keep a record of motor output state
 */
void AP_MotorsMatrixSlewRateLimited::rc_write(uint8_t chan, uint16_t pwm)
{
   // record the requested output value
   _motor_out_pwm_requested[chan] = pwm;
   
   // ensure _motor_out_pwm_max[chan] is never below activation threshold
   _motor_out_pwm_max[chan] = MAX(_motor_out_pwm_max[chan], out_to_pwm(_params._limiter_low_threshold) - _slew_per_iteration);

   // calculate difference between requested output and current max output
   float output_delta = pwm - _motor_out_pwm_max[chan];

   // adjust the max output, based on the requested output
   if (output_delta > _slew_per_iteration) {
       output_delta = _slew_per_iteration;
   } else if (output_delta < -_slew_per_iteration) {
       output_delta = -_slew_per_iteration;
   }
   _motor_out_pwm_max[chan] += output_delta;

   // constrain output to be less than or equal to max output
   pwm = MIN(pwm, _motor_out_pwm_max[chan]);

   // Now that we've done all the calculations, call the parent
   // class rc_write to actually output the new values.
   // The parent method handles channel remapping, etc.
   AP_MotorsMatrix::rc_write(chan, pwm);

   // record the actual output value
   _motor_out_pwm_actual[chan] = pwm;
}

void AP_MotorsMatrixSlewRateLimited::update_recovery_state()
{
    uint64_t current_time = AP_HAL::micros64();

    if (!armed())
    {
        reset_failure_detector(current_time);
        return;
    }

    // check to see if we need to re-examine the recovery state
    if (current_time >= _next_state_transition_time)
    {
        switch (_recovery_state)
        {
        case MOTOR_STATE_NORMAL:
            if (detect_motor_failure(current_time)) {
                ++_failure_count;
                set_state(MOTOR_STATE_RECOVERY);
                start_motor_recovery(current_time);
            }
            break;

        case MOTOR_STATE_RECOVERY:
            set_state(MOTOR_STATE_POST_RECOVERY);
            set_slew_rate(_params._normal_slew_rate);
            reset_failure_detector(current_time);
            _next_state_transition_time = current_time + _params._post_recovery_time_ms * 1000;
            break;

        case MOTOR_STATE_POST_RECOVERY:
            set_state(MOTOR_STATE_NORMAL);
            set_slew_rate(_params._normal_slew_rate);
            reset_failure_detector(current_time);
            _next_state_transition_time = 0;
            break;
        }
    }
}

bool AP_MotorsMatrixSlewRateLimited::detect_motor_failure(uint64_t current_time)
{
    // find highest motor and motor average
    uint8_t highest_motor_index = 0;
    float highest_motor = 0.0f;
    float motor_avg = 0.0f;
    uint8_t motor_count = 0;
    bool failed = false;

    // calculate alpha value for motor failure detection low pass filter
    float motor_filt_alpha = constrain_float((2.0f*M_PI*_params._detection_filter_frequency) /
       ((2.0f*M_PI*_params._detection_filter_frequency) + _loop_rate),0.0f,1.0f);

    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _motor_out_filtered[i] += (get_requested_motor_out(i) - _motor_out_filtered[i]) * motor_filt_alpha;

        if (motor_enabled[i]) {
            if (_motor_out_filtered[i] > highest_motor) {
                highest_motor = _motor_out_filtered[i];
                highest_motor_index = i;
            }

            motor_avg += _motor_out_filtered[i];
            motor_count++;
        }
    }

    motor_avg /= motor_count;


    bool motor_fail_criteria_met = (highest_motor_index == _last_highest_motor) &&
       (highest_motor >= _params._detection_high_threshold) && (motor_avg <= _params._detection_low_threshold);

    if (!motor_fail_criteria_met) {
        _last_motor_good_time = current_time;
    } else if(current_time - _last_motor_good_time > ((uint32_t)_params._detection_time_ms) * 1000) {
        // if criteria for a motor failure are met continuously for _detection_time seconds
       failed = true;
    }

    _last_highest_motor = highest_motor_index;

    return failed;
}

/**
 * Start a motor recovery
 */
void AP_MotorsMatrixSlewRateLimited::start_motor_recovery(uint64_t current_time)
{
    set_slew_rate(_params._recovery_slew_rate);

    // Set the last motor output based on the recovery percentage
    // The slew rate limiter will force the actual outputs to be this
    // value +- one slew rate increment during the next loop iteration.
    float recovery_pwm = out_to_pwm(_params._recovery_output) - _slew_per_iteration;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _motor_out_pwm_max[i] = recovery_pwm;
        }
    }

    // calculate the total recovery time, in seconds
    float recovery_time_length = (get_pwm_output_max() - out_to_pwm(_params._recovery_output)) / _params._recovery_slew_rate;
    // set the state transition time after recovery is complete
    _next_state_transition_time = current_time + (recovery_time_length * 1000 * 1000);
}

/**
 * Get the motor output as a percentage
 */
float AP_MotorsMatrixSlewRateLimited::get_requested_motor_out(uint8_t mot)
{
    return (mot<AP_MOTORS_MAX_NUM_MOTORS) ? pwm_to_out(_motor_out_pwm_requested[mot]) : 0.0f;
}

/**
 * Convert a PWM value to a proportion of full output, using the
 * configured pwm_output_max and pwm_output_min.
 */
float AP_MotorsMatrixSlewRateLimited::pwm_to_out(uint16_t pwm)
{
    return ((float)pwm - get_pwm_output_min()) / (get_pwm_output_max()-get_pwm_output_min());
}

/**
 * Convert a proportion of full output to a PWM value, using the
 * configured pwm_output_max and pwm_output_min.
 */
float AP_MotorsMatrixSlewRateLimited::out_to_pwm(float out)
{
    return (get_pwm_output_min() + ((get_pwm_output_max()-get_pwm_output_min()) * out));
}

/**
 * Set the slew per iteration value based on a slew rate in slew per seconds
 */
void AP_MotorsMatrixSlewRateLimited::set_slew_rate(float slew_per_second)
{
    _slew_per_iteration = slew_per_second / _loop_rate;
}

/**
 * Clears / resets the motor failure detection state
 */
void AP_MotorsMatrixSlewRateLimited::reset_failure_detector(uint64_t current_time)
{
    _last_motor_good_time = current_time;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _motor_out_filtered[i] = get_requested_motor_out(i);
    }
}

void AP_MotorsMatrixSlewRateLimited::set_state(motor_recovery_state new_state)
{
    _recovery_state = new_state;
    log_state(true);
}

void AP_MotorsMatrixSlewRateLimited::log_state(bool force)
{
    uint64_t current_time = AP_HAL::micros64();

    if (force || current_time > _next_log_time) {
        _next_log_time = current_time + (_params._log_interval_ms * 1000);

        DataFlash_Class::instance()->Log_Write("SLW1",
            "TimeUS,State,FailCnt,NxtTrans,SlwRt", "QIHIf",
            current_time, _recovery_state,
            _failure_count, _next_state_transition_time, _slew_per_iteration);

        if (_params._debug != 0) {
            for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    DataFlash_Class::instance()->Log_Write("SLW2",
                        "TimeUS,Motor,Req,Act,Max,Filt",
                        "QIffIf", current_time, i,
                        _motor_out_pwm_requested[i], _motor_out_pwm_actual[i],
                        _motor_out_pwm_max[i], _motor_out_filtered[i]);
                }
            }

            DataFlash_Class::instance()->Log_Write("SLW3",
                "TimeUS,LstHstMtr,LstGdTm",
                "QIII", current_time, _last_highest_motor, _last_motor_good_time);
        }
    }
}


