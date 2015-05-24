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
 * Brief Description:
 *
 * AP_MotorsMatrixSlewRateLimited is an AP_MotorsMatrix, but with the
 * additional capability of limiting the rate of change of the motor
 * outputs. (i.e. slew rate limiting) This is primarily to workaround
 * an issue with signal propogation with the 3DR Solo ESC controllers,
 * but could be applied to other applications where limiting the
 * output slew rate could be useful.
 *
 *
 * Details:
 *
 * The PixHawk 2.0 in the 3DR Solo uses 3.3V logic levels for PWM signaling to
 * the Solo ESC's. The Solo ESC's use 5V logic levels. Although not ideal, under
 * normal conditions, the 3.3V output is voltage is sufficient to be recognized
 * as "logic high" by the 5V logic of the ESC's. However, during times of high
 * motor output, the Solo power distribution system allows the ground voltage at
 * the ESC to "drift upwards" by up to 1 volt. Under these conditions, the
 * potential difference at the ESC for logic high can be as low as 2.3V. This
 * voltage level is not sufficient to be detected by the ESC. The net result is
 * that during high load, the PWM signal to the ESC is no longer recognized, and
 * the ESC shuts down the motor due to lack of a PWM signal.
 * 
 * Unfortunately, when this happens, the aircraft starts to tilt towards the
 * failed motor, causing the flight controller to compensate by increasing the
 * output to that motor. Obviously, this positive feedback loop makes it
 * difficult for the motor to recover communication, and may lead to a situation
 * in which the force of gravity exceeds the average compensating thrust for an
 * extended period of time.
 *
 * 3DR introduced a workaround for this problem that monitored the motor outputs
 * for a sustained (200ms) and large (>18%) difference in the motor output
 * power. If an imbalance was detected, the output power was reset to 50% for
 * all motors, and then slowly ramped back up. In theory, this reset and soft
 * restart blocked the positive feedback resulting from the failure, allowing
 * the motors to restablish communication.
 *
 * Not long afterwards, 3DR introduced a different approach to working around
 * this problem. The new approach was to limit the rate of change (slew rate) of
 * the PWM outputs, to limit the additional current draw during prop
 * accelration. The slew rate was limited 6 PWM units (microseconds) per fast
 * loop iteration (400Hz). 3DR probably felt that this workaround would prevent
 * any communication failures, so they disabled the previous failure detection /
 * recovery code.
 *
 * A few months later, 3DR decided to renable the motor recovery code, so the
 * "final" workaround was to have both slew rate limiting and motor recovery
 * enabled.
 *
 * The important parameters from 3DR's original implementation are:
 *
 *    Max slew rate: +- 6 PWM units (microseconds) / main loop iteration
 *      - With the main loop of 400Hz, this is 2400 PWM units / second.
 *
 *    Initial Motor recovery output level: 50% * (max pwm - min pwm) + min pwm
 *        Recovery output is percentage of difference between min / max pwm.
 *
 *    Motor recovery time: 250 milliseconds to go from min pwm to max pwm
 *        As the motor recovery started at 50%, the actual recovery time is 125ms.
 *        This recovery time is equivalent to a slew rate of 3600 PWM units / second.
 *
 * Note that all of these parameters are based on PWM values, and do not take
 * into account thrust curves / limits / etc.
 *
 * All of the recovery / slew rate code is implemented in the final stage of the
 * motor output pipeline. The "requested" output level is filtered by the
 * reovery code, and the filtered result is the output level actually sent to
 * the motors.
 *
 * When the motor is in a recovery state, the recovery code enforces a linearly
 * increasing (with time) constraint on the maximum PWM output passed through to
 * the motors. The recovery code does not limit the slew rate, just the output.
 * In theory, if the requested output level was 50% of the max, and then at the
 * last moment of recovery, increased to 100%, the actual PWM output would
 * immediately increase from 50% to 100%. However, the input to the motor
 * recovery filter is slew rate limited, so in practice, the rate of change in
 * the output is going to fall between 2400 PWM units / second, and 3600 PWM
 * units / second.
 *
 * AP_MotorsMatrixSlewRateLimited is a rewrite of 3DR's original slew rate
 * limiting, motor failure detection, and motor recovery code. The main
 * difference between this implementation and the original is that the slew rate
 * limiter in this implementation also handles the soft restart for motor
 * recovery. In the original code, these functions were handled differently.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrixSlewRateLimited.h"

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsMatrixSlewRateLimited::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // @Param: SLW_RT_NORM
    // @DisplayName: Normal Motor Slew Rate
    // @Description: Maximum allowed change in motor PWM output per second
    // @Units: pwm/sec
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("SLW_RT_NORM", 1, AP_MotorsMatrixSlewRateLimited, _normal_slew_rate, AP_MOTORS_NORMAL_SLEW_RATE_DEFAULT),

    // @Param: SLW_RT_RCOV
    // @DisplayName: Recovery Motor Slew Rate
    // @Description: Maximum allowed change in motor PWM output per second, during recovery / restart
    // @Units: pwm/sec
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("SLW_RT_RCOV", 2, AP_MotorsMatrixSlewRateLimited, _recovery_slew_rate, AP_MOTORS_RECOVERY_SLEW_RATE_DEFAULT),

    // @Param: SLW_RCOV_OUT
    // @DisplayName: Motor Recovery Output Power
    // @Description: Motor output level to use when starting motor recovery (proportion of max pwm output)
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SLW_RCOV_OUT", 3, AP_MotorsMatrixSlewRateLimited, _recovery_output, AP_MOTORS_RECOVERY_OUTPUT_DEFAULT),

    // @Param: SLW_DET_HIGH
    // @DisplayName: Motor Failure Detection High Threshold
    // @Description: Motor output level high threshold for failure detection
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SLW_DET_HIGH", 4, AP_MotorsMatrixSlewRateLimited, _detection_high_threshold, AP_MOTORS_RECOVERY_HIGH_MOTOR_THRESHOLD_DEFAULT),

    // @Param: SLW_DET_LOW
    // @DisplayName: Motor Failure Detection Low Threshold
    // @Description: Motor output level low threshold for failure detection
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SLW_DET_LOW", 5, AP_MotorsMatrixSlewRateLimited, _detection_low_threshold, AP_MOTORS_RECOVERY_LOW_AVG_THRESHOLD_DEFAULT),

    // @Param: SLW_DET_TIME
    // @DisplayName: Motor Failure Detection Time
    // @Description: Minimum time exceeding threshold to trigger failure detection
    // @Units: milliseconds
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("SLW_DET_TIME", 6, AP_MotorsMatrixSlewRateLimited, _detection_time, AP_MOTORS_RECOVERY_DETECTION_TIME_DEFAULT),

    // @Param: SLW_DET_FREQ
    // @DisplayName: Motor Failure Detection Filter Frequency
    // @Description: Frequency used in failure detection low pass filter
    // @Units: hertz
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("SLW_DET_FREQ", 7, AP_MotorsMatrixSlewRateLimited, _detection_filter_frequency, AP_MOTORS_RECOVERY_MOT_FILT_HZ_DEFAULT),

    // @Param: SLW_DLY_TIME
    // @DisplayName: Motor Failure Post Recovery Delay Time
    // @Description: Time after motor recovery during which failure detection is disabled
    // @Units: milliseconds
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("SLW_DLY_TIME", 8, AP_MotorsMatrixSlewRateLimited, _post_recovery_time, AP_MOTORS_RECOVERY_POST_DELAY_DEFAULT),

    AP_GROUPEND
};

/**
 * Constructor
 */
AP_MotorsMatrixSlewRateLimited::AP_MotorsMatrixSlewRateLimited(uint16_t loop_rate, uint16_t speed_hz) :
    _next_state_transition_time(0),
    _recovery_state(MOTOR_STATE_NORMAL),
    AP_MotorsMatrix(loop_rate, speed_hz)
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(_motor_out_pwm, 0, sizeof(_motor_out_pwm));
    set_filter_alpha();
    set_slew_rate(_normal_slew_rate);
}

/**
 * Re-initialize some state variables after the parameters have been set
 */
void AP_MotorsMatrixSlewRateLimited::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    AP_MotorsMatrix::init(frame_class, frame_type);
    _recovery_state = MOTOR_STATE_NORMAL;
    _next_state_transition_time = 0;
    set_filter_alpha();
    set_slew_rate(_normal_slew_rate);
}


/**
 * Override rc_write so we can constrain the output
 * based on the slew rate (if needed),
 * and update the recovery state
 */
void AP_MotorsMatrixSlewRateLimited::rc_write(uint8_t chan, uint16_t pwm)
{
   // get last output, ensuring its not less than the minimum output
   float last_output = MAX(_motor_out_pwm[chan], get_pwm_output_min() - _slew_per_iteration);

   float output_delta = pwm - last_output;

   // constrain the maximum change, based on the slew rate
   if (output_delta > _slew_per_iteration) {
       output_delta = _slew_per_iteration;
   } else if (output_delta < -_slew_per_iteration) {
       output_delta = -_slew_per_iteration;
   }

   // calculate the new output, constrained by the slew rate
   float new_output = last_output + output_delta;

   // final check to make sure PWM range not exceeded
   new_output = constrain_float(new_output, get_pwm_output_min(), get_pwm_output_max());

   // Now that we've done all the slew calculations, call the parent
   // class rc_write to actually output the new values.
   // The parent method handles channel remapping, etc.
   AP_MotorsMatrix::rc_write(chan, new_output);

   // record the new output value
   _motor_out_pwm[chan] = new_output;

   // update the motor state
   update_recovery_state();
}

void AP_MotorsMatrixSlewRateLimited::update_recovery_state()
{
    uint32_t tnow_ms = AP_HAL::millis();

    if (!armed())
    {
        reset_failure_detector(tnow_ms);
        return;
    }

    // check to see if we need to re-examine the recovery state
    if (tnow_ms >= _next_state_transition_time)
    {
        switch (_recovery_state)
        {
        case MOTOR_STATE_NORMAL:
            if (detect_motor_failure(tnow_ms)) {
                ++_failure_count;
                set_recovery_state(tnow_ms, MOTOR_STATE_RECOVERY);
                start_motor_recovery(tnow_ms);
            }
            break;

        case MOTOR_STATE_RECOVERY:
            set_recovery_state(tnow_ms, MOTOR_STATE_POST_RECOVERY);
            set_slew_rate(_normal_slew_rate);
            reset_failure_detector(tnow_ms);
            _next_state_transition_time = tnow_ms + _post_recovery_time;
            break;

        case MOTOR_STATE_POST_RECOVERY:
            set_recovery_state(tnow_ms, MOTOR_STATE_NORMAL);
            set_slew_rate(_normal_slew_rate);
            reset_failure_detector(tnow_ms);
            _next_state_transition_time = 0;
            break;
        }
    }
}

bool AP_MotorsMatrixSlewRateLimited::detect_motor_failure(uint32_t tnow_ms)
{
    // find highest motor and motor average
    uint8_t highest_motor_index = 0;
    float highest_motor = 0.0f;
    float motor_avg = 0.0f;
    uint8_t motor_count = 0;
    bool failed = false;

    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _motor_out_filtered[i] += (get_motor_out(i) - _motor_out_filtered[i]) * _motor_filt_alpha;

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
       (highest_motor >= _detection_high_threshold) && (motor_avg <= _detection_low_threshold);

    if (!motor_fail_criteria_met) {
        _last_motor_good_time = tnow_ms;
    } else if(tnow_ms - _last_motor_good_time > _detection_time) {
        // if criteria for a motor failure are met continuously for _detection_time seconds
       failed = true;
    }

    _last_highest_motor = highest_motor_index;

//    static uint32_t log_throttle_count = 0;
//    log_throttle_count++;
//    if (log_throttle_count >= 10 || failed) {
//       log_throttle_count = 0;
//       DataFlash_Class::instance()->Log_Write("FAIL", "TimeUS,TNow,HighOut,AvgOut,Out0,Out1,Out2,Out3,HighMot,CritMet,Failed",
//            "QIffffffIII", AP_HAL::micros64(), tnow_ms,
//            (double)highest_motor, (double)motor_avg, (double)_motor_out_filtered[0], (double)_motor_out_filtered[1],
//            (double)_motor_out_filtered[2], (double)_motor_out_filtered[3],
//            (uint32_t)_last_highest_motor, (uint32_t)motor_fail_criteria_met, (uint32_t)failed);
//    }

    return failed;
}


void AP_MotorsMatrixSlewRateLimited::set_recovery_state(uint32_t tnow_ms, motor_recovery_state new_recovery_state)
{
    _recovery_state = new_recovery_state;
    DataFlash_Class::instance()->Log_Write("RCV1", "TimeUS,TNow,State,FailCount", "QIfH", AP_HAL::micros64(), tnow_ms, (double)new_recovery_state, _failure_count);
}


/**
 * Start a motor recovery
 */
void AP_MotorsMatrixSlewRateLimited::start_motor_recovery(uint32_t tnow_ms)
{
    set_slew_rate(_recovery_slew_rate);

    // Set the last motor output based on the recovery percentage
    // The slew rate limiter will force the actual outputs to be this
    // value +- one slew rate increment during the next loop iteration.
    float recovery_pwm = out_to_pwm(_recovery_output) - _slew_per_iteration;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _motor_out_pwm[i] = recovery_pwm;
        }
    }

    // calculate the total recovery time, in seconds
    float recovery_time_length = (get_pwm_output_max() - out_to_pwm(_recovery_output)) / _recovery_slew_rate;
    // set the state transition time after recovery is complete
    _next_state_transition_time = tnow_ms + (recovery_time_length * 1000.0f);

//    DataFlash_Class::instance()->Log_Write("RCV2", "TimeUS,TNow,SlewRate,RecovPwm,RecovTime,StateTime", "QIfffI",
//            AP_HAL::micros64(), tnow_ms, (double)_recovery_slew_rate, (double)recovery_pwm,
//            (double)recovery_time_length, _next_state_transition_time);

}

/**
 * Get the motor output as a percentage
 */
float AP_MotorsMatrixSlewRateLimited::get_motor_out(uint8_t mot)
{
    return (mot<AP_MOTORS_MAX_NUM_MOTORS) ? pwm_to_out(_motor_out_pwm[mot]) : 0.0f;
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
void AP_MotorsMatrixSlewRateLimited::reset_failure_detector(uint32_t tnow_ms)
{
    _last_motor_good_time = tnow_ms;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _motor_out_filtered[i] = get_motor_out(i);
    }
}

/**
 * Calculate alpha for filter based on parameters and loop rate
 */
void AP_MotorsMatrixSlewRateLimited::set_filter_alpha()
{
    _motor_filt_alpha = constrain_float((2.0f*M_PI*_detection_filter_frequency) /
       ((2.0f*M_PI*_detection_filter_frequency) + _loop_rate),0.0f,1.0f);
}
