// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/*
 *       AP_MotorsMatrix.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <AP_HAL.h>
#include "AP_MotorsMatrix.h"

extern const AP_HAL::HAL& hal;

// Init
void AP_MotorsMatrix::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // setup the motors
    setup_motors();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsMatrix::set_update_rate( uint16_t speed_hz )
{
    int8_t i;

    // record requested speed
    _speed_hz = speed_hz;

    // check each enabled motor
    uint32_t mask = 0;
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
		mask |= 1U << _motor_to_channel_map[i];
        }
    }
    hal.rcout->set_freq( mask, _speed_hz );
}

// set frame orientation (normally + or X)
void AP_MotorsMatrix::set_frame_orientation( uint8_t new_orientation )
{
    // return if nothing has changed
    if( new_orientation == _flags.frame_orientation ) {
        return;
    }

    // call parent
    AP_Motors::set_frame_orientation( new_orientation );

    // setup the motors
    setup_motors();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsMatrix::enable()
{
    int8_t i;

    // enable output channels
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            hal.rcout->enable_ch(_motor_to_channel_map[i]);
        }
    }
}

// output_min - sends minimum values out to the motors
void AP_MotorsMatrix::output_min()
{
    int8_t i;

    // set limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = false;

    // fill the motor_out[] array for HIL use and send minimum value to each motor
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            motor_out[i] = _rc_throttle->radio_min;
            hal.rcout->write(_motor_to_channel_map[i], motor_out[i]);
        }
    }
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix::output_armed()
{
    int8_t i;
    int16_t out_min_pwm = _rc_throttle->radio_min + _min_throttle;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = _rc_throttle->radio_max;                      // maximum pwm value we can send to the motors
    int16_t out_mid_pwm = (out_min_pwm+out_max_pwm)/2;                  // mid pwm value we can send to the motors
    int16_t out_best_thr_pwm;  // the is the best throttle we can come up which provides good control without climbing
    float rpy_scale = 1.0; // this is used to scale the roll, pitch and yaw to fit within the motor limits

    int16_t rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.

    int16_t rpy_low = 0;    // lowest motor value
    int16_t rpy_high = 0;   // highest motor value
    int16_t yaw_allowed;    // amount of yaw we can fit in
    int16_t thr_adj;        // the difference between the pilot's desired throttle and out_best_thr_pwm (the throttle that is actually provided)

    // initialize limits flag
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // Throttle is 0 to 1000 only
    // To-Do: we should not really be limiting this here because we don't "own" this _rc_throttle object
    if (_rc_throttle->servo_out < 0) {
        _rc_throttle->servo_out = 0;
        limit.throttle_lower = true;
    }
    if (_rc_throttle->servo_out > _max_throttle) {
        _rc_throttle->servo_out = _max_throttle;
        limit.throttle_upper = true;
    }

    // capture desired roll, pitch, yaw and throttle from receiver
    _rc_roll->calc_pwm();
    _rc_pitch->calc_pwm();
    _rc_throttle->calc_pwm();
    _rc_yaw->calc_pwm();

    // if we are not sending a throttle output, we cut the motors
    if (_rc_throttle->servo_out == 0) {
        // range check spin_when_armed
        if (_spin_when_armed_ramped < 0) {
             _spin_when_armed_ramped = 0;
        }
        if (_spin_when_armed_ramped > _min_throttle) {
            _spin_when_armed_ramped = _min_throttle;
        }
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            // spin motors at minimum
            if (motor_enabled[i]) {
                motor_out[i] = _rc_throttle->radio_min + _spin_when_armed_ramped;
            }
        }

        // Every thing is limited
        limit.roll_pitch = true;
        limit.yaw = true;
        limit.throttle_lower = true;

    } else {

        // check if throttle is below limit
        if (_rc_throttle->radio_out <= out_min_pwm) {       // perhaps being at min throttle itself is not a problem, only being under is
            limit.throttle_lower = true;
        }

        // calculate roll and pitch for each motor
        // set rpy_low and rpy_high to the lowest and highest values of the motors
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rpy_out[i] = _rc_roll->pwm_out * _roll_factor[i] +
                             _rc_pitch->pwm_out * _pitch_factor[i];

                // record lowest roll pitch command
                if (rpy_out[i] < rpy_low) {
                    rpy_low = rpy_out[i];
                }
                // record highest roll pich command
                if (rpy_out[i] > rpy_high) {
                    rpy_high = rpy_out[i];
                }
            }
        }

        // calculate throttle that gives most possible room for yaw (range 1000 ~ 2000) which is the lower of:
        //      1. mid throttle - average of highest and lowest motor (this would give the maximum possible room margin above the highest motor and below the lowest)
        //      2. the higher of:
        //            a) the pilot's throttle input
        //            b) the mid point between the pilot's input throttle and hover-throttle
        //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
        //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
        //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favour reducing throttle *because* it provides better yaw control)
        //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favour reducing throttle instead of better yaw control because the pilot has commanded it
        int16_t motor_mid = (rpy_low+rpy_high)/2;
        out_best_thr_pwm = min(out_mid_pwm - motor_mid, max(_rc_throttle->radio_out, (_rc_throttle->radio_out+_hover_out)/2));

        // calculate amount of yaw we can fit into the throttle range
        // this is always equal to or less than the requested yaw from the pilot or rate controller
        yaw_allowed = min(out_max_pwm - out_best_thr_pwm, out_best_thr_pwm - out_min_pwm) - (rpy_high-rpy_low)/2;
        yaw_allowed = max(yaw_allowed, AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM);

        if (_rc_yaw->pwm_out >= 0) {
            // if yawing right
            if (yaw_allowed > _rc_yaw->pwm_out) {
                yaw_allowed = _rc_yaw->pwm_out; // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
            }else{
                limit.yaw = true;
            }
        }else{
            // if yawing left
            yaw_allowed = -yaw_allowed;
            if( yaw_allowed < _rc_yaw->pwm_out ) {
                yaw_allowed = _rc_yaw->pwm_out; // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
            }else{
                limit.yaw = true;
            }
        }

        // add yaw to intermediate numbers for each motor
        rpy_low = 0;
        rpy_high = 0;
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rpy_out[i] =    rpy_out[i] +
                                yaw_allowed * _yaw_factor[i];

                // record lowest roll+pitch+yaw command
                if( rpy_out[i] < rpy_low ) {
                    rpy_low = rpy_out[i];
                }
                // record highest roll+pitch+yaw command
                if( rpy_out[i] > rpy_high) {
                    rpy_high = rpy_out[i];
                }
            }
        }

        // check everything fits
        thr_adj = _rc_throttle->radio_out - out_best_thr_pwm;

        // calc upper and lower limits of thr_adj
        int16_t thr_adj_max = out_max_pwm-(out_best_thr_pwm+rpy_high);

        // if we are increasing the throttle (situation #2 above)..
        if (thr_adj > 0) {
            // increase throttle as close as possible to requested throttle
            // without going over out_max_pwm
            if (thr_adj > thr_adj_max){
                thr_adj = thr_adj_max;
                // we haven't even been able to apply full throttle command
                limit.throttle_upper = true;
            }
        }else if(thr_adj < 0){
            // decrease throttle as close as possible to requested throttle
            // without going under out_min_pwm or over out_max_pwm
            // earlier code ensures we can't break both boundaries
            int16_t thr_adj_min = min(out_min_pwm-(out_best_thr_pwm+rpy_low),0);
            if (thr_adj > thr_adj_max) {
                thr_adj = thr_adj_max;
                limit.throttle_upper = true;
            }
            if (thr_adj < thr_adj_min) {
                thr_adj = thr_adj_min;
                limit.throttle_lower = true;
            }
        }

        // do we need to reduce roll, pitch, yaw command
        // earlier code does not allow both limit's to be passed simultainiously with abs(_yaw_factor)<1
        if ((rpy_low+out_best_thr_pwm)+thr_adj < out_min_pwm){
            rpy_scale = (float)(out_min_pwm-thr_adj-out_best_thr_pwm)/rpy_low;
            // we haven't even been able to apply full roll, pitch and minimal yaw without scaling
            limit.roll_pitch = true;
            limit.yaw = true;
        }else if((rpy_high+out_best_thr_pwm)+thr_adj > out_max_pwm){
            rpy_scale = (float)(out_max_pwm-thr_adj-out_best_thr_pwm)/rpy_high;
            // we haven't even been able to apply full roll, pitch and minimal yaw without scaling
            limit.roll_pitch = true;
            limit.yaw = true;
        }

        // add scaled roll, pitch, constrained yaw and throttle for each motor
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = out_best_thr_pwm+thr_adj +
                               rpy_scale*rpy_out[i];
            }
        }

        // adjust for throttle curve
        if (_throttle_curve_enabled) {
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = _throttle_curve.get_y(motor_out[i]);
                }
            }
        }
        // clip motor output if required (shouldn't be)
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = constrain_int16(motor_out[i], out_min_pwm, out_max_pwm);
            }
        }
    }

    // send output to each motor
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            hal.rcout->write(_motor_to_channel_map[i], motor_out[i]);
        }
    }
}

// output_disarmed - sends commands to the motors
void AP_MotorsMatrix::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin each motor for a moment to allow the user to confirm the motor order and spin direction
void AP_MotorsMatrix::output_test()
{
    uint8_t min_order, max_order;
    uint8_t i,j;

    // find min and max orders
    min_order = _test_order[0];
    max_order = _test_order[0];
    for(i=1; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( _test_order[i] < min_order )
            min_order = _test_order[i];
        if( _test_order[i] > max_order )
            max_order = _test_order[i];
    }

    // shut down all motors
    output_min();

    // first delay is longer
    hal.scheduler->delay(4000);

    // loop through all the possible orders spinning any motors that match that description
    for( i=min_order; i<=max_order; i++ ) {
        for( j=0; j<AP_MOTORS_MAX_NUM_MOTORS; j++ ) {
            if( motor_enabled[j] && _test_order[j] == i ) {
                // turn on this motor and wait 1/3rd of a second
                hal.rcout->write(_motor_to_channel_map[j], _rc_throttle->radio_min + _min_throttle);
                hal.scheduler->delay(300);
                hal.rcout->write(_motor_to_channel_map[j], _rc_throttle->radio_min);
                hal.scheduler->delay(2000);
            }
        }
    }

    // shut down all motors
    output_min();
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // increment number of motors if this motor is being newly motor_enabled
        if( !motor_enabled[motor_num] ) {
            motor_enabled[motor_num] = true;
            _num_motors++;
        }

        // set roll, pitch, thottle factors and opposite motor (for stability patch)
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;
    }
}

// add_motor using just position and prop direction
void AP_MotorsMatrix::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    // call raw motor set-up method
    add_motor_raw(
        motor_num,
        cosf(radians(angle_degrees + 90)),               // roll factor
        cosf(radians(angle_degrees)),                    // pitch factor
        yaw_factor,                                      // yaw factor
        testing_order);

}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // if the motor was enabled decrement the number of motors
        if( motor_enabled[motor_num] )
            _num_motors--;

        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _roll_factor[motor_num] = 0;
        _pitch_factor[motor_num] = 0;
        _yaw_factor[motor_num] = 0;
    }
}

// remove_all_motors - removes all motor definitions
void AP_MotorsMatrix::remove_all_motors()
{
    for( int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        remove_motor(i);
    }
    _num_motors = 0;
}
