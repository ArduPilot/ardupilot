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
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrix.h"

extern const AP_HAL::HAL& hal;

// Init
void AP_MotorsMatrix::Init()
{
    // setup the motors
    setup_motors();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsMatrix::set_update_rate( uint16_t speed_hz )
{
    uint8_t i;

    // record requested speed
    _speed_hz = speed_hz;

    // check each enabled motor
    uint32_t mask = 0;
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
		mask |= 1U << i;
        }
    }
    rc_set_freq( mask, _speed_hz );
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
            rc_enable_ch(i);
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
    hal.rcout->cork();
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_write(i, _throttle_radio_min);
        }
    }
    hal.rcout->push();
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsMatrix::get_motor_mask()
{
    uint16_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            mask |= 1U << i;
        }
    }
    return rc_map_mask(mask);
}

void AP_MotorsMatrix::output_armed_not_stabilizing()
{
    uint8_t i;
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];                    // final outputs sent to the motors
    int16_t out_min_pwm = _throttle_radio_min + _min_throttle;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = _throttle_radio_max;                      // maximum pwm value we can send to the motors

    // initialize limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    int16_t thr_in_min = rel_pwm_to_thr_range(_spin_when_armed_ramped);
    if (_throttle_control_input <= thr_in_min) {
        _throttle_control_input = thr_in_min;
        limit.throttle_lower = true;
    }

    if (_throttle_control_input >= _hover_out) {
        _throttle_control_input = _hover_out;
        limit.throttle_upper = true;
    }

    throttle_radio_output = calc_throttle_radio_output();

    // set output throttle
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_out[i] = throttle_radio_output;
        }
    }

    if(throttle_radio_output >= out_min_pwm) {
        // apply thrust curve and voltage scaling
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = apply_thrust_curve_and_volt_scaling(motor_out[i], out_min_pwm, out_max_pwm);
            }
        }
    }

    // send output to each motor
    hal.rcout->cork();
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
void AP_MotorsMatrix::output_armed_stabilizing()
{
    int8_t i;
    int16_t roll_pwm;                                               // roll pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t pitch_pwm;                                              // pitch pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t yaw_pwm;                                                // yaw pwm value, initially calculated by calc_yaw_pwm() but may be modified after, +/- 400
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min_pwm = _throttle_radio_min + _min_throttle;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = _throttle_radio_max;                      // maximum pwm value we can send to the motors
    int16_t out_mid_pwm = (out_min_pwm+out_max_pwm)/2;              // mid pwm value we can send to the motors
    int16_t out_best_thr_pwm;                                       // the is the best throttle we can come up which provides good control without climbing
    float rpy_scale = 1.0;                                          // this is used to scale the roll, pitch and yaw to fit within the motor limits

    int16_t rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors

    int16_t rpy_low = 0;    // lowest motor value
    int16_t rpy_high = 0;   // highest motor value
    int16_t yaw_allowed;    // amount of yaw we can fit in
    int16_t thr_adj;        // the difference between the pilot's desired throttle and out_best_thr_pwm (the throttle that is actually provided)

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // Ensure throttle is within bounds of 0 to 1000
    int16_t thr_in_min = rel_pwm_to_thr_range(_min_throttle);
    if (_throttle_control_input <= thr_in_min) {
        _throttle_control_input = thr_in_min;
        limit.throttle_lower = true;
    }
    if (_throttle_control_input >= _max_throttle) {
        _throttle_control_input = _max_throttle;
        limit.throttle_upper = true;
    }

    roll_pwm = calc_roll_pwm();
    pitch_pwm = calc_pitch_pwm();
    yaw_pwm = calc_yaw_pwm();
    throttle_radio_output = calc_throttle_radio_output();

    // calculate roll and pitch for each motor
    // set rpy_low and rpy_high to the lowest and highest values of the motors
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpy_out[i] = roll_pwm * _roll_factor[i] * get_compensation_gain() +
                            pitch_pwm * _pitch_factor[i] * get_compensation_gain();

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
    out_best_thr_pwm = MIN(out_mid_pwm - motor_mid, MAX(throttle_radio_output, throttle_radio_output*MAX(0,1.0f-_throttle_thr_mix)+get_hover_throttle_as_pwm()*_throttle_thr_mix));

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller
    yaw_allowed = MIN(out_max_pwm - out_best_thr_pwm, out_best_thr_pwm - out_min_pwm) - (rpy_high-rpy_low)/2;
    yaw_allowed = MAX(yaw_allowed, _yaw_headroom);

    if (yaw_pwm >= 0) {
        // if yawing right
        if (yaw_allowed > yaw_pwm * get_compensation_gain()) {
            yaw_allowed = yaw_pwm * get_compensation_gain(); // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
        }else{
            limit.yaw = true;
        }
    }else{
        // if yawing left
        yaw_allowed = -yaw_allowed;
        if (yaw_allowed < yaw_pwm * get_compensation_gain()) {
            yaw_allowed = yaw_pwm * get_compensation_gain(); // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
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
    thr_adj = throttle_radio_output - out_best_thr_pwm;

    // calculate upper and lower limits of thr_adj
    int16_t thr_adj_max = MAX(out_max_pwm-(out_best_thr_pwm+rpy_high),0);

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
        int16_t thr_adj_min = MIN(out_min_pwm-(out_best_thr_pwm+rpy_low),0);
        if (thr_adj > thr_adj_max) {
            thr_adj = thr_adj_max;
            limit.throttle_upper = true;
        }
        if (thr_adj < thr_adj_min) {
            thr_adj = thr_adj_min;
        }
    }

    // do we need to reduce roll, pitch, yaw command
    // earlier code does not allow both limit's to be passed simultaneously with abs(_yaw_factor)<1
    if ((rpy_low+out_best_thr_pwm)+thr_adj < out_min_pwm){
        // protect against divide by zero
        if (rpy_low != 0) {
            rpy_scale = (float)(out_min_pwm-thr_adj-out_best_thr_pwm)/rpy_low;
        }
        // we haven't even been able to apply full roll, pitch and minimal yaw without scaling
        limit.roll_pitch = true;
        limit.yaw = true;
    }else if((rpy_high+out_best_thr_pwm)+thr_adj > out_max_pwm){
        // protect against divide by zero
        if (rpy_high != 0) {
            rpy_scale = (float)(out_max_pwm-thr_adj-out_best_thr_pwm)/rpy_high;
        }
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

    // apply thrust curve and voltage scaling
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_out[i] = apply_thrust_curve_and_volt_scaling(motor_out[i], out_min_pwm, out_max_pwm);
        }
    }

    // clip motor output if required (shouldn't be)
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_out[i] = constrain_int16(motor_out[i], out_min_pwm, out_max_pwm);
        }
    }

    // send output to each motor
    hal.rcout->cork();
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}

// output_disarmed - sends commands to the motors
void AP_MotorsMatrix::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsMatrix::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // loop through all the possible orders spinning any motors that match that description
    hal.rcout->cork();
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _test_order[i] == motor_seq) {
            // turn on this motor
            rc_write(i, pwm);
        }
    }
    hal.rcout->push();
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // increment number of motors if this motor is being newly motor_enabled
        if( !motor_enabled[motor_num] ) {
            motor_enabled[motor_num] = true;
        }

        // set roll, pitch, thottle factors and opposite motor (for stability patch)
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        uint8_t chan;
        if (RC_Channel_aux::find_channel((RC_Channel_aux::Aux_servo_function_t)(RC_Channel_aux::k_motor1+motor_num),
                                         chan)) {
            _motor_map[motor_num] = chan;
            _motor_map_mask |= 1U<<motor_num;
        } else {
            // disable this channel from being used by RC_Channel_aux
            RC_Channel_aux::disable_aux_channel(motor_num);
        }
    }
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void AP_MotorsMatrix::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {
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
}
