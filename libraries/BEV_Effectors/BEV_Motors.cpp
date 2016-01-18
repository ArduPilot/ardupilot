/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL.h>
#include "BEV_Motors.h"

extern const AP_HAL::HAL& hal;
//
// public methods
void BEV_Motors::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    //BEV hardcoding in the FireFLY6 motor arrangement
    add_motor_raw(AP_MOTORS_MOT_3,  0.0, -1.000, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1); //aft top (ccw)
    add_motor_raw(AP_MOTORS_MOT_4, -1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2); //front right top (ccw)
    add_motor_raw(AP_MOTORS_MOT_5,  1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3); //front left top (ccw)
    add_motor_raw(AP_MOTORS_MOT_6,  0.0, -1.000, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4); //aft bottom (cw)
    add_motor_raw(AP_MOTORS_MOT_7, -1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5); //front right bottom (cw)
    add_motor_raw(AP_MOTORS_MOT_8,  1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6); //front left bottom (cw)

    set_update_rate(_speed_hz);
}

void BEV_Motors::set_update_rate(uint16_t speed_hz)
{
    uint32_t mask = 0;
    // record requested speed
    _speed_hz = speed_hz;

    //set update rate for bus 1, outputs 1 and 2 (elevons) to be low.
    //the analog servos can't handle update rates > 100 hz
    mask = 0x00000003; //mask for pins 1 and 2
    hal.rcout->set_freq(mask, 50);

    //update rate for bus 2, outputs 3 and 4 (motors)
    mask = 0x0000000C; //mask for pins 3 and 4
    hal.rcout->set_freq(mask, _speed_hz);

    //update rate for bus 3, outputs 5-8 (motors)
    mask = 0x000000F0; //mask for pins 5-8
    hal.rcout->set_freq(mask, _speed_hz);
}

// output_armed - sends commands to the motors
#define is_transition_motor(motor_number) ( (motor_number == 3) || (motor_number == 4) || (motor_number == 6) || (motor_number == 7) )
void BEV_Motors::output_armed()
{
    int8_t i;
    int16_t out_min_pwm = _rc_throttle.radio_min + _min_throttle;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = _rc_throttle.radio_max;                      // maximum pwm value we can send to the motors
    int16_t out_mid_pwm = (out_min_pwm+out_max_pwm)/2;                  // mid pwm value we can send to the motors
    int16_t out_best_thr_pwm;  // the is the best throttle we can come up which provides good control without climbing
    float rpy_scale = 1.0; // this is used to scale the roll, pitch and yaw to fit within the motor limits

    int16_t rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    //BEV added for transition scalaing
    float trans_adj_roll_factor[AP_MOTORS_MAX_NUM_MOTORS];
    float trans_adj_yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];

    int16_t rpy_low = 0;    // lowest motor value
    int16_t rpy_high = 0;   // highest motor value
    int16_t yaw_allowed;    // amount of yaw we can fit in
    int16_t thr_adj;        // the difference between the pilot's desired throttle and out_best_thr_pwm (the throttle that is actually provided)

    //BEV the roll and yaw adjustment factors for the transitioning motors
    float trans_native_factor = cosf(radians(_transition_angle_deg));
    float trans_new_factor = sinf(radians(_transition_angle_deg));

    //BEV rotate roll and yaw factors on transition motors when transitioning
    if(_transition_angle_deg>1) {
        for(i=0;i<AP_MOTORS_MAX_NUM_MOTORS;i++) {
            if(motor_enabled[i]) {
                if(is_transition_motor(i)) {
                    trans_adj_roll_factor[i] = trans_native_factor*_roll_factor[i] + trans_new_factor*_yaw_factor[i];
                    trans_adj_yaw_factor[i] = trans_native_factor*_yaw_factor[i] + trans_new_factor*_roll_factor[i];
                } else {
                    trans_adj_roll_factor[i] = _roll_factor[i];
                    trans_adj_yaw_factor[i] = _yaw_factor[i];
                }
            }
        }
    } else {
        for(i=0;i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if(motor_enabled[i]) {
                trans_adj_roll_factor[i] = _roll_factor[i];
                trans_adj_yaw_factor[i] = _yaw_factor[i];
            }
        }
    }

    // initialize limits flag
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // Throttle is 0 to 1000 only
    // To-Do: we should not really be limiting this here because we don't "own" this _rc_throttle object
    if (_rc_throttle.servo_out <= 0) {
        _rc_throttle.servo_out = 0;
        limit.throttle_lower = true;
    }
    if (_rc_throttle.servo_out >= _max_throttle) {
        _rc_throttle.servo_out = _max_throttle;
        limit.throttle_upper = true;
    }

    // capture desired roll, pitch, yaw and throttle from receiver
    _rc_roll.calc_pwm();
    _rc_pitch.calc_pwm();
    _rc_throttle.calc_pwm();
    _rc_yaw.calc_pwm();

    // if we are not sending a throttle output, we cut the motors
    if (_rc_throttle.servo_out == 0) {
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
                _motor_out[i] = _rc_throttle.radio_min + _spin_when_armed_ramped;
            }
        }

        // Every thing is limited
        limit.roll_pitch = true;
        limit.yaw = true;

    } else {

        // check if throttle is below limit
        if (_rc_throttle.servo_out <= _min_throttle) {  // perhaps being at min throttle itself is not a problem, only being under is
            limit.throttle_lower = true;
        }

        // calculate roll and pitch for each motor
        // set rpy_low and rpy_high to the lowest and highest values of the motors
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rpy_out[i] = _rc_roll.pwm_out * trans_adj_roll_factor[i] +
                             _rc_pitch.pwm_out * _pitch_factor[i];

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
        out_best_thr_pwm = min(out_mid_pwm - motor_mid, max(_rc_throttle.radio_out, (_rc_throttle.radio_out+_hover_out)/2));

        // calculate amount of yaw we can fit into the throttle range
        // this is always equal to or less than the requested yaw from the pilot or rate controller
        yaw_allowed = min(out_max_pwm - out_best_thr_pwm, out_best_thr_pwm - out_min_pwm) - (rpy_high-rpy_low)/2;
        yaw_allowed = max(yaw_allowed, AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM);

        if (_rc_yaw.pwm_out >= 0) {
            // if yawing right
            if (yaw_allowed > _rc_yaw.pwm_out) {
                yaw_allowed = _rc_yaw.pwm_out; // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
            }else{
                limit.yaw = true;
            }
        }else{
            // if yawing left
            yaw_allowed = -yaw_allowed;
            if( yaw_allowed < _rc_yaw.pwm_out ) {
                yaw_allowed = _rc_yaw.pwm_out; // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
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
                                yaw_allowed * trans_adj_yaw_factor[i];

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
        thr_adj = _rc_throttle.radio_out - out_best_thr_pwm;

        // calc upper and lower limits of thr_adj
        int16_t thr_adj_max = max(out_max_pwm-(out_best_thr_pwm+rpy_high),0);

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
                _motor_out[i] = out_best_thr_pwm+thr_adj +
                               rpy_scale*rpy_out[i];
            }
        }
        // adjust for throttle curve
        if (_throttle_curve_enabled) {
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    _motor_out[i] = _throttle_curve.get_y(_motor_out[i]);
                }
            }
        }

        // clip motor output if required (shouldn't be)
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                _motor_out[i] = constrain_int16(_motor_out[i], out_min_pwm, out_max_pwm);
            }
        }
    }
}

void BEV_Motors::output_min()
{
    int8_t i;

    // set limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = false;

    // fill the motor_out[] array for use by BEF_Effectors master class
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        _motor_out[i] = _rc_throttle.radio_min;
    }
}
