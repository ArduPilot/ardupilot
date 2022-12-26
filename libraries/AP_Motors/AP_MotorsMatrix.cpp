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

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrix.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsMatrix::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // record requested frame class and type
    _active_frame_class = frame_class;
    _active_frame_type = frame_type;

    if (frame_class == MOTOR_FRAME_SCRIPTING_MATRIX) {
        // if Scripting frame class, do nothing scripting must call its own dedicated init function
        return;
    }

    // setup the motors
    setup_motors(frame_class, frame_type);

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

#if AP_SCRIPTING_ENABLED
// dedicated init for lua scripting
bool AP_MotorsMatrix::init(uint8_t expected_num_motors)
{
    if (_active_frame_class != MOTOR_FRAME_SCRIPTING_MATRIX) {
        // not the correct class
        return false;
    }

    // Make sure the correct number of motors have been added
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    set_initialised_ok(expected_num_motors == num_motors);

    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    switch (num_motors) {
        case 3:
            _mav_type = MAV_TYPE_TRICOPTER;
            break;
        case 4:
            _mav_type = MAV_TYPE_QUADROTOR;
            break;
        case 6:
            _mav_type = MAV_TYPE_HEXAROTOR;
            break;
        case 8:
            _mav_type = MAV_TYPE_OCTOROTOR;
            break;
        case 10:
            _mav_type = MAV_TYPE_DECAROTOR;
            break;
        case 12:
            _mav_type = MAV_TYPE_DODECAROTOR;
            break;
        default:
            _mav_type = MAV_TYPE_GENERIC;
    }

    normalise_rpy_factors();

    set_update_rate(_speed_hz);

    return true;
}

// Set throttle factor from scripting
bool AP_MotorsMatrix::set_throttle_factor(int8_t motor_num, float throttle_factor)
{
    if ((_active_frame_class != MOTOR_FRAME_SCRIPTING_MATRIX) ) {
        // not the correct class
        return false;
    }

    if (initialised_ok() || !motor_enabled[motor_num]) {
        // Already setup or given motor is not enabled
        return false;
    }

    _throttle_factor[motor_num] = throttle_factor;
    return true;
}

#endif // AP_SCRIPTING_ENABLED

// set update rate to motors - a value in hertz
void AP_MotorsMatrix::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    uint32_t mask = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            mask |= 1U << i;
        }
    }
    rc_set_freq(mask, _speed_hz);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsMatrix::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // exit immediately if armed or no change
    if (armed() || (frame_class == _active_frame_class && _active_frame_type == frame_type)) {
        return;
    }
    _active_frame_class = frame_class;
    _active_frame_type = frame_type;

    init(frame_class, frame_type);

}

void AP_MotorsMatrix::output_to_motors()
{
    int8_t i;

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: {
            // no output
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    _actuator[i] = 0.0f;
                }
            }
            break;
        }
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
                }
            }
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], thrust_to_actuator(_thrust_rpyt_out[i]));
                }
            }
            break;
    }

    // convert output to PWM and send to each motor
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, output_to_pwm(_actuator[i]));
        }
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsMatrix::get_motor_mask()
{
    uint32_t motor_mask = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_mask |= 1U << i;
        }
    }
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix::output_armed_stabilizing()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_avg_max;           // throttle thrust average maximum value, 0.0 - 1.0
    float   throttle_thrust_max;        // throttle thrust maximum value, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain(); // compensation for battery voltage and altitude
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;
    throttle_avg_max = _throttle_avg_max * compensation_gain;

    // If thrust boost is active then do not limit maximum thrust
    throttle_thrust_max = _thrust_boost_ratio + (1.0f - _thrust_boost_ratio) * _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= throttle_thrust_max) {
        throttle_thrust = throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // ensure that throttle_avg_max is between the input throttle and the maximum throttle
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, throttle_thrust_max);

    // calculate the highest allowed average thrust that will provide maximum control range
    throttle_thrust_best_rpy = MIN(0.5f, throttle_avg_max);

    // calculate throttle that gives most possible room for yaw which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // Under the motor lost condition we remove the highest motor output from our calculations and let that motor go greater than 1.0
    // To ensure control and maximum righting performance Hex and Octo have some optimal settings that should be used
    // Y6               : MOT_YAW_HEADROOM = 350, ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.375, ATC_RAT_PIT_IMAX = 0.375, ATC_RAT_YAW_IMAX = 0.375
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.75,  ATC_RAT_PIT_IMAX = 0.75,  ATC_RAT_YAW_IMAX = 0.375
    // Usable minimums below may result in attitude offsets when motors are lost. Hex aircraft are only marginal and must be handles with care
    // Hex              : MOT_YAW_HEADROOM = 0,   ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.25,  ATC_RAT_PIT_IMAX = 0.25,  ATC_RAT_YAW_IMAX = 0.25
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.5,   ATC_RAT_PIT_IMAX = 0.5,   ATC_RAT_YAW_IMAX = 0.25
    // Quads cannot make use of motor loss handling because it doesn't have enough degrees of freedom.

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller
    float rp_low = 1.0f;    // lowest thrust value
    float rp_high = -1.0f;  // highest thrust value
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // calculate the thrust outputs for roll and pitch
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
            // record lowest roll + pitch command
            if (_thrust_rpyt_out[i] < rp_low) {
                rp_low = _thrust_rpyt_out[i];
            }
            // record highest roll + pitch command
            if (_thrust_rpyt_out[i] > rp_high && (!_thrust_boost || i != _motor_lost_index)) {
                rp_high = _thrust_rpyt_out[i];
            }

            // Check the maximum yaw control that can be used on this channel
            // Exclude any lost motors if thrust boost is enabled
            if (!is_zero(_yaw_factor[i]) && (!_thrust_boost || i != _motor_lost_index)){
                if (is_positive(yaw_thrust * _yaw_factor[i])) {
                    yaw_allowed = MIN(yaw_allowed, fabsf(MAX(1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]), 0.0f)/_yaw_factor[i]));
                } else {
                    yaw_allowed = MIN(yaw_allowed, fabsf(MAX(throttle_thrust_best_rpy + _thrust_rpyt_out[i], 0.0f)/_yaw_factor[i]));
                }
            }
        }
    }

    // calculate the maximum yaw control that can be used
    // todo: make _yaw_headroom 0 to 1
    float yaw_allowed_min = (float)_yaw_headroom * 0.001f;

    // increase yaw headroom to 50% if thrust boost enabled
    yaw_allowed_min = _thrust_boost_ratio * 0.5f + (1.0f - _thrust_boost_ratio) * yaw_allowed_min;

    // Let yaw access minimum amount of head room
    yaw_allowed = MAX(yaw_allowed, yaw_allowed_min);

    // Include the lost motor scaled by _thrust_boost_ratio to smoothly transition this motor in and out of the calculation
    if (_thrust_boost && motor_enabled[_motor_lost_index]) {
        // record highest roll + pitch command
        if (_thrust_rpyt_out[_motor_lost_index] > rp_high) {
            rp_high = _thrust_boost_ratio * rp_high + (1.0f - _thrust_boost_ratio) * _thrust_rpyt_out[_motor_lost_index];
        }

        // Check the maximum yaw control that can be used on this channel
        // Exclude any lost motors if thrust boost is enabled
        if (!is_zero(_yaw_factor[_motor_lost_index])){
            if (is_positive(yaw_thrust * _yaw_factor[_motor_lost_index])) {
                yaw_allowed = _thrust_boost_ratio * yaw_allowed + (1.0f - _thrust_boost_ratio) * MIN(yaw_allowed, fabsf(MAX(1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[_motor_lost_index]), 0.0f)/_yaw_factor[_motor_lost_index]));
            } else {
                yaw_allowed = _thrust_boost_ratio * yaw_allowed + (1.0f - _thrust_boost_ratio) * MIN(yaw_allowed, fabsf(MAX(throttle_thrust_best_rpy + _thrust_rpyt_out[_motor_lost_index], 0.0f)/_yaw_factor[_motor_lost_index]));
            }
        }
    }

    if (fabsf(yaw_thrust) > yaw_allowed) {
        // not all commanded yaw can be used
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // add yaw control to thrust outputs
    float rpy_low = 1.0f;   // lowest thrust value
    float rpy_high = -1.0f; // highest thrust value
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

            // record lowest roll + pitch + yaw command
            if (_thrust_rpyt_out[i] < rpy_low) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll + pitch + yaw command
            // Exclude any lost motors if thrust boost is enabled
            if (_thrust_rpyt_out[i] > rpy_high && (!_thrust_boost || i != _motor_lost_index)) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }
    // Include the lost motor scaled by _thrust_boost_ratio to smoothly transition this motor in and out of the calculation
    if (_thrust_boost) {
        // record highest roll + pitch + yaw command
        if (_thrust_rpyt_out[_motor_lost_index] > rpy_high && motor_enabled[_motor_lost_index]) {
            rpy_high = _thrust_boost_ratio * rpy_high + (1.0f - _thrust_boost_ratio) * _thrust_rpyt_out[_motor_lost_index];
        }
    }

    // calculate any scaling needed to make the combined thrust outputs fit within the output range
    if (rpy_high - rpy_low > 1.0f) {
        rpy_scale = 1.0f / (rpy_high - rpy_low);
    }
    if (throttle_avg_max + rpy_low < 0) {
        rpy_scale = MIN(rpy_scale, -throttle_avg_max / rpy_low);
    }

    // calculate how close the motors can come to the desired throttle
    rpy_high *= rpy_scale;
    rpy_low *= rpy_scale;
    throttle_thrust_best_rpy = -rpy_low;
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f) {
        // Full range is being used by roll, pitch, and yaw.
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else {
        if (thr_adj < 0.0f) {
            // Throttle can't be reduced to desired value
            // todo: add lower limit flag and ensure it is handled correctly in altitude controller
            thr_adj = 0.0f;
        } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy + rpy_high)) {
            // Throttle can't be increased to desired value
            thr_adj = 1.0f - (throttle_thrust_best_rpy + rpy_high);
            limit.throttle_upper = true;
        }
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    const float throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = (throttle_thrust_best_plus_adj * _throttle_factor[i]) + (rpy_scale * _thrust_rpyt_out[i]);
        }
    }

    // determine throttle thrust for harmonic notch
    // compensation_gain can never be zero
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

    // check for failed motor
    check_for_failed_motor(throttle_thrust_best_plus_adj);
}

// check for failed motor
//   should be run immediately after output_armed_stabilizing
//   first argument is the sum of:
//      a) throttle_thrust_best_rpy : throttle level (from 0 to 1) providing maximum roll, pitch and yaw range without climbing
//      b) thr_adj: the difference between the pilot's desired throttle and throttle_thrust_best_rpy
//   records filtered motor output values in _thrust_rpyt_out_filt array
//   sets thrust_balanced to true if motors are balanced, false if a motor failure is detected
//   sets _motor_lost_index to index of failed motor
void AP_MotorsMatrix::check_for_failed_motor(float throttle_thrust_best_plus_adj)
{
    // record filtered and scaled thrust output for motor loss monitoring purposes
    float alpha = _dt / (_dt + 0.5f);
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out_filt[i] += alpha * (_thrust_rpyt_out[i] - _thrust_rpyt_out_filt[i]);
        }
    }

    float rpyt_high = 0.0f;
    float rpyt_sum = 0.0f;
    uint8_t number_motors = 0.0f;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            number_motors += 1;
            rpyt_sum += _thrust_rpyt_out_filt[i];
            // record highest filtered thrust command
            if (_thrust_rpyt_out_filt[i] > rpyt_high) {
                rpyt_high = _thrust_rpyt_out_filt[i];
                // hold motor lost index constant while thrust boost is active
                if (!_thrust_boost) {
                    _motor_lost_index = i;
                }
            }
        }
    }

    float thrust_balance = 1.0f;
    if (rpyt_sum > 0.1f) {
        thrust_balance = rpyt_high * number_motors / rpyt_sum;
    }
    // ensure thrust balance does not activate for multirotors with less than 6 motors
    if (number_motors >= 6 && thrust_balance >= 1.5f && _thrust_balanced) {
        _thrust_balanced = false;
    }
    if (thrust_balance <= 1.25f && !_thrust_balanced) {
        _thrust_balanced = true;
    }

    // check to see if thrust boost is using more throttle than _throttle_thrust_max
    if ((_throttle_thrust_max * get_compensation_gain() > throttle_thrust_best_plus_adj) && (rpyt_high < 0.9f) && _thrust_balanced) {
        _thrust_boost = false;
    }
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsMatrix::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // loop through all the possible orders spinning any motors that match that description
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _test_order[i] == motor_seq) {
            // turn on this motor
            rc_write(i, pwm);
        }
    }
}

// output_test_num - spin a motor connected to the specified output channel
//  (should only be performed during testing)
//  If a motor output channel is remapped, the mapped channel is used.
//  Returns true if motor output is set, false otherwise
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
bool AP_MotorsMatrix::output_test_num(uint8_t output_channel, int16_t pwm)
{
    if (!armed()) {
        return false;
    }

    // Is channel in supported range?
    if (output_channel > AP_MOTORS_MAX_NUM_MOTORS - 1) {
        return false;
    }

    // Is motor enabled?
    if (!motor_enabled[output_channel]) {
        return false;
    }

    rc_write(output_channel, pwm); // output
    return true;
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order, float throttle_factor)
{
    if (initialised_ok()) {
        // do not allow motors to be set if the current frame type has init correctly
        return;
    }

    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {

        // enable motor
        motor_enabled[motor_num] = true;

        // set roll, pitch, yaw and throttle factors
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;
        _throttle_factor[motor_num] = throttle_factor;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // call parent class method
        add_motor_num(motor_num);
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
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _roll_factor[motor_num] = 0.0f;
        _pitch_factor[motor_num] = 0.0f;
        _yaw_factor[motor_num] = 0.0f;
        _throttle_factor[motor_num] = 0.0f;
    }
}

void AP_MotorsMatrix::add_motors(const struct MotorDef *motors, uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        const auto &motor = motors[i];
        add_motor(i, motor.angle_degrees, motor.yaw_factor, motor.testing_order);
    }
}
void AP_MotorsMatrix::add_motors_raw(const struct MotorDefRaw *motors, uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        const auto &m = motors[i];
        add_motor_raw(i, m.roll_fac, m.pitch_fac, m.yaw_fac, m.testing_order);
    }
}
#if AP_MOTORS_FRAME_QUAD_ENABLED
bool AP_MotorsMatrix::setup_quad_matrix(motor_frame_type frame_type)
{
    _frame_class_string = "QUAD";
    _mav_type = MAV_TYPE_QUADROTOR;
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            {   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    case MOTOR_FRAME_TYPE_NYT_PLUS: {
        _frame_type_string = "NYT_PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  90, 0,  2 },
            { -90, 0,  4 },
            {   0, 0,  1 },
            { 180, 0,  3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_NYT_X: {
        _frame_type_string = "NYT_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, 0,  1 },
            { -135, 0,  3 },
            {  -45, 0,  4 },
            {  135, 0,  2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
#endif //APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    case MOTOR_FRAME_TYPE_BF_X: {
        // betaflight quad X order
        // see: https://fpvfrenzy.com/betaflight-motor-order/
        _frame_type_string = "BF_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_BF_X_REV: {
        // betaflight quad X order, reversed motors
        _frame_type_string = "X_REV";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_DJI_X: {
        // DJI quad X order
        // see https://forum44.djicdn.com/data/attachment/forum/201711/26/172348bppvtt1ot1nrtp5j.jpg
        _frame_type_string = "DJI_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_CW_X: {
        // "clockwise X" motor order. Motors are ordered clockwise from front right
        // matching test order
        _frame_type_string = "CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_V: {
        _frame_type_string = "V";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45,   0.7981f,   1 },
            { -135,   1.0000f,   3 },
            {  -45,  -0.7981f,   4 },
            {  135,  -1.0000f,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_H: {
        // H frame set-up - same as X but motors spin in opposite directiSons
        _frame_type_string = "H";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_VTAIL: {
        /*
            Tested with: Lynxmotion Hunter Vtail 400
            - inverted rear outward blowing motors (at a 40 degree angle)
            - should also work with non-inverted rear outward blowing motors
            - no roll in rear motors
            - no yaw in front motors
            - should fly like some mix between a tricopter and X Quadcopter

            Roll control comes only from the front motors, Yaw control only from the rear motors.
            Roll & Pitch factor is measured by the angle away from the top of the forward axis to each arm.

            Note: if we want the front motors to help with yaw,
                motors 1's yaw factor should be changed to sin(radians(40)).  Where "40" is the vtail angle
                motors 3's yaw factor should be changed to -sin(radians(40))
        */
        _frame_type_string = "VTAIL";
        add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
        add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
        add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
        add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        break;
    }
    case MOTOR_FRAME_TYPE_ATAIL:
        /*
            The A-Shaped VTail is the exact same as a V-Shaped VTail, with one difference:
            - The Yaw factors are reversed, because the rear motors are facing different directions

            With V-Shaped VTails, the props make a V-Shape when spinning, but with
            A-Shaped VTails, the props make an A-Shape when spinning.
            - Rear thrust on a V-Shaped V-Tail Quad is outward
            - Rear thrust on an A-Shaped V-Tail Quad is inward

            Still functions the same as the V-Shaped VTail mixing below:
            - Yaw control is entirely in the rear motors
            - Roll is is entirely in the front motors
        */
        _frame_type_string = "ATAIL";
        add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
        add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
        add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
        break;
    case MOTOR_FRAME_TYPE_PLUSREV: {
        // plus with reversed motor directions
        _frame_type_string = "PLUSREV";
        static const AP_MotorsMatrix::MotorDef motors[] {
            { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
            { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_Y4:
        _frame_type_string = "Y4";
        // Y4 motor definition with right front CCW, left front CW
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f,  1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            {  1.0f,  1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    default:
        // quad frame class does not support this frame type
        _frame_type_string = "UNSUPPORTED";
        return false;
    }
    return true;
}
#endif //AP_MOTORS_FRAME_QUAD_ENABLED
#if AP_MOTORS_FRAME_HEXA_ENABLED
bool AP_MotorsMatrix::setup_hexa_matrix(motor_frame_type frame_type)
{
    _frame_class_string = "HEXA";
    _mav_type = MAV_TYPE_HEXAROTOR;
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            { -120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            {   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6 },
            {  120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_H: {
        // H is same as X except middle motors are closer to center
        _frame_type_string = "H";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f, 0.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
            { 1.0f, 0.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
            { 1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
            { -1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            { -1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            { 1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_DJI_X: {
        _frame_type_string = "DJI_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_CW_X: {
        _frame_type_string = "CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // hexa frame class does not support this frame type
        _frame_type_string = "UNSUPPORTED";
        return false;
    } //hexa
    return true;
}
#endif ////AP_MOTORS_FRAME_HEXA_ENABLED
#if AP_MOTORS_FRAME_OCTA_ENABLED
bool AP_MotorsMatrix::setup_octa_matrix(motor_frame_type frame_type)
{
    _frame_class_string = "OCTA";
    _mav_type = MAV_TYPE_OCTOROTOR;
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
        };

        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            { -157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            {   67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {  157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            {  -22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
            { -112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6 },
            {  -67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            {  112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_V: {
        _frame_type_string = "V";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            {  0.83f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 },
            { -0.67f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
            {  0.67f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            { -0.50f, -1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {  1.00f,  1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
            { -0.83f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            { -1.00f,  1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            {  0.50f, -1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_H: {
        _frame_type_string = "H";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            {  1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
            { -1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            { -1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {  1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
            {  1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            {  1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 },
            { -1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_I: {
        _frame_type_string = "I";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { 0.333f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            { -0.333f,  1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            {    1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            {  0.333f,  1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
            { -0.333f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {   -1.0f,  1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            {   -1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
            {    1.0f,  1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_DJI_X: {
        _frame_type_string = "DJI_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  -22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8 },
            {  -67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7 },
            { -112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            { -157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            {  112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {   67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_CW_X: {
        _frame_type_string = "CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {   67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            { -157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            { -112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            {  -67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7 },
            {  -22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // octa frame class does not support this frame type
        _frame_type_string = "UNSUPPORTED";
        return false;
    } // octa frame type
    return true;
}
#endif //AP_MOTORS_FRAME_OCTA_ENABLED
#if AP_MOTORS_FRAME_OCTAQUAD_ENABLED
bool AP_MotorsMatrix::setup_octaquad_matrix(motor_frame_type frame_type)
{
    _mav_type = MAV_TYPE_OCTOROTOR;
    _frame_class_string = "OCTAQUAD";
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
            {   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_V: {
        _frame_type_string = "V";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45,   0.7981f,  1 },
            {  -45,  -0.7981f,  7 },
            { -135,   1.0000f,  5 },
            {  135,  -1.0000f,  3 },
            {  -45,   0.7981f,  8 },
            {   45,  -0.7981f,  2 },
            {  135,   1.0000f,  4 },
            { -135,  -1.0000f,  6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_H: {
        // H frame set-up - same as X but motors spin in opposite directions
        _frame_type_string = "H";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_CW_X: {
        _frame_type_string = "CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    // BF/X cinelifters using two 4-in-1 ESCs are quite common
    // see: https://fpvfrenzy.com/betaflight-motor-order/
    case MOTOR_FRAME_TYPE_BF_X: {
        _frame_type_string = "BF_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_BF_X_REV: {
    // betaflight octa quad X order, reversed motors
        _frame_type_string = "X_REV";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // octaquad frame class does not support this frame type
        _frame_type_string = "UNSUPPORTED";
        return false;
    } //octaquad
    return true;
}
#endif // AP_MOTORS_FRAME_OCTAQUAD_ENABLED
#if AP_MOTORS_FRAME_DODECAHEXA_ENABLED
bool AP_MotorsMatrix::setup_dodecahexa_matrix(motor_frame_type frame_type)
{
    _mav_type = MAV_TYPE_DODECAROTOR;
    _frame_class_string = "DODECAHEXA";
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 }, // forward-top
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 }, // forward-bottom
            {   60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 }, // forward-right-top
            {   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 }, // forward-right-bottom
            {  120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 }, // back-right-top
            {  120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 }, // back-right-bottom
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 }, // back-top
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 }, // back-bottom
            { -120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9 }, // back-left-top
            { -120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10 }, // back-left-bottom
            {  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11 }, // forward-left-top
            {  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12 }, // forward-left-bottom
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   1 }, // forward-right-top
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    2 }, // forward-right-bottom
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    3 }, // right-top
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   4 }, // right-bottom
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   5 }, // back-right-top
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    6 }, // back-right-bottom
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    7 }, // back-left-top
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   8 }, // back-left-bottom
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   9 }, // left-top
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10 }, // left-bottom
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11 }, // forward-left-top
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12 }, // forward-left-bottom
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // dodeca-hexa frame class does not support this frame type
        _frame_type_string = "UNSUPPORTED";
        return false;
    } //dodecahexa
    return true;
}
#endif //AP_MOTORS_FRAME_DODECAHEXA_ENABLED
#if AP_MOTORS_FRAME_Y6_ENABLED
bool AP_MotorsMatrix::setup_y6_matrix(motor_frame_type frame_type)
{
    _mav_type = MAV_TYPE_HEXAROTOR;
    _frame_class_string = "Y6";
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_Y6B: {
        // Y6 motor definition with all top motors spinning clockwise, all bottom motors counter clockwise
        _frame_type_string = "Y6B";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            { -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
            {  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_Y6F: {
        // Y6 motor layout for FireFlyY6
        _frame_type_string = "Y6F";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            { -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            {  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
            { -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            {  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    default: {
        _frame_type_string = "default";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            {  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
            {  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            {  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
            { -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            {  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    } //y6
    return true;
}
#endif // AP_MOTORS_FRAME_Y6_ENABLED
#if AP_MOTORS_FRAME_DECA_ENABLED
bool AP_MotorsMatrix::setup_deca_matrix(motor_frame_type frame_type)
{
    _mav_type = MAV_TYPE_DECAROTOR;
    _frame_class_string = "DECA";
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   1 },
            {   36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    2 },
            {   72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   3 },
            {  108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    4 },
            {  144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   5 },
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    6 },
            { -144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   7 },
            { -108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    8 },
            {  -72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   9 },
            {  -36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X:
    case MOTOR_FRAME_TYPE_CW_X: {
        _frame_type_string = "X/CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   18, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   1 },
            {   54, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    2 },
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   3 },
            {  126, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    4 },
            {  162, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   5 },
            { -162, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    6 },
            { -126, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   7 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    8 },
            {  -54, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   9 },
            {  -18, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // deca frame class does not support this frame type
        return false;
    } //deca
    return true;
}
#endif // AP_MOTORS_FRAME_DECA_ENABLED

void AP_MotorsMatrix::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }
    set_initialised_ok(false);
    bool success = true;

    switch (frame_class) {
#if AP_MOTORS_FRAME_QUAD_ENABLED
    case MOTOR_FRAME_QUAD:
        success = setup_quad_matrix(frame_type);
        break;  // quad
#endif //AP_MOTORS_FRAME_QUAD_ENABLED
#if AP_MOTORS_FRAME_HEXA_ENABLED
    case MOTOR_FRAME_HEXA:
        success = setup_hexa_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_HEXA_ENABLED
#if AP_MOTORS_FRAME_OCTA_ENABLED
    case MOTOR_FRAME_OCTA:
        success = setup_octa_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_OCTA_ENABLED
#if AP_MOTORS_FRAME_OCTAQUAD_ENABLED
    case MOTOR_FRAME_OCTAQUAD:
        success = setup_octaquad_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_OCTAQUAD_ENABLED
#if AP_MOTORS_FRAME_DODECAHEXA_ENABLED
    case MOTOR_FRAME_DODECAHEXA:
        success = setup_dodecahexa_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_DODECAHEXA_ENABLED
#if AP_MOTORS_FRAME_Y6_ENABLED
    case MOTOR_FRAME_Y6:
        success = setup_y6_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_Y6_ENABLED
#if AP_MOTORS_FRAME_DECA_ENABLED
    case MOTOR_FRAME_DECA:
        success = setup_deca_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_DECA_ENABLED
    default:
        // matrix doesn't support the configured class
        _frame_class_string = "UNSUPPORTED";
        success = false;
        _mav_type = MAV_TYPE_GENERIC;
        break;
    } // switch frame_class

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    set_initialised_ok(success);
}

// normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
// normalizes throttle factors so max value is 1 and no value is less than 0
void AP_MotorsMatrix::normalise_rpy_factors()
{
    float roll_fac = 0.0f;
    float pitch_fac = 0.0f;
    float yaw_fac = 0.0f;
    float throttle_fac = 0.0f;

    // find maximum roll, pitch and yaw factors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            roll_fac = MAX(roll_fac,fabsf(_roll_factor[i]));
            pitch_fac = MAX(pitch_fac,fabsf(_pitch_factor[i]));
            yaw_fac = MAX(yaw_fac,fabsf(_yaw_factor[i]));
            throttle_fac = MAX(throttle_fac,MAX(0.0f,_throttle_factor[i]));
        }
    }

    // scale factors back to -0.5 to +0.5 for each axis
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (!is_zero(roll_fac)) {
                _roll_factor[i] = 0.5f * _roll_factor[i] / roll_fac;
            }
            if (!is_zero(pitch_fac)) {
                _pitch_factor[i] = 0.5f * _pitch_factor[i] / pitch_fac;
            }
            if (!is_zero(yaw_fac)) {
                _yaw_factor[i] = 0.5f * _yaw_factor[i] / yaw_fac;
            }
            if (!is_zero(throttle_fac)) {
                _throttle_factor[i] = MAX(0.0f,_throttle_factor[i] / throttle_fac);
            }
        }
    }
}


/*
  call vehicle supplied thrust compensation if set. This allows
  vehicle code to compensate for vehicle specific motor arrangements
  such as tiltrotors or tiltwings
*/
void AP_MotorsMatrix::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
    }
}

/*
  disable the use of motor torque to control yaw. Used when an
  external mechanism such as vectoring is used for yaw control
*/
void AP_MotorsMatrix::disable_yaw_torque(void)
{
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _yaw_factor[i] = 0;
    }
}

// singleton instance
AP_MotorsMatrix *AP_MotorsMatrix::_singleton;
