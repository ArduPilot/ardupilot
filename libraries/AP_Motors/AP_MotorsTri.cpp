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
 *       AP_MotorsTri.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTri.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsTri::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // parameters 1 ~ 29 were reserved for tradheli
    // parameters 30 ~ 39 reserved for tricopter
    // parameters 40 ~ 49 for single copter and coax copter (these have identical parameter files)

    // @Param: YAW_SV_REV
    // @DisplayName: Yaw Servo Reverse
    // @Description: Yaw servo reversing. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
    // @Values: -1:Reversed,1:Normal
    // @User: Standard
    AP_GROUPINFO("YAW_SV_REV", 31,     AP_MotorsTri,  _yaw_servo_reverse, 1),

    // @Param: YAW_SV_TRIM
    // @DisplayName: Yaw Servo Trim/Center
    // @Description: Trim or center position of yaw servo
    // @Range: 1250 1750
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_TRIM", 32,     AP_MotorsTri,  _yaw_servo_trim, 1500),

    // @Param: YAW_SV_MIN
    // @DisplayName: Yaw Servo Min Position
    // @Description: Minimum angle limit of yaw servo
    // @Range: 1000 1400
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_MIN", 33,     AP_MotorsTri,  _yaw_servo_min, 1250),

    // @Param: YAW_SV_MAX
    // @DisplayName: Yaw Servo Max Position
    // @Description: Maximum angle limit of yaw servo
    // @Range: 1600 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_MAX", 34,     AP_MotorsTri,  _yaw_servo_max, 1750),


    AP_GROUPEND
};

// init
void AP_MotorsTri::Init()
{
    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;

    // disable CH7 from being used as an aux output (i.e. for camera gimbal, etc)
    RC_Channel_aux::disable_aux_channel(AP_MOTORS_CH_TRI_YAW);
}

// set update rate to motors - a value in hertz
void AP_MotorsTri::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
	    1U << AP_MOTORS_MOT_1 |
	    1U << AP_MOTORS_MOT_2 |
	    1U << AP_MOTORS_MOT_4;
    rc_set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsTri::enable()
{
    // enable output channels
    rc_enable_ch(AP_MOTORS_MOT_1);
    rc_enable_ch(AP_MOTORS_MOT_2);
    rc_enable_ch(AP_MOTORS_MOT_4);
    rc_enable_ch(AP_MOTORS_CH_TRI_YAW);
}

// output_min - sends minimum values out to the motors
void AP_MotorsTri::output_min()
{
    // set lower limit flag
    limit.throttle_lower = true;

    // send minimum value to each motor
    hal.rcout->cork();
    rc_write(AP_MOTORS_MOT_1, _throttle_radio_min);
    rc_write(AP_MOTORS_MOT_2, _throttle_radio_min);
    rc_write(AP_MOTORS_MOT_4, _throttle_radio_min);
    rc_write(AP_MOTORS_CH_TRI_YAW, _yaw_servo_trim);
    hal.rcout->push();
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTri::get_motor_mask()
{
    // tri copter uses channels 1,2,4 and 7
    return rc_map_mask(1U << AP_MOTORS_MOT_1) |
        (1U << AP_MOTORS_MOT_2) |
        (1U << AP_MOTORS_MOT_4) |
        (1U << AP_MOTORS_CH_TRI_YAW);
}

void AP_MotorsTri::output_armed_not_stabilizing()
{
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min = _throttle_radio_min + _min_throttle;
    int16_t out_max = _throttle_radio_max;
    int16_t motor_out[AP_MOTORS_MOT_4+1];

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

    motor_out[AP_MOTORS_MOT_1] = throttle_radio_output;
    motor_out[AP_MOTORS_MOT_2] = throttle_radio_output;
    motor_out[AP_MOTORS_MOT_4] = throttle_radio_output;

    if(throttle_radio_output >= out_min) {
        // adjust for thrust curve and voltage scaling
        motor_out[AP_MOTORS_MOT_1] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_1], out_min, out_max);
        motor_out[AP_MOTORS_MOT_2] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_2], out_min, out_max);
        motor_out[AP_MOTORS_MOT_4] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_4], out_min, out_max);
    }

    hal.rcout->cork();

    // send output to each motor
    rc_write(AP_MOTORS_MOT_1, motor_out[AP_MOTORS_MOT_1]);
    rc_write(AP_MOTORS_MOT_2, motor_out[AP_MOTORS_MOT_2]);
    rc_write(AP_MOTORS_MOT_4, motor_out[AP_MOTORS_MOT_4]);

    // send centering signal to yaw servo
    rc_write(AP_MOTORS_CH_TRI_YAW, _yaw_servo_trim);

    hal.rcout->push();
}

// sends commands to the motors
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
void AP_MotorsTri::output_armed_stabilizing()
{
    int16_t roll_pwm;                                               // roll pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t pitch_pwm;                                              // pitch pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t yaw_radio_output;                                       // final yaw pwm value sent to motors, typically ~1100-1900
    int16_t out_min = _throttle_radio_min + _min_throttle;
    int16_t out_max = _throttle_radio_max;
    int16_t motor_out[AP_MOTORS_MOT_4+1];

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // Throttle is 0 to 1000 only
    int16_t thr_in_min = rel_pwm_to_thr_range(_min_throttle);
    if (_throttle_control_input <= thr_in_min) {
        _throttle_control_input = thr_in_min;
        limit.throttle_lower = true;
    }
    if (_throttle_control_input >= _max_throttle) {
        _throttle_control_input = _max_throttle;
        limit.throttle_upper = true;
    }

    // tricopters limit throttle to 80%
    // To-Do: implement improved stability patch and remove this limit
    if (_throttle_control_input > 800) {
        _throttle_control_input = 800;
        limit.throttle_upper = true;
    }

    roll_pwm = calc_roll_pwm();
    pitch_pwm = calc_pitch_pwm();
    throttle_radio_output = calc_throttle_radio_output();
    yaw_radio_output = calc_yaw_radio_output();

    // if we are not sending a throttle output, we cut the motors
    if( is_zero(_throttle_control_input) ) {
        // range check spin_when_armed
        if (_spin_when_armed_ramped < 0) {
            _spin_when_armed_ramped = 0;
        }
        if (_spin_when_armed_ramped > _min_throttle) {
            _spin_when_armed_ramped = _min_throttle;
        }
        motor_out[AP_MOTORS_MOT_1] = _throttle_radio_min + _spin_when_armed_ramped;
        motor_out[AP_MOTORS_MOT_2] = _throttle_radio_min + _spin_when_armed_ramped;
        motor_out[AP_MOTORS_MOT_4] = _throttle_radio_min + _spin_when_armed_ramped;

    }else{
        int16_t roll_out            = (float)(roll_pwm * 0.866f);
        int16_t pitch_out           = pitch_pwm / 2;

        // check if throttle is below limit
        if (_throttle_control_input <= _min_throttle) {
            limit.throttle_lower = true;
            _throttle_control_input = _min_throttle;
            throttle_radio_output = calc_throttle_radio_output();
        }

        // TODO: set limits.roll_pitch and limits.yaw

        //left front
        motor_out[AP_MOTORS_MOT_2] = throttle_radio_output + roll_out + pitch_out;
        //right front
        motor_out[AP_MOTORS_MOT_1] = throttle_radio_output - roll_out + pitch_out;
        // rear
        motor_out[AP_MOTORS_MOT_4] = throttle_radio_output - pitch_pwm;

        // Tridge's stability patch
        if(motor_out[AP_MOTORS_MOT_1] > out_max) {
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_1] - out_max);
            motor_out[AP_MOTORS_MOT_4] -= (motor_out[AP_MOTORS_MOT_1] - out_max);
            motor_out[AP_MOTORS_MOT_1] = out_max;
        }

        if(motor_out[AP_MOTORS_MOT_2] > out_max) {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_2] - out_max);
            motor_out[AP_MOTORS_MOT_4] -= (motor_out[AP_MOTORS_MOT_2] - out_max);
            motor_out[AP_MOTORS_MOT_2] = out_max;
        }

        if(motor_out[AP_MOTORS_MOT_4] > out_max) {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_4] - out_max);
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_4] - out_max);
            motor_out[AP_MOTORS_MOT_4] = out_max;
        }

        // adjust for thrust curve and voltage scaling
        motor_out[AP_MOTORS_MOT_1] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_1], out_min, out_max);
        motor_out[AP_MOTORS_MOT_2] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_2], out_min, out_max);
        motor_out[AP_MOTORS_MOT_4] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_4], out_min, out_max);

        // ensure motors don't drop below a minimum value and stop
        motor_out[AP_MOTORS_MOT_1] = MAX(motor_out[AP_MOTORS_MOT_1],    out_min);
        motor_out[AP_MOTORS_MOT_2] = MAX(motor_out[AP_MOTORS_MOT_2],    out_min);
        motor_out[AP_MOTORS_MOT_4] = MAX(motor_out[AP_MOTORS_MOT_4],    out_min);
    }

    hal.rcout->cork();

    // send output to each motor
    rc_write(AP_MOTORS_MOT_1, motor_out[AP_MOTORS_MOT_1]);
    rc_write(AP_MOTORS_MOT_2, motor_out[AP_MOTORS_MOT_2]);
    rc_write(AP_MOTORS_MOT_4, motor_out[AP_MOTORS_MOT_4]);

    // send out to yaw command to tail servo
    rc_write(AP_MOTORS_CH_TRI_YAW, yaw_radio_output);

    hal.rcout->push();
}

// output_disarmed - sends commands to the motors
void AP_MotorsTri::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTri::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // front right motor
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // back motor
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 3:
            // back servo
            rc_write(AP_MOTORS_CH_TRI_YAW, pwm);
            break;
        case 4:
            // front left motor
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

// calc_yaw_radio_output - calculate final radio output for yaw channel
int16_t AP_MotorsTri::calc_yaw_radio_output()
{
    int16_t ret;

    if (_yaw_servo_reverse < 0) {
        if (_yaw_control_input >= 0){
            ret = (_yaw_servo_trim - (_yaw_control_input/4500 * (_yaw_servo_trim - _yaw_servo_min)));
        } else {
            ret = (_yaw_servo_trim - (_yaw_control_input/4500 * (_yaw_servo_max - _yaw_servo_trim)));
        }
    } else {
        if (_yaw_control_input >= 0){
            ret = ((_yaw_control_input/4500 * (_yaw_servo_max - _yaw_servo_trim)) + _yaw_servo_trim);
        } else {
            ret = ((_yaw_control_input/4500 * (_yaw_servo_trim - _yaw_servo_min)) + _yaw_servo_trim);
        }
    }

    return ret;
}
