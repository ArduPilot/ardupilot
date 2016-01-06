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
 *       AP_MotorsSingle.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsSingle.h"

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_MotorsSingle::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // parameters 1 ~ 29 were reserved for tradheli
    // parameters 30 ~ 39 reserved for tricopter
    // parameters 40 ~ 49 for single copter and coax copter (these have identical parameter files)

    // @Param: ROLL_SV_REV
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Reversed,1:Normal
    AP_GROUPINFO("ROLL_SV_REV", 40, AP_MotorsSingle, _rev_roll, AP_MOTORS_SING_POSITIVE),

    // @Param: PITCH_SV_REV
    // @DisplayName: Reverse pitch feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Reversed,1:Normal
    AP_GROUPINFO("PITCH_SV_REV", 41, AP_MotorsSingle, _rev_pitch, AP_MOTORS_SING_POSITIVE),

	// @Param: YAW_SV_REV
    // @DisplayName: Reverse yaw feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Reversed,1:Normal
    AP_GROUPINFO("YAW_SV_REV", 42, AP_MotorsSingle, _rev_yaw, AP_MOTORS_SING_POSITIVE),

	// @Param: SV_SPEED
    // @DisplayName: Servo speed 
    // @Description: Servo update speed in hz
    // @Values: 50, 125, 250
    AP_GROUPINFO("SV_SPEED", 43, AP_MotorsSingle, _servo_speed, AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS),

    AP_GROUPEND
};
// init
void AP_MotorsSingle::Init()
{
    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the main ESC can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_7] = true;

    // we set four servos to angle
    _servo1.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo2.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo3.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo4.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo1.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
    _servo2.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
    _servo3.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
    _servo4.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);

    // disable CH7 from being used as an aux output (i.e. for camera gimbal, etc)
    RC_Channel_aux::disable_aux_channel(CH_7);
}

// set update rate to motors - a value in hertz
void AP_MotorsSingle::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4 ;
    rc_set_freq(mask, _servo_speed);
    uint32_t mask2 = 1U << AP_MOTORS_MOT_7;
    rc_set_freq(mask2, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsSingle::enable()
{
    // enable output channels
    rc_enable_ch(AP_MOTORS_MOT_1);
    rc_enable_ch(AP_MOTORS_MOT_2);
    rc_enable_ch(AP_MOTORS_MOT_3);
    rc_enable_ch(AP_MOTORS_MOT_4);
    rc_enable_ch(AP_MOTORS_MOT_7);
}

// output_min - sends minimum values out to the motor and trim values to the servos
void AP_MotorsSingle::output_min()
{
    // send minimum value to each motor
    hal.rcout->cork();
    rc_write(AP_MOTORS_MOT_1, _servo1.radio_trim);
    rc_write(AP_MOTORS_MOT_2, _servo2.radio_trim);
    rc_write(AP_MOTORS_MOT_3, _servo3.radio_trim);
    rc_write(AP_MOTORS_MOT_4, _servo4.radio_trim);
    rc_write(AP_MOTORS_MOT_7, _throttle_radio_min);
    hal.rcout->push();
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsSingle::get_motor_mask()
{
    // single copter uses channels 1,2,3,4 and 7
    return rc_map_mask(1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << 6);
}

void AP_MotorsSingle::output_armed_not_stabilizing()
{
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min = _throttle_radio_min + _min_throttle;

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
    if (_throttle_control_input >= _max_throttle) {
        _throttle_control_input = _max_throttle;
        limit.throttle_upper = true;
    }

    throttle_radio_output = calc_throttle_radio_output();

    // front servo
    _servo1.servo_out = 0;
    // right servo
    _servo2.servo_out = 0;
    // rear servo
    _servo3.servo_out = 0;
    // left servo
    _servo4.servo_out = 0;

    _servo1.calc_pwm();
    _servo2.calc_pwm();
    _servo3.calc_pwm();
    _servo4.calc_pwm();

    if (throttle_radio_output >= out_min) {
        throttle_radio_output = apply_thrust_curve_and_volt_scaling(throttle_radio_output, out_min, _throttle_radio_max);
    }

    hal.rcout->cork();
    rc_write(AP_MOTORS_MOT_1, _servo1.radio_out);
    rc_write(AP_MOTORS_MOT_2, _servo2.radio_out);
    rc_write(AP_MOTORS_MOT_3, _servo3.radio_out);
    rc_write(AP_MOTORS_MOT_4, _servo4.radio_out);
    rc_write(AP_MOTORS_MOT_7, throttle_radio_output);
    hal.rcout->push();
}

// sends commands to the motors
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
void AP_MotorsSingle::output_armed_stabilizing()
{
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min = _throttle_radio_min + _min_throttle;

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

    // calculate throttle PWM
    throttle_radio_output = calc_throttle_radio_output();

    // adjust for thrust curve and voltage scaling
    throttle_radio_output = apply_thrust_curve_and_volt_scaling(throttle_radio_output, out_min, _throttle_radio_max);

    // ensure motor doesn't drop below a minimum value and stop
    throttle_radio_output = MAX(throttle_radio_output, out_min);

    // TODO: set limits.roll_pitch and limits.yaw

    // front servo
    _servo1.servo_out = _rev_roll*_roll_control_input + _rev_yaw*_yaw_control_input;
    // right servo
    _servo2.servo_out = _rev_pitch*_pitch_control_input + _rev_yaw*_yaw_control_input;
    // rear servo
    _servo3.servo_out = -_rev_roll*_roll_control_input + _rev_yaw*_yaw_control_input;
    // left servo
    _servo4.servo_out = -_rev_pitch*_pitch_control_input + _rev_yaw*_yaw_control_input;

    _servo1.calc_pwm();
    _servo2.calc_pwm();
    _servo3.calc_pwm();
    _servo4.calc_pwm();

    // send output to each motor
    hal.rcout->cork();
    rc_write(AP_MOTORS_MOT_1, _servo1.radio_out);
    rc_write(AP_MOTORS_MOT_2, _servo2.radio_out);
    rc_write(AP_MOTORS_MOT_3, _servo3.radio_out);
    rc_write(AP_MOTORS_MOT_4, _servo4.radio_out);
    rc_write(AP_MOTORS_MOT_7, throttle_radio_output);
    hal.rcout->push();
}

// output_disarmed - sends commands to the motors
void AP_MotorsSingle::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsSingle::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // flap servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // flap servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // flap servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // flap servo 4
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // spin main motor
            rc_write(AP_MOTORS_MOT_7, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
