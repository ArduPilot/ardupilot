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
#include "AP_MotorsCoax.h"

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_MotorsCoax::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // parameters 1 ~ 29 were reserved for tradheli
    // parameters 30 ~ 39 reserved for tricopter
    // parameters 40 ~ 49 for single copter and coax copter (these have identical parameter files)

    // @Param: ROLL_SV_REV
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Reversed,1:Normal
    AP_GROUPINFO("ROLL_SV_REV", 40, AP_MotorsCoax, _rev_roll, AP_MOTORS_COAX_POSITIVE),

    // @Param: PITCH_SV_REV
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Reversed,1:Normal
    AP_GROUPINFO("PITCH_SV_REV", 41, AP_MotorsCoax, _rev_pitch, AP_MOTORS_COAX_POSITIVE),

	// @Param: YAW_SV_REV
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Reversed,1:Normal
    AP_GROUPINFO("YAW_SV_REV", 42, AP_MotorsCoax, _rev_yaw, AP_MOTORS_COAX_POSITIVE),

	// @Param: SV_SPEED
    // @DisplayName: Servo speed 
    // @Description: Servo update speed
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("SV_SPEED", 43, AP_MotorsCoax, _servo_speed, AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS),

    AP_GROUPEND
};
// init
void AP_MotorsCoax::Init()
{
    // set update rate for the 2 motors (but not the servo on channel 1&2)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;

    // set ranges for fin servos
    _servo1.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo2.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo1.set_angle(AP_MOTORS_COAX_SERVO_INPUT_RANGE);
    _servo2.set_angle(AP_MOTORS_COAX_SERVO_INPUT_RANGE);
}

// set update rate to motors - a value in hertz
void AP_MotorsCoax::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the two motors
    uint32_t mask2 =
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4 ;
    rc_set_freq(mask2, _speed_hz);

    // set update rate for the two servos
    uint32_t mask =
      1U << AP_MOTORS_MOT_1 |
      1U << AP_MOTORS_MOT_2 ;
    rc_set_freq(mask, _servo_speed);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsCoax::enable()
{
    // enable output channels
    rc_enable_ch(AP_MOTORS_MOT_1);
    rc_enable_ch(AP_MOTORS_MOT_2);
    rc_enable_ch(AP_MOTORS_MOT_3);
    rc_enable_ch(AP_MOTORS_MOT_4);
}

// output_min - sends minimum values out to the motor and trim values to the servos
void AP_MotorsCoax::output_min()
{
    // send minimum value to each motor
    hal.rcout->cork();
    rc_write(AP_MOTORS_MOT_1, _servo1.radio_trim);
    rc_write(AP_MOTORS_MOT_2, _servo2.radio_trim);
    rc_write(AP_MOTORS_MOT_3, _throttle_radio_min);
    rc_write(AP_MOTORS_MOT_4, _throttle_radio_min);
    hal.rcout->push();
}

void AP_MotorsCoax::output_armed_not_stabilizing()
{
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min = _throttle_radio_min + _min_throttle;
    int16_t motor_out;

    int16_t min_thr = rel_pwm_to_thr_range(_spin_when_armed_ramped);

    // initialize limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    if (_throttle_control_input <= min_thr) {
        _throttle_control_input = min_thr;
        limit.throttle_lower = true;
    }
    if (_throttle_control_input >= _max_throttle) {
        _throttle_control_input = _max_throttle;
        limit.throttle_upper = true;
    }

    throttle_radio_output = calc_throttle_radio_output();

    motor_out = throttle_radio_output;

    _servo1.servo_out = 0;
    _servo1.calc_pwm();

    _servo2.servo_out = 0;
    _servo2.calc_pwm();

    if (motor_out >= out_min) {
        motor_out = apply_thrust_curve_and_volt_scaling(motor_out, out_min, _throttle_radio_max);
    }

    hal.rcout->cork();
    rc_write(AP_MOTORS_MOT_1, _servo1.radio_out);
    rc_write(AP_MOTORS_MOT_2, _servo2.radio_out);
    rc_write(AP_MOTORS_MOT_3, motor_out);
    rc_write(AP_MOTORS_MOT_4, motor_out);
    hal.rcout->push();
}

// sends commands to the motors
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
void AP_MotorsCoax::output_armed_stabilizing()
{
    int16_t yaw_pwm;                                                // yaw pwm value, initially calculated by calc_yaw_pwm() but may be modified after, +/- 400
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min = _throttle_radio_min + _min_throttle;
    int16_t motor_out[4];

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    int16_t thr_in_min = rel_pwm_to_thr_range(_min_throttle);
    if (_throttle_control_input <= thr_in_min) {
        _throttle_control_input = thr_in_min;
        limit.throttle_lower = true;
    }
    if (_throttle_control_input >= _max_throttle) {
        _throttle_control_input = _max_throttle;
        limit.throttle_upper = true;
    }

    // calculate throttle and yaw PWM
    throttle_radio_output = calc_throttle_radio_output();
    yaw_pwm = calc_yaw_pwm();

    // motors
    motor_out[AP_MOTORS_MOT_3] = _rev_yaw*yaw_pwm + throttle_radio_output;
    motor_out[AP_MOTORS_MOT_4] = -_rev_yaw*yaw_pwm + throttle_radio_output;

    // TODO: set limits.roll_pitch and limits.yaw

    // front
    _servo1.servo_out = _rev_roll*_roll_control_input;
    // right
    _servo2.servo_out = _rev_pitch*_pitch_control_input;

    _servo1.calc_pwm();
    _servo2.calc_pwm();

    // adjust for thrust curve and voltage scaling
    motor_out[AP_MOTORS_MOT_3] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_3], out_min, _throttle_radio_max);
    motor_out[AP_MOTORS_MOT_4] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_4], out_min, _throttle_radio_max);

    // ensure motors don't drop below a minimum value and stop
    motor_out[AP_MOTORS_MOT_3] = MAX(motor_out[AP_MOTORS_MOT_3],    out_min);
    motor_out[AP_MOTORS_MOT_4] = MAX(motor_out[AP_MOTORS_MOT_4],    out_min);

    // send output to each motor
    hal.rcout->cork();
    rc_write(AP_MOTORS_MOT_1, _servo1.radio_out);
    rc_write(AP_MOTORS_MOT_2, _servo2.radio_out);
    rc_write(AP_MOTORS_MOT_3, motor_out[AP_MOTORS_MOT_3]);
    rc_write(AP_MOTORS_MOT_4, motor_out[AP_MOTORS_MOT_4]);
    hal.rcout->push();
}

// output_disarmed - sends commands to the motors
void AP_MotorsCoax::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsCoax::output_test(uint8_t motor_seq, int16_t pwm)
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
            // motor 1
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // motor 2
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
