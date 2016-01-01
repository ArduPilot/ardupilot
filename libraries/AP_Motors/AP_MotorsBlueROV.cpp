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
 *       AP_MotorsBlueROV.cpp - ArduCopter motors library
 *       Code by Rustom Jehangir. BlueRobotics.com
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsBlueROV.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsBlueROV::var_info[] = {
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
    AP_GROUPINFO("YAW_SV_REV", 31,     AP_MotorsBlueROV,  _yaw_servo_reverse, 1),

    // @Param: YAW_SV_TRIM
    // @DisplayName: Yaw Servo Trim/Center
    // @Description: Trim or center position of yaw servo
    // @Range: 1250 1750
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_TRIM", 32,     AP_MotorsBlueROV,  _yaw_servo_trim, 1500),

    // @Param: YAW_SV_MIN
    // @DisplayName: Yaw Servo Min Position
    // @Description: Minimum angle limit of yaw servo
    // @Range: 1000 1400
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_MIN", 33,     AP_MotorsBlueROV,  _yaw_servo_min, 1250),

    // @Param: YAW_SV_MAX
    // @DisplayName: Yaw Servo Max Position
    // @Description: Maximum angle limit of yaw servo
    // @Range: 1600 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_MAX", 34,     AP_MotorsBlueROV,  _yaw_servo_max, 1750),


    AP_GROUPEND
};

// init
void AP_MotorsBlueROV::Init()
{
    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;

    // disable CH7 from being used as an aux output (i.e. for lights, etc)
    RC_Channel_aux::disable_aux_channel(AP_MOTORS_CH_CAM_PITCH);
}

// set update rate to motors - a value in hertz
void AP_MotorsBlueROV::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
	    1U << AP_MOTORS_MOT_1 |
	    1U << AP_MOTORS_MOT_2 |
		1U << AP_MOTORS_MOT_3 |
	    1U << AP_MOTORS_MOT_4 |
		1U << AP_MOTORS_MOT_5 |
		1U << AP_MOTORS_MOT_6;
    hal.rcout->set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsBlueROV::enable()
{
    // enable output channels
    hal.rcout->enable_ch(AP_MOTORS_MOT_1);
    hal.rcout->enable_ch(AP_MOTORS_MOT_2);
    hal.rcout->enable_ch(AP_MOTORS_MOT_3);
    hal.rcout->enable_ch(AP_MOTORS_MOT_4);
    hal.rcout->enable_ch(AP_MOTORS_MOT_5);
    hal.rcout->enable_ch(AP_MOTORS_MOT_6);
    hal.rcout->enable_ch(AP_MOTORS_CH_CAM_PITCH);
}

// output_min - sends minimum values out to the motors
void AP_MotorsBlueROV::output_min()
{
    // set lower limit flag
    limit.throttle_lower = true;

    int16_t out_stopped = 1500;

    // send minimum value to each motor
    hal.rcout->cork();
    hal.rcout->write(AP_MOTORS_MOT_1, out_stopped);
    hal.rcout->write(AP_MOTORS_MOT_2, out_stopped);
    hal.rcout->write(AP_MOTORS_MOT_3, out_stopped);
    hal.rcout->write(AP_MOTORS_MOT_4, out_stopped);
    hal.rcout->write(AP_MOTORS_MOT_5, out_stopped);
    hal.rcout->write(AP_MOTORS_MOT_6, out_stopped);
    hal.rcout->write(AP_MOTORS_CH_CAM_PITCH, _yaw_servo_trim);
    hal.rcout->push();
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsBlueROV::get_motor_mask()
{
    // BlueROV uses channels 1,2,3,4,5,6 and 7
    return (1U << AP_MOTORS_MOT_1) |
        (1U << AP_MOTORS_MOT_2) |
		(1U << AP_MOTORS_MOT_3) |
        (1U << AP_MOTORS_MOT_4) |
		(1U << AP_MOTORS_MOT_5) |
		(1U << AP_MOTORS_MOT_6) |
        (1U << AP_MOTORS_CH_CAM_PITCH);
}

void AP_MotorsBlueROV::output_armed_not_stabilizing()
{
    int16_t throttle_output; // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min = 1100;
    int16_t out_max = 1900;
    int16_t out_stopped = 1500;
    int16_t motor_out[AP_MOTORS_MOT_6+1];

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

    throttle_output = out_stopped + _throttle_control_input;

    motor_out[AP_MOTORS_MOT_1] = throttle_output;
    motor_out[AP_MOTORS_MOT_2] = throttle_output;
    motor_out[AP_MOTORS_MOT_3] = throttle_output;
    motor_out[AP_MOTORS_MOT_4] = throttle_output;
    motor_out[AP_MOTORS_MOT_5] = throttle_output;
    motor_out[AP_MOTORS_MOT_6] = throttle_output;

    //if(throttle_output >= out_min) {
        // adjust for thrust curve and voltage scaling
        //motor_out[AP_MOTORS_MOT_1] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_1], out_min, out_max);
        //motor_out[AP_MOTORS_MOT_2] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_2], out_min, out_max);
        //motor_out[AP_MOTORS_MOT_3] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_3], out_min, out_max);
        //motor_out[AP_MOTORS_MOT_4] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_4], out_min, out_max);
        //motor_out[AP_MOTORS_MOT_5] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_5], out_min, out_max);
        //motor_out[AP_MOTORS_MOT_6] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_6], out_min, out_max);
    //}

    hal.rcout->cork();

    // send output to each motor
    hal.rcout->write(AP_MOTORS_MOT_1, motor_out[AP_MOTORS_MOT_1]);
    hal.rcout->write(AP_MOTORS_MOT_2, motor_out[AP_MOTORS_MOT_2]);
    hal.rcout->write(AP_MOTORS_MOT_3, motor_out[AP_MOTORS_MOT_3]);
    hal.rcout->write(AP_MOTORS_MOT_4, motor_out[AP_MOTORS_MOT_4]);
    hal.rcout->write(AP_MOTORS_MOT_5, motor_out[AP_MOTORS_MOT_5]);
    hal.rcout->write(AP_MOTORS_MOT_6, motor_out[AP_MOTORS_MOT_6]);

    // send centering signal to yaw servo
    hal.rcout->write(AP_MOTORS_CH_CAM_PITCH, _yaw_servo_trim);

    hal.rcout->push();
}

// sends commands to the motors
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
void AP_MotorsBlueROV::output_armed_stabilizing()
{
    int16_t roll_pwm;                                               // roll pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t pitch_pwm;                                              // pitch pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t throttle_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t forward_output;                                   // total forward value
    int16_t strafe_output;                                    // total strafe value
    int16_t yaw_radio_output;                                       // final yaw pwm value sent to motors, typically ~1100-1900
    int16_t out_min = 1100;
    int16_t out_max = 1900;
    int16_t out_stopped = 1500;
    int16_t motor_out[AP_MOTORS_MOT_6+1];

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
    //if (_throttle_control_input > 800) {
    //    _throttle_control_input = 800;
    //    limit.throttle_upper = true;
    //}

    roll_pwm = calc_roll_pwm();
    pitch_pwm = calc_pitch_pwm();
    throttle_output = _throttle_control_input;
    forward_output = _forward_control_input; // ToDo: Clean this up to match others
    strafe_output = _strafe_control_input;
    yaw_radio_output = calc_yaw_radio_output();

	//left front vertical
	motor_out[AP_MOTORS_MOT_1] = out_stopped + _throttle_control_input + roll_pwm + 0.5*pitch_pwm + 0.2*strafe_output;
	//left forward
	motor_out[AP_MOTORS_MOT_2] = out_stopped + forward_output + yaw_radio_output;
	//rear vertical
	motor_out[AP_MOTORS_MOT_3] = out_stopped + _throttle_control_input + roll_pwm - pitch_pwm;
	//right forward
	motor_out[AP_MOTORS_MOT_4] = out_stopped + forward_output - yaw_radio_output;
	//right front vertical
	motor_out[AP_MOTORS_MOT_5] = out_stopped + _throttle_control_input - roll_pwm + 0.5*pitch_pwm - 0.2*strafe_output;
	// strafe
	motor_out[AP_MOTORS_MOT_6] = out_stopped + strafe_output;

	// adjust for thrust curve and voltage scaling
	//motor_out[AP_MOTORS_MOT_1] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_1], out_min, out_max);
	//motor_out[AP_MOTORS_MOT_2] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_2], out_min, out_max);
	//motor_out[AP_MOTORS_MOT_3] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_3], out_min, out_max);
	//motor_out[AP_MOTORS_MOT_4] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_4], out_min, out_max);
	//motor_out[AP_MOTORS_MOT_5] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_5], out_min, out_max);
	//motor_out[AP_MOTORS_MOT_6] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_6], out_min, out_max);

    hal.rcout->cork();

    // send output to each motor
    hal.rcout->write(AP_MOTORS_MOT_1, motor_out[AP_MOTORS_MOT_1]);
	hal.rcout->write(AP_MOTORS_MOT_2, motor_out[AP_MOTORS_MOT_2]);
	hal.rcout->write(AP_MOTORS_MOT_3, motor_out[AP_MOTORS_MOT_3]);
	hal.rcout->write(AP_MOTORS_MOT_4, motor_out[AP_MOTORS_MOT_4]);
	hal.rcout->write(AP_MOTORS_MOT_5, motor_out[AP_MOTORS_MOT_5]);
	hal.rcout->write(AP_MOTORS_MOT_6, motor_out[AP_MOTORS_MOT_6]);

    // send out to yaw command to tail servo
    hal.rcout->write(AP_MOTORS_CH_CAM_PITCH, yaw_radio_output);

    hal.rcout->push();
}

// output_disarmed - sends commands to the motors
void AP_MotorsBlueROV::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsBlueROV::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            hal.rcout->write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            hal.rcout->write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
			hal.rcout->write(AP_MOTORS_MOT_3, pwm);
			break;
        case 4:
            hal.rcout->write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
			hal.rcout->write(AP_MOTORS_MOT_5, pwm);
			break;
        case 6:
        	hal.rcout->write(AP_MOTORS_MOT_6, pwm);
			break;
        case 7:
			hal.rcout->write(AP_MOTORS_CH_CAM_PITCH, pwm);
			break;
        default:
            // do nothing
            break;
    }
}

// calc_yaw_radio_output - calculate final radio output for yaw channel
int16_t AP_MotorsBlueROV::calc_yaw_radio_output()
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
