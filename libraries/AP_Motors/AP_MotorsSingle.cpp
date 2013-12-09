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

#include <AP_HAL.h>
#include <AP_Math.h>
#include "AP_MotorsSingle.h"

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_MotorsSingle::var_info[] PROGMEM = {
    // 0 was used by TB_RATIO

    // @Param: TCRV_ENABLE
    // @DisplayName: Thrust Curve Enable
    // @Description: Controls whether a curve is used to linearize the thrust produced by the motors
    // @User: Advanced
    // @Values: 0:Disabled,1:Enable
    AP_GROUPINFO("TCRV_ENABLE", 1, AP_MotorsSingle, _throttle_curve_enabled, THROTTLE_CURVE_ENABLED),

    // @Param: TCRV_MIDPCT
    // @DisplayName: Thrust Curve mid-point percentage
    // @Description: Set the pwm position that produces half the maximum thrust of the motors
    // @User: Advanced
    // @Range: 20 80
    // @Increment: 1
    AP_GROUPINFO("TCRV_MIDPCT", 2, AP_MotorsSingle, _throttle_curve_mid, THROTTLE_CURVE_MID_THRUST),

    // @Param: TCRV_MAXPCT
    // @DisplayName: Thrust Curve max thrust percentage
    // @Description: Set to the lowest pwm position that produces the maximum thrust of the motors.  Most motors produce maximum thrust below the maximum pwm value that they accept.
    // @User: Advanced
    // @Range: 20 80
    // @Increment: 1
    AP_GROUPINFO("TCRV_MAXPCT", 3, AP_MotorsSingle, _throttle_curve_max, THROTTLE_CURVE_MAX_THRUST),

    // @Param: SPIN_ARMED
    // @DisplayName: Motors always spin when armed
    // @Description: Controls whether motors always spin when armed (must be below THR_MIN)
    // @Values: 0:Do Not Spin,70:VerySlow,100:Slow,130:Medium,150:Fast
    // @User: Standard
    AP_GROUPINFO("SPIN_ARMED", 5, AP_MotorsSingle, _spin_when_armed, AP_MOTORS_SPIN_WHEN_ARMED),

    // @Param: REV_ROLL
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_ROLL", 6, AP_MotorsSingle, _rev_roll, AP_MOTORS_SING_POSITIVE),

    // @Param: REV_PITCH
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_PITCH", 7, AP_MotorsSingle, _rev_pitch, AP_MOTORS_SING_POSITIVE),

	// @Param: REV_ROLL
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_YAW", 8, AP_MotorsSingle, _rev_yaw, AP_MOTORS_SING_POSITIVE),

	// @Param: SV_SPEED
    // @DisplayName: Servo speed 
    // @Description: Servo update speed
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("SV_SPEED", 9, AP_MotorsSingle, _servo_speed, AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS),

    AP_GROUPEND
};
// init
void AP_MotorsSingle::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
}

// set update rate to motors - a value in hertz
void AP_MotorsSingle::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_1] |
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_2] |
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_3] |
		1U << _motor_to_channel_map[AP_MOTORS_MOT_4] ;
    hal.rcout->set_freq(mask, _servo_speed);
	uint32_t mask2 = 1U << _motor_to_channel_map[AP_MOTORS_MOT_7];
	hal.rcout->set_freq(mask2, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsSingle::enable()
{
    // enable output channels
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_1]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_2]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_3]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_4]);
	hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_7]);
}

// output_min - sends minimum values out to the motor and trim values to the servos
void AP_MotorsSingle::output_min()
{
    // fill the motor_out[] array for HIL use
    motor_out[AP_MOTORS_MOT_1] = _servo1->radio_trim;
    motor_out[AP_MOTORS_MOT_2] = _servo2->radio_trim;
	motor_out[AP_MOTORS_MOT_3] = _servo3->radio_trim;
    motor_out[AP_MOTORS_MOT_4] = _servo4->radio_trim;
	motor_out[AP_MOTORS_MOT_7] = _rc_throttle->radio_min;

    // send minimum value to each motor
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo1->radio_trim);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo2->radio_trim);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo3->radio_trim);
	hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo4->radio_trim);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_7], _rc_throttle->radio_min);
}

// output_armed - sends commands to the motors
void AP_MotorsSingle::output_armed()
{
    int16_t out_min = _rc_throttle->radio_min + _min_throttle;

    // Throttle is 0 to 1000 only
    _rc_throttle->servo_out = constrain_int16(_rc_throttle->servo_out, 0, _max_throttle);

    // capture desired throttle from receiver
    _rc_throttle->calc_pwm();

    // if we are not sending a throttle output, we cut the motors
    if(_rc_throttle->servo_out == 0) {
        // range check spin_when_armed
        if (_spin_when_armed < 0) {
            _spin_when_armed = 0;
        }
        if (_spin_when_armed > _min_throttle) {
            _spin_when_armed = _min_throttle;
        }
        
        motor_out[AP_MOTORS_MOT_7] = _rc_throttle->radio_min + _spin_when_armed;
    }else{

		//motor
		motor_out[AP_MOTORS_MOT_7] = _rc_throttle->radio_out;
        //front
        _servo1->servo_out = _rev_roll*_rc_roll->servo_out + _rev_yaw*_rc_yaw->servo_out;
		//right
        _servo2->servo_out = _rev_pitch*_rc_pitch->servo_out + _rev_yaw*_rc_yaw->servo_out;
		//rear
		_servo3->servo_out = -_rev_roll*_rc_roll->servo_out + _rev_yaw*_rc_yaw->servo_out;
		//left
		_servo4->servo_out = -_rev_pitch*_rc_pitch->servo_out + _rev_yaw*_rc_yaw->servo_out;

		_servo1->calc_pwm();
		_servo2->calc_pwm();
		_servo3->calc_pwm();
		_servo4->calc_pwm();

		motor_out[AP_MOTORS_MOT_1] = _servo1->radio_out;
		motor_out[AP_MOTORS_MOT_2] = _servo2->radio_out;
		motor_out[AP_MOTORS_MOT_3] = _servo3->radio_out;
		motor_out[AP_MOTORS_MOT_4] = _servo4->radio_out;

        // adjust for throttle curve
        if( _throttle_curve_enabled ) {
            motor_out[AP_MOTORS_MOT_7] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_7]);
        }

        // ensure motors don't drop below a minimum value and stop
        motor_out[AP_MOTORS_MOT_7] = max(motor_out[AP_MOTORS_MOT_7],    out_min);
    }

    // send output to each motor
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], motor_out[AP_MOTORS_MOT_1]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], motor_out[AP_MOTORS_MOT_2]);
	hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], motor_out[AP_MOTORS_MOT_3]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], motor_out[AP_MOTORS_MOT_4]);
	hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_7], motor_out[AP_MOTORS_MOT_7]);

}

// output_disarmed - sends commands to the motors
void AP_MotorsSingle::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin each motor for a moment to allow the user to confirm the motor order and spin direction
void AP_MotorsSingle::output_test()
{
    // Send minimum values to all motors
    output_min();

    // spin main motor
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_7], _rc_throttle->radio_min + _min_throttle);
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_7], _rc_throttle->radio_min);
    hal.scheduler->delay(2000);   

    // flap servo 1
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo1->radio_min);
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo1->radio_max);
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo1->radio_trim);
    hal.scheduler->delay(2000);

    // flap servo 2
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo2->radio_min);
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo2->radio_max);
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo2->radio_trim);
    hal.scheduler->delay(2000);

    // flap servo 3
	hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo3->radio_min);
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo3->radio_max);
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo3->radio_trim);
    hal.scheduler->delay(2000);

    // flap servo 4
	hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo4->radio_min);
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo4->radio_max);
    hal.scheduler->delay(1000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo4->radio_trim);

    // Send minimum values to all motors
    output_min();
}
