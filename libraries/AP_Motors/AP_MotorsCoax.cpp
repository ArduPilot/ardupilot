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
#include "AP_MotorsCoax.h"

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_MotorsCoax::var_info[] PROGMEM = {
    // 0 was used by TB_RATIO

    // @Param: TCRV_ENABLE
    // @DisplayName: Thrust Curve Enable
    // @Description: Controls whether a curve is used to linearize the thrust produced by the motors
    // @User: Advanced
    // @Values: 0:Disabled,1:Enable
    AP_GROUPINFO("TCRV_ENABLE", 1, AP_MotorsCoax, _throttle_curve_enabled, THROTTLE_CURVE_ENABLED),

    // @Param: TCRV_MIDPCT
    // @DisplayName: Thrust Curve mid-point percentage
    // @Description: Set the pwm position that produces half the maximum thrust of the motors
    // @User: Advanced
    // @Range: 20 80
    // @Increment: 1
    AP_GROUPINFO("TCRV_MIDPCT", 2, AP_MotorsCoax, _throttle_curve_mid, THROTTLE_CURVE_MID_THRUST),

    // @Param: TCRV_MAXPCT
    // @DisplayName: Thrust Curve max thrust percentage
    // @Description: Set to the lowest pwm position that produces the maximum thrust of the motors.  Most motors produce maximum thrust below the maximum pwm value that they accept.
    // @User: Advanced
    // @Range: 20 80
    // @Increment: 1
    AP_GROUPINFO("TCRV_MAXPCT", 3, AP_MotorsCoax, _throttle_curve_max, THROTTLE_CURVE_MAX_THRUST),

    // @Param: SPIN_ARMED
    // @DisplayName: Motors always spin when armed
    // @Description: Controls whether motors always spin when armed (must be below THR_MIN)
    // @Values: 0:Do Not Spin,70:VerySlow,100:Slow,130:Medium,150:Fast
    // @User: Standard
    AP_GROUPINFO("SPIN_ARMED", 5, AP_MotorsCoax, _spin_when_armed, AP_MOTORS_SPIN_WHEN_ARMED),

    // @Param: REV_ROLL
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_ROLL", 6, AP_MotorsCoax, _rev_roll, AP_MOTORS_COAX_POSITIVE),

    // @Param: REV_PITCH
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_PITCH", 7, AP_MotorsCoax, _rev_pitch, AP_MOTORS_COAX_POSITIVE),

	// @Param: REV_ROLL
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_YAW", 8, AP_MotorsCoax, _rev_yaw, AP_MOTORS_COAX_POSITIVE),

	// @Param: SV_SPEED
    // @DisplayName: Servo speed 
    // @Description: Servo update speed
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("SV_SPEED", 9, AP_MotorsCoax, _servo_speed, AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS),

    AP_GROUPEND
};
// init
void AP_MotorsCoax::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

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
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]) ;
    hal.rcout->set_freq(mask2, _speed_hz);

    // set update rate for the two servos
    uint32_t mask =
      1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]) |
      1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]) ;
    hal.rcout->set_freq(mask, _servo_speed);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsCoax::enable()
{
    // enable output channels
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]));
}

// output_min - sends minimum values out to the motor and trim values to the servos
void AP_MotorsCoax::output_min()
{
    // send minimum value to each motor
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), _servo1.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), _servo2.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), _rc_throttle.radio_min);
}

// output_armed - sends commands to the motors
void AP_MotorsCoax::output_armed()
{
    int16_t out_min = _rc_throttle.radio_min + _min_throttle;
    int16_t motor_out[4];

    // Throttle is 0 to 1000 only
    if (_rc_throttle.servo_out <= 0) {
        _rc_throttle.servo_out = 0;
        limit.throttle_lower = true;
    }
    if (_rc_throttle.servo_out >= _max_throttle) {
        _rc_throttle.servo_out = _max_throttle;
        limit.throttle_upper = true;
    }

    // capture desired throttle and yaw from receiver
    _rc_throttle.calc_pwm();
    _rc_yaw.calc_pwm();

    // if we are not sending a throttle output, we cut the motors
    if(_rc_throttle.servo_out == 0) {
        // range check spin_when_armed
        if (_spin_when_armed < 0) {
            _spin_when_armed = 0;
        }
        if (_spin_when_armed > _min_throttle) {
            _spin_when_armed = _min_throttle;
        }
        motor_out[AP_MOTORS_MOT_3] = _rc_throttle.radio_min + _spin_when_armed;
        motor_out[AP_MOTORS_MOT_4] = _rc_throttle.radio_min + _spin_when_armed;
    }else{

        // check if throttle is below limit
        if (_rc_throttle.servo_out <= _min_throttle) {  // perhaps being at min throttle itself is not a problem, only being under is
            limit.throttle_lower = true;
        }

        // motors
        motor_out[AP_MOTORS_MOT_3] = _rev_yaw*_rc_yaw.pwm_out + _rc_throttle.radio_out;
        motor_out[AP_MOTORS_MOT_4] = -_rev_yaw*_rc_yaw.pwm_out +_rc_throttle.radio_out;
        // front
        _servo1.servo_out = _rev_roll*_rc_roll.servo_out;
        // right
        _servo2.servo_out = _rev_pitch*_rc_pitch.servo_out;
		_servo1.calc_pwm();
		_servo2.calc_pwm();

        // adjust for throttle curve
        if( _throttle_curve_enabled ) {
            motor_out[AP_MOTORS_MOT_3] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_3]);
            motor_out[AP_MOTORS_MOT_4] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_4]);
        }

        // ensure motors don't drop below a minimum value and stop
        motor_out[AP_MOTORS_MOT_3] = max(motor_out[AP_MOTORS_MOT_3],    out_min);
        motor_out[AP_MOTORS_MOT_4] = max(motor_out[AP_MOTORS_MOT_4],    out_min);
    }

    // send output to each motor
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), _servo1.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), _servo2.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), motor_out[AP_MOTORS_MOT_3]);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), motor_out[AP_MOTORS_MOT_4]);
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
    if (!_flags.armed) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // flap servo 1
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), pwm);
            break;
        case 2:
            // flap servo 2
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), pwm);
            break;
        case 3:
            // motor 1
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), pwm);
            break;
        case 4:
            // motor 2
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), pwm);
            break;
        default:
            // do nothing
            break;
    }
}
