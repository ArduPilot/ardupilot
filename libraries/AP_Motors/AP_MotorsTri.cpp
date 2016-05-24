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
    AP_GROUPINFO("YAW_SV_REV", 31,     AP_MotorsTri,  _yaw_reverse, 1),

    // @Param: YAW_SV_TRIM
    // @DisplayName: Yaw Servo Trim/Center
    // @Description: Trim or center position of yaw servo
    // @Range: 1250 1750
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_TRIM", 32,     AP_MotorsTri,  _yaw_servo_trim, 1500),

    // @Param: YAW_SV_MIN
    // @DisplayName: Yaw Servo Min PWM
    // @Description: Yaw servo's minimum pwm value
    // @Range: 1000 1400
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_MIN", 33,     AP_MotorsTri,  _yaw_servo_min, 1250),

    // @Param: YAW_SV_MAX
    // @DisplayName: Yaw Servo Max PWM
    // @Description: Yaw servo's maximum pwm value
    // @Range: 1600 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_MAX", 34,     AP_MotorsTri,  _yaw_servo_max, 1750),

    // @Param: YAW_SV_ANGLE
    // @DisplayName: Yaw Servo Max Lean Angle
    // @Description: Yaw servo's maximum lean angle
    // @Range: 5 80
    // @Units: Degrees
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_SV_ANGLE", 35,   AP_MotorsTri,  _yaw_servo_angle_max_deg, 30),

    AP_GROUPEND
};

// init
void AP_MotorsTri::Init()
{
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_4);
    
    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;

    // allow mapping of motor7
    add_motor_num(AP_MOTORS_CH_TRI_YAW);
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

void AP_MotorsTri::output_to_motors()
{
    switch (_multicopter_flags.spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            hal.rcout->cork();
            rc_write(AP_MOTORS_MOT_1, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_2, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_4, get_pwm_output_min());
            rc_write(AP_MOTORS_CH_TRI_YAW, _yaw_servo_trim);
            hal.rcout->push();
            break;
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            hal.rcout->cork();
            rc_write(AP_MOTORS_MOT_1, constrain_int16(get_pwm_output_min() + _throttle_low_end_pct * _min_throttle, get_pwm_output_min(), get_pwm_output_min() + _min_throttle));
            rc_write(AP_MOTORS_MOT_2, constrain_int16(get_pwm_output_min() + _throttle_low_end_pct * _min_throttle, get_pwm_output_min(), get_pwm_output_min() + _min_throttle));
            rc_write(AP_MOTORS_MOT_4, constrain_int16(get_pwm_output_min() + _throttle_low_end_pct * _min_throttle, get_pwm_output_min(), get_pwm_output_min() + _min_throttle));
            rc_write(AP_MOTORS_CH_TRI_YAW, _yaw_servo_trim);
            hal.rcout->push();
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            hal.rcout->cork();
            rc_write(AP_MOTORS_MOT_1, calc_thrust_to_pwm(_thrust_right));
            rc_write(AP_MOTORS_MOT_2, calc_thrust_to_pwm(_thrust_left));
            rc_write(AP_MOTORS_MOT_4, calc_thrust_to_pwm(_thrust_rear));
            rc_write(AP_MOTORS_CH_TRI_YAW, calc_yaw_radio_output(_pivot_angle, radians(_yaw_servo_angle_max_deg)));
            hal.rcout->push();
            break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTri::get_motor_mask()
{
    // tri copter uses channels 1,2,4 and 7
    return rc_map_mask((1U << AP_MOTORS_MOT_1) |
                       (1U << AP_MOTORS_MOT_2) |
                       (1U << AP_MOTORS_MOT_4) |
                       (1U << AP_MOTORS_CH_TRI_YAW));
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsTri::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   throttle_thrust_rpy_mix;    // partial calculation of throttle_thrust_best_rpy
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   rpy_low = 0.0f;             // lowest motor value
    float   rpy_high = 0.0f;            // highest motor value
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    float   throttle_thrust_hover = get_hover_throttle_as_high_end_pct();   // throttle hover thrust value, 0.0 - 1.0

    // sanity check YAW_SV_ANGLE parameter value to avoid divide by zero
    _yaw_servo_angle_max_deg = constrain_float(_yaw_servo_angle_max_deg, AP_MOTORS_TRI_SERVO_RANGE_DEG_MIN, AP_MOTORS_TRI_SERVO_RANGE_DEG_MAX);

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in * get_compensation_gain();
    yaw_thrust = _yaw_in * get_compensation_gain()*sinf(radians(_yaw_servo_angle_max_deg)); // we scale this so a thrust request of 1.0f will ask for full servo deflection at full rear throttle
    throttle_thrust = get_throttle() * get_compensation_gain();

    // calculate angle of yaw pivot
    _pivot_angle = safe_asin(yaw_thrust);
    if (fabsf(_pivot_angle) > radians(_yaw_servo_angle_max_deg)) {
        limit.yaw = true;
        _pivot_angle = constrain_float(_pivot_angle, -radians(_yaw_servo_angle_max_deg), radians(_yaw_servo_angle_max_deg));
    }

    float pivot_thrust_max = cosf(_pivot_angle);
    float thrust_max = 1.0f;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    throttle_thrust_rpy_mix = MAX(throttle_thrust, throttle_thrust*MAX(0.0f,1.0f-_throttle_rpy_mix)+throttle_thrust_hover*_throttle_rpy_mix);

    // The following mix may be offer less coupling between axis but needs testing
    //_thrust_right = roll_thrust * -0.5f + pitch_thrust * 1.0f;
    //_thrust_left = roll_thrust * 0.5f + pitch_thrust * 1.0f;
    //_thrust_rear = 0;

    _thrust_right = roll_thrust * -0.5f + pitch_thrust * 0.5f;
    _thrust_left = roll_thrust * 0.5f + pitch_thrust * 0.5f;
    _thrust_rear = pitch_thrust * -0.5f;

    // calculate roll and pitch for each motor
    // set rpy_low and rpy_high to the lowest and highest values of the motors

    // record lowest roll pitch command
    rpy_low = MIN(_thrust_right,_thrust_left);
    rpy_high = MAX(_thrust_right,_thrust_left);
    if (rpy_low > _thrust_rear){
        rpy_low = _thrust_rear;
    }
    // check to see if the rear motor will reach maximum thrust before the front two motors
    if ((1.0f - rpy_high) > (pivot_thrust_max - _thrust_rear)){
        thrust_max = pivot_thrust_max;
        rpy_high = _thrust_rear;
    }

    // calculate throttle that gives most possible room for yaw (range 1000 ~ 2000) which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible room margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // check everything fits
    throttle_thrust_best_rpy = MIN(0.5f*thrust_max - (rpy_low+rpy_high)/2.0, throttle_thrust_rpy_mix);
    if(is_zero(rpy_low)){
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-throttle_thrust_best_rpy/rpy_low, 0.0f, 1.0f);
    }

    // calculate how close the motors can come to the desired throttle
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if(rpy_scale < 1.0f){
        // Full range is being used by roll, pitch, and yaw.
        limit.roll_pitch = true;
        if (thr_adj > 0.0f){
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    }else{
        if(thr_adj < -(throttle_thrust_best_rpy+rpy_low)){
            // Throttle can't be reduced to desired value
            thr_adj = -(throttle_thrust_best_rpy+rpy_low);
        }else if(thr_adj > thrust_max - (throttle_thrust_best_rpy+rpy_high)){
            // Throttle can't be increased to desired value
            thr_adj = thrust_max - (throttle_thrust_best_rpy+rpy_high);
            limit.throttle_upper = true;
        }
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    _thrust_right = throttle_thrust_best_rpy+thr_adj + rpy_scale*_thrust_right;
    _thrust_left = throttle_thrust_best_rpy+thr_adj + rpy_scale*_thrust_left;
    _thrust_rear = throttle_thrust_best_rpy+thr_adj + rpy_scale*_thrust_rear;

    // scale pivot thrust to account for pivot angle
    // we should not need to check for divide by zero as _pivot_angle is constrained to the 5deg ~ 80 deg range
    _thrust_rear = _thrust_rear/cosf(_pivot_angle);

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);
    _thrust_rear = constrain_float(_thrust_rear, 0.0f, 1.0f);
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
int16_t AP_MotorsTri::calc_yaw_radio_output(float yaw_input, float yaw_input_max)
{
    int16_t ret;

    if (_yaw_reverse < 0) {
        yaw_input = -yaw_input;
    }

    if (yaw_input >= 0){
        ret = (_yaw_servo_trim + (yaw_input/yaw_input_max * (_yaw_servo_max - _yaw_servo_trim)));
    } else {
        ret = (_yaw_servo_trim + (yaw_input/yaw_input_max * (_yaw_servo_trim - _yaw_servo_min)));
    }

    return ret;
}

/*
  call vehicle supplied thrust compensation if set. This allows for
  vehicle specific thrust compensation for motor arrangements such as
  the forward motors tilting
*/
void AP_MotorsTri::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        // convert 3 thrust values into an array indexed by motor number
        float thrust[4] { _thrust_right, _thrust_left, 0, _thrust_rear };

        // apply vehicle supplied compensation function
        _thrust_compensation_callback(thrust, 4);

        // extract compensated thrust values
        _thrust_right = thrust[0];
        _thrust_left  = thrust[1];
        _thrust_rear  = thrust[3];
    }
}
