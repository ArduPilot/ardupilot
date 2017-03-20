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
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;


// init
void AP_MotorsCoax::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    _servo1 = SRV_Channels::get_channel_for(SRV_Channel::k_motor1, CH_1);
    _servo2 = SRV_Channels::get_channel_for(SRV_Channel::k_motor2, CH_2);
    _servo3 = SRV_Channels::get_channel_for(SRV_Channel::k_motor3, CH_3);
    _servo4 = SRV_Channels::get_channel_for(SRV_Channel::k_motor4, CH_4);
    if (!_servo1 || !_servo2 || !_servo3 || !_servo4) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "MotorsCoax: unable to setup output channels");
        // don't set initialised_ok
        return;
    }
    
    // set the motor_enabled flag so that the main ESC can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;

    // we set four servos to angle
    _servo1->set_angle(AP_MOTORS_COAX_SERVO_INPUT_RANGE);
    _servo2->set_angle(AP_MOTORS_COAX_SERVO_INPUT_RANGE);
    _servo3->set_angle(AP_MOTORS_COAX_SERVO_INPUT_RANGE);
    _servo4->set_angle(AP_MOTORS_COAX_SERVO_INPUT_RANGE);

    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_COAX);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsCoax::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_COAX);
}

// set update rate to motors - a value in hertz
void AP_MotorsCoax::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    uint32_t mask =
        1U << AP_MOTORS_MOT_5 |
        1U << AP_MOTORS_MOT_6 ;
    rc_set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsCoax::enable()
{
    // enable output channels
    rc_enable_ch(AP_MOTORS_MOT_1);
    rc_enable_ch(AP_MOTORS_MOT_2);
    rc_enable_ch(AP_MOTORS_MOT_3);
    rc_enable_ch(AP_MOTORS_MOT_4);
    rc_enable_ch(AP_MOTORS_MOT_5);
    rc_enable_ch(AP_MOTORS_MOT_6);
}

void AP_MotorsCoax::output_to_motors()
{
    switch (_spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            hal.rcout->cork();
            rc_write(AP_MOTORS_MOT_1, calc_pwm_output_1to1(_roll_radio_passthrough, _servo1));
            rc_write(AP_MOTORS_MOT_2, calc_pwm_output_1to1(_pitch_radio_passthrough, _servo2));
            rc_write(AP_MOTORS_MOT_3, calc_pwm_output_1to1(-_roll_radio_passthrough, _servo3));
            rc_write(AP_MOTORS_MOT_4, calc_pwm_output_1to1(-_pitch_radio_passthrough, _servo4));
            rc_write(AP_MOTORS_MOT_5, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_6, get_pwm_output_min());
            hal.rcout->push();
            break;
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            hal.rcout->cork();
            rc_write(AP_MOTORS_MOT_1, calc_pwm_output_1to1(_spin_up_ratio * _actuator_out[0], _servo1));
            rc_write(AP_MOTORS_MOT_2, calc_pwm_output_1to1(_spin_up_ratio * _actuator_out[1], _servo2));
            rc_write(AP_MOTORS_MOT_3, calc_pwm_output_1to1(_spin_up_ratio * _actuator_out[2], _servo3));
            rc_write(AP_MOTORS_MOT_4, calc_pwm_output_1to1(_spin_up_ratio * _actuator_out[3], _servo4));
            rc_write(AP_MOTORS_MOT_5, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_MOT_6, calc_spin_up_to_pwm());
            hal.rcout->push();
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            hal.rcout->cork();
            rc_write(AP_MOTORS_MOT_1, calc_pwm_output_1to1(_actuator_out[0], _servo1));
            rc_write(AP_MOTORS_MOT_2, calc_pwm_output_1to1(_actuator_out[1], _servo2));
            rc_write(AP_MOTORS_MOT_3, calc_pwm_output_1to1(_actuator_out[2], _servo3));
            rc_write(AP_MOTORS_MOT_4, calc_pwm_output_1to1(_actuator_out[3], _servo4));
            rc_write(AP_MOTORS_MOT_5, calc_thrust_to_pwm(_thrust_yt_ccw));
            rc_write(AP_MOTORS_MOT_6, calc_thrust_to_pwm(_thrust_yt_cw));
            hal.rcout->push();
            break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsCoax::get_motor_mask()
{
    uint32_t mask =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4 |
        1U << AP_MOTORS_MOT_5 |
        1U << AP_MOTORS_MOT_6;
    return rc_map_mask(mask);
}

// sends commands to the motors
void AP_MotorsCoax::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   thrust_min_rpy;             // the minimum throttle setting that will not limit the roll and pitch output
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    float   thrust_out;                 //
    float   rp_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   actuator_allowed = 0.0f;    // amount of yaw we can fit in

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in * get_compensation_gain();
    yaw_thrust = _yaw_in * get_compensation_gain();
    throttle_thrust = get_throttle() * get_compensation_gain();

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    _throttle_avg_max = constrain_float(_throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    float rp_thrust_max = MAX(fabsf(roll_thrust), fabsf(pitch_thrust));

    // calculate how much roll and pitch must be scaled to leave enough range for the minimum yaw
    if (is_zero(rp_thrust_max)) {
        rp_scale = 1.0f;
    } else {
        rp_scale = constrain_float((1.0f - MIN(fabsf(yaw_thrust), 0.5f*(float)_yaw_headroom/1000.0f)) / rp_thrust_max, 0.0f, 1.0f);
        if (rp_scale < 1.0f) {
            limit.roll_pitch = true;
        }
    }

    actuator_allowed = 2.0f * (1.0f - rp_scale * rp_thrust_max);
    if (fabsf(yaw_thrust) > actuator_allowed) {
        yaw_thrust = constrain_float(yaw_thrust, -actuator_allowed, actuator_allowed);
        limit.yaw = true;
    }

    // calculate the minimum thrust that doesn't limit the roll, pitch and yaw forces
    thrust_min_rpy = MAX(fabsf(rp_scale * rp_thrust_max), fabsf(yaw_thrust));

    thr_adj = throttle_thrust - _throttle_avg_max;
    if (thr_adj < (thrust_min_rpy - _throttle_avg_max)) {
        // Throttle can't be reduced to the desired level because this would mean roll or pitch control
        // would not be able to reach the desired level because of lack of thrust.
        thr_adj = MIN(thrust_min_rpy, _throttle_avg_max) - _throttle_avg_max;
    }

    // calculate the throttle setting for the lift fan
    thrust_out = _throttle_avg_max + thr_adj;

    if (fabsf(yaw_thrust) > thrust_out) {
        yaw_thrust = constrain_float(yaw_thrust, -thrust_out, thrust_out);
        limit.yaw = true;
    }

    _thrust_yt_ccw = thrust_out + 0.5f * yaw_thrust;
    _thrust_yt_cw = thrust_out - 0.5f * yaw_thrust;

    // limit thrust out for calculation of actuator gains
    float thrust_out_actuator = constrain_float(MAX(_throttle_hover*0.5f,thrust_out), 0.1f, 1.0f);

    if (is_zero(thrust_out)) {
        limit.roll_pitch = true;
    }
    // force of a lifting surface is approximately equal to the angle of attack times the airflow velocity squared
    // static thrust is proportional to the airflow velocity squared
    // therefore the torque of the roll and pitch actuators should be approximately proportional to
    // the angle of attack multiplied by the static thrust.
    _actuator_out[0] = roll_thrust/thrust_out_actuator;
    _actuator_out[1] = pitch_thrust/thrust_out_actuator;
    if (fabsf(_actuator_out[0]) > 1.0f) {
        limit.roll_pitch = true;
        _actuator_out[0] = constrain_float(_actuator_out[0], -1.0f, 1.0f);
    }
    if (fabsf(_actuator_out[1]) > 1.0f) {
        limit.roll_pitch = true;
        _actuator_out[1] = constrain_float(_actuator_out[1], -1.0f, 1.0f);
    }
    _actuator_out[2] = -_actuator_out[0];
    _actuator_out[3] = -_actuator_out[1];
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
            // flap servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // flap servo 4
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // motor 1
            rc_write(AP_MOTORS_MOT_5, pwm);
            break;
        case 6:
            // motor 2
            rc_write(AP_MOTORS_MOT_6, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
