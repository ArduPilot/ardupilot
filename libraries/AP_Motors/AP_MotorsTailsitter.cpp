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
 *       AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500
#define THROTTLE_RANGE       100

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TAILSITTER);
}


/// Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t loop_rate, uint16_t speed_hz) :
    AP_MotorsMulticopter(loop_rate, speed_hz)
{
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!_flags.initialised_ok) {
        return;
    }
    float throttle_left  = 0;
    float throttle_right = 0;
    
    switch (_spool_mode) {
        case SHUT_DOWN:
            _throttle = 0;
            // set limits flags
            limit.roll_pitch = true;
            limit.yaw = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;
            break;
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            _throttle = constrain_float(_spin_up_ratio, 0.0f, 1.0f) * _spin_min;
            // set limits flags
            limit.roll_pitch = true;
            limit.yaw = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            throttle_left  = constrain_float(_throttle + _rudder*0.5, 0, 1);
            throttle_right = constrain_float(_throttle - _rudder*0.5, 0, 1);
            // initialize limits flags
            limit.roll_pitch = false;
            limit.yaw = false;
            limit.throttle_lower = false;
            limit.throttle_upper = false;
            break;
    }
    // outputs are setup here, and written to the HAL by the plane servos loop
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,  _aileron*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, _elevator*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder,   _rudder*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _throttle*THROTTLE_RANGE);

    // also support differential roll with twin motors
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  throttle_left*THROTTLE_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle_right*THROTTLE_RANGE);

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
#endif
}

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   rpy_thrust;                 // the minimum throttle setting that will not limit the roll pitch and yaw output
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    // roll, pitch, yaw should only be scaled by air density
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

    rpy_thrust = MAX(MAX(fabsf(roll_thrust), fabsf(pitch_thrust)), fabsf(yaw_thrust));

    thr_adj = throttle_thrust - _throttle_avg_max;
    if (thr_adj < (rpy_thrust - _throttle_avg_max)) {
        // Throttle can't be reduced to the desired level because this would mean roll or pitch control
        // would not be able to reach the desired level because of lack of thrust.
        thr_adj = MIN(rpy_thrust, _throttle_avg_max) - _throttle_avg_max;
    }

    // calculate the throttle setting for the lift fan
    _throttle = _throttle_avg_max + thr_adj;

    if (is_zero(_throttle)) {
        limit.roll_pitch = true;
        limit.yaw = true;
    }

    // limit thrust out for calculation of actuator gains
    float thrust_out_actuator = constrain_float(MAX(_throttle_hover*0.5f,_throttle), 0.1f, 1.0f);

    _aileron = -constrain_float(yaw_thrust/thrust_out_actuator, -1.0f, 1.0f);
    _elevator = constrain_float(pitch_thrust/thrust_out_actuator, -1.0f, 1.0f);
    _rudder = constrain_float(roll_thrust/thrust_out_actuator, -1.0f, 1.0f);

    // sanity check throttle is above zero and below current limited throttle
    if (_throttle <= 0.0f) {
        _throttle = 0.0f;
        limit.throttle_lower = true;
    }
    if (_throttle >= _throttle_thrust_max) {
        _throttle = _throttle_thrust_max;
        limit.throttle_upper = true;
    }
}

