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
            break;
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            _throttle = constrain_float(_spin_up_ratio, 0.0f, 1.0f) * _spin_min;
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            throttle_left  = constrain_float(_throttle + _rudder*0.5, 0, 1);
            throttle_right = constrain_float(_throttle - _rudder*0.5, 0, 1);
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
    _aileron = -_yaw_in;
    _elevator = _pitch_in;
    _rudder = _roll_in;
    _throttle = get_throttle();

    // sanity check throttle is above zero and below current limited throttle
    if (_throttle <= 0.0f) {
        _throttle = 0.0f;
        limit.throttle_lower = true;
    }
    if (_throttle >= _throttle_thrust_max) {
        _throttle = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    _throttle = constrain_float(_throttle, 0.1, 1);
}

