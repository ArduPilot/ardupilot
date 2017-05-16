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
//    float   elevon_thrust;              // demand for elevon thrust

    // apply voltage and air pressure compensation
    // roll, pitch, yaw should only be scaled by air density
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in * get_compensation_gain();
    yaw_thrust = _yaw_in * get_compensation_gain();
    throttle_thrust = get_throttle() * get_compensation_gain();

//    elevon_thrust = MAX(fabsf(pitch_thrust), fabsf(yaw_thrust));
//
//    // boost throttle to reduce elevon thrust demand
//    throttle_thrust *= (1.0f + elevon_thrust);
//
//    // override boost at low throttle demand (for safety)
//    if (get_throttle() < (0.5f * _throttle_hover)) {
//        throttle_thrust *= (get_throttle() / (0.5f * _throttle_hover));
//    }

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    // _throttle_thrust_max seems to be set to 1.0, leaving no headroom for differential thrust
    // make sure we have at least a 10% margin
    if (throttle_thrust >= MIN(_throttle_thrust_max, 0.9f)) {
        throttle_thrust = MIN(_throttle_thrust_max, 0.9f);
        limit.throttle_upper = true;
    }
    if (is_zero(throttle_thrust)) {
        limit.roll_pitch = true;
        limit.yaw = true;
    }

//    // temporary debug logging
//    static int dec_count=0;
//    if (dec_count++ >= 3) {
//        dec_count = 0;
//        DataFlash_Class::instance()->Log_Write("TCOMP", "TimeUS,Rthr,Pthr,Ythr,Tthr,Ethr", "Qfffff",
//                                               AP_HAL::micros64(),
//                                               roll_thrust, pitch_thrust, yaw_thrust,
//                                               throttle_thrust, elevon_thrust);
//    }
//
    // in tailsitter (nose-up) config, aileron controls earth-frame yaw (reversed)
    _aileron = -yaw_thrust;
    // and rudder controls earth-frame roll
    _rudder = roll_thrust;
    // elevator and throttle controls are unchanged
    _elevator = pitch_thrust;
    _throttle = throttle_thrust;
}

