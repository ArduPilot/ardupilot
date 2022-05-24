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

#if AP_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrix_6DoF_Scripting.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

void AP_MotorsMatrix_6DoF_Scripting::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
        case SpoolState::GROUND_IDLE:
        {
            // no output, cant spin up for ground idle because we don't know which way motors should be spining
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    _actuator[i] = 0.0f;
                }
            }
            break;
        }
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    if (_reversible[i]) {
                        // revesible motor can provide both positive and negative thrust, +- spin max, spin min does not apply
                        if (is_positive(_thrust_rpyt_out[i])) { 
                            _actuator[i] = apply_thrust_curve_and_volt_scaling(_thrust_rpyt_out[i]) * _spin_max;

                        } else if (is_negative(_thrust_rpyt_out[i])) {
                            _actuator[i] = -apply_thrust_curve_and_volt_scaling(-_thrust_rpyt_out[i]) * _spin_max;

                        } else {
                            _actuator[i] = 0.0f;
                        }
                    } else {
                        // motor can only provide trust in a single direction, spin min to spin max as 'normal' copter
                         _actuator[i] = thrust_to_actuator(_thrust_rpyt_out[i]);
                    }
                }
            }
            break;
    }

    // Send to each motor
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _actuator[i] * 4500);
        }
    }
}

// output_armed - sends commands to the motors
void AP_MotorsMatrix_6DoF_Scripting::output_armed_stabilizing()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   right_thrust;               // right thrust input value, +/- 1.0

    // note that the throttle, forwards and right inputs are not in bodyframe, they are in the frame of the 'normal' 4DoF copter were pretending to be

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain(); // compensation for battery voltage and altitude
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;

    // scale horizontal thrust with throttle, this mimics a normal copter
    // so we don't break the lean angle proportional acceleration assumption made by the position controller
    forward_thrust = get_forward() * throttle_thrust;
    right_thrust = get_lateral() * throttle_thrust;


    // set throttle limit flags
    if (throttle_thrust <= 0) {
        throttle_thrust = 0;
        // we cant thrust down, the vehicle can do it, but it would break a lot of assumptions further up the control stack
        // 1G decent probably plenty anyway....
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= 1) {
        throttle_thrust = 1;
        limit.throttle_upper = true;
    }

    // rotate the thrust into bodyframe
    Matrix3f rot;
    Vector3f thrust_vec;
    rot.from_euler312(_roll_offset, _pitch_offset, 0.0f);


    /*
        upwards thrust, independent of orientation
    */
    thrust_vec.x = 0.0f;
    thrust_vec.y = 0.0f;
    thrust_vec.z = throttle_thrust;
    thrust_vec = rot * thrust_vec;
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] =  thrust_vec.x * _forward_factor[i];
            _thrust_rpyt_out[i] += thrust_vec.y * _right_factor[i];
            _thrust_rpyt_out[i] += thrust_vec.z * _throttle_factor[i];

            if (fabsf(_thrust_rpyt_out[i]) >= 1) {
                // if we hit this the mixer is probably scaled incorrectly
                limit.throttle_upper = true;
            }
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i],-1.0f,1.0f);
        }
    }


    /*
        rotations: roll, pitch and yaw
    */
    float rpy_ratio = 1.0f;  // scale factor, output will be scaled by this ratio so it can all fit evenly
    float thrust[AP_MOTORS_MAX_NUM_MOTORS];
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            thrust[i] =  roll_thrust * _roll_factor[i];
            thrust[i] += pitch_thrust * _pitch_factor[i];
            thrust[i] += yaw_thrust * _yaw_factor[i];
            float total_thrust = _thrust_rpyt_out[i] + thrust[i];
            // control input will be limited by motor range
            if (total_thrust > 1.0f) {
                rpy_ratio = MIN(rpy_ratio,(1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            } else if (total_thrust < -1.0f) {
                rpy_ratio = MIN(rpy_ratio,(-1.0f -_thrust_rpyt_out[i]) / thrust[i]);
            }
        }
    }

    // set limit flags if output is being scaled
    if (rpy_ratio < 1) {
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
    }

    // scale back rotations evenly so it will all fit
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i] + thrust[i] * rpy_ratio,-1.0f,1.0f);
        }
    }

    /*
        forward and lateral, independent of orentaiton
    */
    thrust_vec.x = forward_thrust;
    thrust_vec.y = right_thrust;
    thrust_vec.z = 0.0f;
    thrust_vec = rot * thrust_vec;

    float horz_ratio = 1.0f; 
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            thrust[i] =  thrust_vec.x * _forward_factor[i];
            thrust[i] += thrust_vec.y * _right_factor[i];
            thrust[i] += thrust_vec.z * _throttle_factor[i];
            float total_thrust = _thrust_rpyt_out[i] + thrust[i];
            // control input will be limited by motor range
            if (total_thrust > 1.0f) {
                horz_ratio = MIN(horz_ratio,(1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            } else if (total_thrust < -1.0f) {
                horz_ratio = MIN(horz_ratio,(-1.0f -_thrust_rpyt_out[i]) / thrust[i]);
            }
        }
    }

    // scale back evenly so it will all fit
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i] + thrust[i] * horz_ratio,-1.0f,1.0f);
        }
    }

    /*
        apply deadzone to revesible motors, this stops motors from reversing direction too often
        re-use yaw headroom param for deadzone, constain to a max of 25%
    */
    const float deadzone = constrain_float(_yaw_headroom.get() * 0.001f,0.0f,0.25f);
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _reversible[i]) {
            if (is_negative(_thrust_rpyt_out[i])) {
                if ((_thrust_rpyt_out[i] > -deadzone) && is_positive(_last_thrust_out[i])) {
                    _thrust_rpyt_out[i] = 0.0f;
                } else {
                    _last_thrust_out[i] = _thrust_rpyt_out[i];
                }
            } else if (is_positive(_thrust_rpyt_out[i])) {
                if ((_thrust_rpyt_out[i] < deadzone) && is_negative(_last_thrust_out[i])) {
                    _thrust_rpyt_out[i] = 0.0f;
                } else {
                    _last_thrust_out[i] = _thrust_rpyt_out[i];
                }
            }
        }
    }

}

// sets the roll and pitch offset, this rotates the thrust vector in body frame
// these are typically set such that the throttle thrust vector is earth frame up
void AP_MotorsMatrix_6DoF_Scripting::set_roll_pitch(float roll_deg, float pitch_deg)
{
    _roll_offset = radians(roll_deg);
    _pitch_offset = radians(pitch_deg);
}

// add_motor, take roll, pitch, yaw, throttle(up), forward, right factors along with a bool if the motor is reversible and the testing order, called from scripting
void AP_MotorsMatrix_6DoF_Scripting::add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order)
{
    if (initialised_ok()) {
        // don't allow matrix to be changed after init
        return;
    }

    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        motor_enabled[motor_num] = true;

        _roll_factor[motor_num] = roll_factor;
        _pitch_factor[motor_num] = pitch_factor;
        _yaw_factor[motor_num] = yaw_factor;

        _throttle_factor[motor_num] = throttle_factor;
        _forward_factor[motor_num] = forward_factor;
        _right_factor[motor_num] = right_factor;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // ensure valid motor number is provided
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);

        uint8_t chan;
        if (!SRV_Channels::find_channel(function, chan)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
            return;
        }

        _reversible[motor_num] = reversible;
        if (_reversible[motor_num]) {
            // reversible, set to angle type hard code trim to 1500
            SRV_Channels::set_angle(function, 4500);
            SRV_Channels::set_trim_to_pwm_for(function, 1500);
        } else {
            SRV_Channels::set_range(function, 4500);
        }
        SRV_Channels::set_output_min_max(function, get_pwm_output_min(), get_pwm_output_max());
    }
}

bool AP_MotorsMatrix_6DoF_Scripting::init(uint8_t expected_num_motors) {
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    set_initialised_ok(expected_num_motors == num_motors);

    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    switch (num_motors) {
        case 3:
            _mav_type = MAV_TYPE_TRICOPTER;
            break;
        case 4:
            _mav_type = MAV_TYPE_QUADROTOR;
            break;
        case 6:
            _mav_type = MAV_TYPE_HEXAROTOR;
            break;
        case 8:
            _mav_type = MAV_TYPE_OCTOROTOR;
            break;
        case 10:
            _mav_type = MAV_TYPE_DECAROTOR;
            break;
        case 12:
            _mav_type = MAV_TYPE_DODECAROTOR;
            break;
        default:
            _mav_type = MAV_TYPE_GENERIC;
    }

    return true;
}

// singleton instance
AP_MotorsMatrix_6DoF_Scripting *AP_MotorsMatrix_6DoF_Scripting::_singleton;

#endif // AP_SCRIPTING_ENABLED
