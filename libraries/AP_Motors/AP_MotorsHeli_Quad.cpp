/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_MotorsHeli_Quad.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Quad::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // Indices 1-3 were used by RSC_PWM_MIN, RSC_PWM_MAX and RSC_PWM_REV and should not be used

    AP_GROUPEND
};

#define QUAD_SERVO_MAX_ANGLE 4500

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Quad::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        mask |= 1U << (AP_MOTORS_MOT_1+i);
    }

    rc_set_freq(mask, _speed_hz);
}

// init_outputs
bool AP_MotorsHeli_Quad::init_outputs()
{
    if (_flags.initialised_ok) {
        return true;
    }

    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        add_motor_num(CH_1+i);
        SRV_Channels::set_angle(SRV_Channels::get_motor_function(i), QUAD_SERVO_MAX_ANGLE);
    }

    // set rotor servo range
    _rotor.init_servo();

    _flags.initialised_ok = true;

    return true;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Quad::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
    case 1 ... AP_MOTORS_HELI_QUAD_NUM_MOTORS:
        rc_write(AP_MOTORS_MOT_1 + (motor_seq-1), pwm);
        break;
    case AP_MOTORS_HELI_QUAD_NUM_MOTORS+1:
        // main rotor
        rc_write(AP_MOTORS_HELI_QUAD_RSC, pwm);
        break;
    default:
        // do nothing
        break;
    }
}

// set_desired_rotor_speed
void AP_MotorsHeli_Quad::set_desired_rotor_speed(float desired_speed)
{
    _rotor.set_desired_speed(desired_speed);
}

// set_rotor_rpm - used for governor with speed sensor
void AP_MotorsHeli_Quad::set_rpm(float rotor_rpm)
{
    _rotor.set_rotor_rpm(rotor_rpm);
}

// calculate_armed_scalars
void AP_MotorsHeli_Quad::calculate_armed_scalars()
{
    // Set common RSC variables
    _rotor.set_ramp_time(_rsc_ramp_time);
    _rotor.set_runup_time(_rsc_runup_time);
    _rotor.set_critical_speed(_rsc_critical*0.001f);
    _rotor.set_idle_output(_rsc_idle_output*0.001f);
    _rotor.set_slewrate(_rsc_slewrate);

    // Set rsc mode specific parameters
    if (_rsc_mode == ROTOR_CONTROL_MODE_OPEN_LOOP_POWER_OUTPUT) {
        _rotor.set_throttle_curve(_rsc_thrcrv.get_thrcrv());
    } else if (_rsc_mode == ROTOR_CONTROL_MODE_CLOSED_LOOP_POWER_OUTPUT) {
        _rotor.set_throttle_curve(_rsc_thrcrv.get_thrcrv());
        _rotor.set_governor_disengage(_rsc_gov.get_disengage()*0.01f);
        _rotor.set_governor_droop_response(_rsc_gov.get_droop_response()*0.01f);
        _rotor.set_governor_reference(_rsc_gov.get_reference());
        _rotor.set_governor_range(_rsc_gov.get_range());
        _rotor.set_governor_thrcurve(_rsc_gov.get_thrcurve()*0.01f);
    }
}

// calculate_scalars
void AP_MotorsHeli_Quad::calculate_scalars()
{
    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min = AP_MOTORS_HELI_COLLECTIVE_MIN;
        _collective_max = AP_MOTORS_HELI_COLLECTIVE_MAX;
    }

    _collective_mid = constrain_int16(_collective_mid, _collective_min, _collective_max);

    // calculate collective mid point as a number from 0 to 1000
    _collective_mid_pct = ((float)(_collective_mid-_collective_min))/((float)(_collective_max-_collective_min));

    // calculate factors based on swash type and servo position
    calculate_roll_pitch_collective_factors();

    // set mode of main rotor controller and trigger recalculation of scalars
    _rotor.set_control_mode(static_cast<RotorControlMode>(_rsc_mode.get()));
    enable_rsc_parameters();
    calculate_armed_scalars();
}

// calculate_swash_factors - calculate factors based on swash type and servo position
void AP_MotorsHeli_Quad::calculate_roll_pitch_collective_factors()
{
    // assume X quad layout, with motors at 45, 135, 225 and 315 degrees
    // order FrontRight, RearLeft, FrontLeft, RearLeft
    const float angles[AP_MOTORS_HELI_QUAD_NUM_MOTORS] = { 45, 225, 315, 135 };
    const bool x_clockwise[AP_MOTORS_HELI_QUAD_NUM_MOTORS] = { false, false, true, true };
    const float cos45 = cosf(radians(45));

    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        bool clockwise = x_clockwise[i];
        if (_frame_type == MOTOR_FRAME_TYPE_H) {
            // reverse yaw for H frame
            clockwise = !clockwise;
        }
        _rollFactor[CH_1+i]       = -0.5*sinf(radians(angles[i]))/cos45;
        _pitchFactor[CH_1+i]      =  0.5*cosf(radians(angles[i]))/cos45;
        _yawFactor[CH_1+i]        = clockwise?-0.5:0.5;
        _collectiveFactor[CH_1+i] = 1;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeli_Quad::get_motor_mask()
{
    uint16_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        mask |= 1U << (AP_MOTORS_MOT_1+i);
    }
    mask |= 1U << AP_MOTORS_HELI_QUAD_RSC;
    return mask;
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Quad::update_motor_control(RotorControlState state)
{
    // Send state update to motors
    _rotor.output(state);

    if (state == ROTOR_CONTROL_STOP) {
        // set engine run enable aux output to not run position to kill engine when disarmed
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    } else {
        // else if armed, set engine run enable output to run position
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    }

    // Check if rotors are run-up
    _heliflags.rotor_runup_complete = _rotor.is_runup_complete();
}

//
// move_actuators - moves swash plate to attitude of parameters passed in
//                - expected ranges:
//                       roll : -1 ~ +1
//                       pitch: -1 ~ +1
//                       collective: 0 ~ 1
//                       yaw:   -1 ~ +1
//
void AP_MotorsHeli_Quad::move_actuators(float roll_out, float pitch_out, float collective_in, float yaw_out)
{
    // initialize limits flag
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // constrain collective input
    float collective_out = collective_in;
    if (collective_out <= 0.0f) {
        collective_out = 0.0f;
        limit.throttle_lower = true;
    }
    if (collective_out >= 1.0f) {
        collective_out = 1.0f;
        limit.throttle_upper = true;
    }

    // ensure not below landed/landing collective
    if (_heliflags.landing_collective && collective_out < _collective_mid_pct) {
        collective_out = _collective_mid_pct;
        limit.throttle_lower = true;
    }

    float collective_range = (_collective_max - _collective_min)*0.001f;

    if (_heliflags.inverted_flight) {
        collective_out = 1 - collective_out;
    }

    // feed power estimate into main rotor controller
    _rotor.set_collective(fabsf(collective_out));

    // scale collective to -1 to 1
    collective_out = collective_out*2-1;

    // reserve some collective for attitude control
    collective_out *= collective_range;

    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        _out[i] =
            _rollFactor[CH_1+i] * roll_out +
            _pitchFactor[CH_1+i] * pitch_out +
            _collectiveFactor[CH_1+i] * collective_out;
    }

    // see if we need to scale down yaw_out
    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        float y = _yawFactor[CH_1+i] * yaw_out;
        if (_out[i] < 0) {
            // the slope of the yaw effect changes at zero collective
            y = -y;
        }
        if (_out[i] * (_out[i] + y) < 0) {
            // applying this yaw demand would change the sign of the
            // collective, which means the yaw would not be applied
            // evenly. We scale down the overall yaw demand to prevent
            // it crossing over zero
            float s = -(_out[i] / y);
            yaw_out *= s;
        }
    }

    // now apply the yaw correction
    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        float y = _yawFactor[CH_1+i] * yaw_out;
        if (_out[i] < 0) {
            // the slope of the yaw effect changes at zero collective
            y = -y;
        }
        _out[i] += y;
    }

}

void AP_MotorsHeli_Quad::output_to_motors()
{
    if (!_flags.initialised_ok) {
        return;
    }

    // move the servos
    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        rc_write_angle(AP_MOTORS_MOT_1+i, _out[i] * QUAD_SERVO_MAX_ANGLE);
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            update_motor_control(ROTOR_CONTROL_STOP);
            break;
        case SpoolState::GROUND_IDLE:
            // sends idle output to motors when armed. rotor could be static or turning (autorotation)
            update_motor_control(ROTOR_CONTROL_IDLE);
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
            // set motor output based on thrust requests
            update_motor_control(ROTOR_CONTROL_ACTIVE);
            break;
        case SpoolState::SPOOLING_DOWN:
            // sends idle output to motors and wait for rotor to stop
            update_motor_control(ROTOR_CONTROL_IDLE);
            break;
    }
}

// servo_test - move servos through full range of movement
void AP_MotorsHeli_Quad::servo_test()
{
    // not implemented
}
