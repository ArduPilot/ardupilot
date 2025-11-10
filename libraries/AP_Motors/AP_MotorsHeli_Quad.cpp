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
#include <GCS_MAVLink/GCS.h>
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
void AP_MotorsHeli_Quad::init_outputs()
{
    if (initialised_ok()) {
        return;
    }

    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        add_motor_num(CH_1+i);
        SRV_Channels::set_angle(SRV_Channels::get_motor_function(i), QUAD_SERVO_MAX_ANGLE);
    }

    // set rotor servo range
    _main_rotor.init_servo();

    set_initialised_ok(_frame_class == MOTOR_FRAME_HELI_QUAD);
}

// calculate_armed_scalars
void AP_MotorsHeli_Quad::calculate_armed_scalars()
{
    // Set rsc mode specific parameters
    if (_main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_THROTTLECURVE || _main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_AUTOTHROTTLE) {
        _main_rotor.set_throttle_curve();
    }
    // keeps user from changing RSC mode while armed
    if (_main_rotor._rsc_mode.get() != _main_rotor.get_control_mode()) {
        _main_rotor.reset_rsc_mode_param();
        _heliflags.save_rsc_mode = true;
    }
    // saves rsc mode parameter when disarmed if it had been reset while armed
    if (_heliflags.save_rsc_mode && !armed()) {
        _main_rotor._rsc_mode.save();
        _heliflags.save_rsc_mode = false;
    }
}

// calculate_scalars
void AP_MotorsHeli_Quad::calculate_scalars()
{
    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min.set(AP_MOTORS_HELI_COLLECTIVE_MIN);
        _collective_max.set(AP_MOTORS_HELI_COLLECTIVE_MAX);
    }

    _collective_zero_thrust_deg.set(constrain_float(_collective_zero_thrust_deg, _collective_min_deg, _collective_max_deg));

    _collective_land_min_deg.set(constrain_float(_collective_land_min_deg, _collective_min_deg, _collective_max_deg));

    if (!is_equal((float)_collective_max_deg, (float)_collective_min_deg)) {
        // calculate collective zero thrust point as a number from 0 to 1
        _collective_zero_thrust_pct = (_collective_zero_thrust_deg-_collective_min_deg)/(_collective_max_deg-_collective_min_deg);

        // calculate collective land min point as a number from 0 to 1
        _collective_land_min_pct = (_collective_land_min_deg-_collective_min_deg)/(_collective_max_deg-_collective_min_deg);
    } else {
        _collective_zero_thrust_pct = 0.0f;
        _collective_land_min_pct = 0.0f;
    }

    // calculate factors based on swash type and servo position
    calculate_roll_pitch_collective_factors();

    // set mode of main rotor controller and trigger recalculation of scalars
    _main_rotor.set_control_mode(static_cast<RotorControlMode>(_main_rotor._rsc_mode.get()));
    calculate_armed_scalars();
}

// calculate_swash_factors - calculate factors based on swash type and servo position
void AP_MotorsHeli_Quad::calculate_roll_pitch_collective_factors()
{
    // assume X quad layout, with motors at 45, 135, 225 and 315 degrees
    // order FrontRight, RearLeft, FrontLeft, RearRight
    const float angles[AP_MOTORS_HELI_QUAD_NUM_MOTORS] = { 45, 225, 315, 135 };
    const bool x_clockwise[AP_MOTORS_HELI_QUAD_NUM_MOTORS] = { false, false, true, true };
    const float cos45 = cosf(radians(45));

    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        bool clockwise = x_clockwise[i];
        if (_frame_type == MOTOR_FRAME_TYPE_H) {
            // reverse yaw for H frame
            clockwise = !clockwise;
        }
        _rollFactor[CH_1+i]       = -0.25*sinf(radians(angles[i]))/cos45;
        _pitchFactor[CH_1+i]      =  0.25*cosf(radians(angles[i]))/cos45;
        _yawFactor[CH_1+i]        = clockwise?-0.25:0.25;
        _collectiveFactor[CH_1+i] = 1;
    }
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Quad::update_motor_control(AP_MotorsHeli_RSC::RotorControlState state)
{
    // Send state update to motors
    _main_rotor.output(state);

    if (state == AP_MotorsHeli_RSC::RotorControlState::STOP) {
        // set engine run enable aux output to not run position to kill engine when disarmed
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::MIN);
    } else {
        // else if armed, set engine run enable output to run position
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::MAX);
    }

    // Check if rotors are run-up
    set_rotor_runup_complete(_main_rotor.is_runup_complete());

    // Check if rotors are spooled down
    _heliflags.rotor_spooldown_complete = _main_rotor.is_spooldown_complete();
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
    if (_heliflags.landing_collective && collective_out < _collective_land_min_pct) {
        collective_out = _collective_land_min_pct;
        limit.throttle_lower = true;
    }

    // updates below land min collective flag
    _heliflags.below_land_min_coll = !is_positive(collective_out - _collective_land_min_pct);

    // updates takeoff collective flag based on 50% hover collective
    update_takeoff_collective_flag(collective_out);

    float collective_range = (_collective_max - _collective_min) * 0.001f;

    // feed power estimate into main rotor controller
    _main_rotor.set_collective(fabsf(collective_out));

    // rescale collective for overhead calc
    collective_out -= _collective_zero_thrust_pct;

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
        if (_out[i] < 0.0f) {
            // the slope of the yaw effect changes at zero collective
            y = -y;
        }
        if (_out[i] * (_out[i] + y) < 0.0f) {
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
        if (_out[i] < 0.0f) {
            // the slope of the yaw effect changes at zero collective
            y = -y;
        }
        _out[i] += y;
    }

    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        // scale output to 0 to 1
        _out[i] += _collective_zero_thrust_pct;
        // scale output to -1 to 1 for servo output
        _out[i] = _out[i] * 2.0f - 1.0f;
    }
}

void AP_MotorsHeli_Quad::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    // move the servos
    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        rc_write_angle(AP_MOTORS_MOT_1+i, _out[i] * QUAD_SERVO_MAX_ANGLE);
    }

    update_motor_control(get_rotor_control_state());

}

// servo_test - move servos through full range of movement
void AP_MotorsHeli_Quad::servo_test()
{
    // not implemented
}

#if HAL_LOGGING_ENABLED
// heli motors logging - called at 10 Hz
void AP_MotorsHeli_Quad::Log_Write(void)
{
    _main_rotor.write_log();
}
#endif
