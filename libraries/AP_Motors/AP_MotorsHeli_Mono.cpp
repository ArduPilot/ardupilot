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
 *       AP_MotorsHeli_Mono.cpp - ArduCopter motors library for monocopters (all rotating with single wing or wings)
 *
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS_View.h>

#include "AP_MotorsHeli_Mono.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Mono::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // Indices 1-3 were used by RSC_PWM_MIN, RSC_PWM_MAX and RSC_PWM_REV and should not be used

    // @Param: PHZ_OFFST
    // @DisplayName: Monocopter phase offset
    // @Description: Angular offset of attitude correction commands
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("PHZ_OFFST", 1, AP_MotorsHeli_Mono, _phase_offset, 0.0f),

    // @Param: HOVER_RPM
    // @DisplayName: Monocopter hover rpm
    // @Description: Hover rpm, used to scale gain
    // @Range: 0 1000
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("HOVER_RPM", 2, AP_MotorsHeli_Mono, _hover_rpm, 180.0f),

    // @Param: SYNC_PIN
    // @DisplayName: pin to sync with zero crossing of yaw and virtual yaw
    // @Description: gives a rising output when monocopter is yaw forwards, used for orientation LED, falling ouput at yaw 180
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,57:BB Blue GP0 pin 3,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1/BB Blue GP0 pin 6,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2/BB Blue GP0 pin 5
    AP_GROUPINFO("SYNC_PIN", 3, AP_MotorsHeli_Mono, _sync_pin, -1),

    // @Param: SYNC_PHZ
    // @DisplayName: phase offset for sync pulse
    // @Description: used to account for any offset in led mounting from forwards direction
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("SYNC_PHZ", 4, AP_MotorsHeli_Mono, _sync_phase_offset, 0.0f),

    AP_GROUPEND
};

#define SERVO_OUTPUT_RANGE  4500

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Mono::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor1, speed_hz);
}

// init
bool AP_MotorsHeli_Mono::init_outputs()
{
    if (_flags.initialised_ok) {
        return true;
    }

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_2);
    SRV_Channels::set_angle(SRV_Channel::k_motor1, SERVO_OUTPUT_RANGE);

    // set rotor servo range
    _rotor.init_servo();

    _flags.initialised_ok = true;

    // init sync pulse pin
    if (_sync_pin != -1) {
        sync_on = true;
        hal.gpio->pinMode(_sync_pin, HAL_GPIO_OUTPUT);
    }

    return true;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Mono::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // throttle
            rc_write(AP_MOTORS_HELI_DUAL_RSC, pwm);

            break;
        case 2:
            // 'aileron'
           rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

// set_desired_rotor_speed
void AP_MotorsHeli_Mono::set_desired_rotor_speed(float desired_speed)
{
    _rotor.set_desired_speed(desired_speed);
}

// calculate_armed_scalars
void AP_MotorsHeli_Mono::calculate_armed_scalars()
{
    float thrcrv[5];
    for (uint8_t i = 0; i < 5; i++) {
        thrcrv[i]=_rsc_thrcrv[i]*0.001f;
    }
    _rotor.set_ramp_time(_rsc_ramp_time);
    _rotor.set_runup_time(_rsc_runup_time);
    _rotor.set_critical_speed(_rsc_critical*0.001f);
    _rotor.set_idle_output(_rsc_idle_output*0.001f);
    _rotor.set_throttle_curve(thrcrv, (uint16_t)_rsc_slewrate.get());
}

// calculate_scalars
void AP_MotorsHeli_Mono::calculate_scalars()
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
    calculate_armed_scalars();
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeli_Mono::get_motor_mask()
{
    uint32_t mask = 0;

    mask |= 1U << AP_MOTORS_MOT_1;
    mask |= 1U << CH_1;

    return mask;
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Mono::update_motor_control(RotorControlState state)
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

// calculate output to elevator
void AP_MotorsHeli_Mono::move_actuators(float roll_out, float pitch_out, float collective_in, float yaw_out)
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
    if (_heliflags.landing_collective && collective_out < (_land_collective_min*0.001f)) {
        collective_out = _land_collective_min*0.001f;
        limit.throttle_lower = true;
    }

    float collective_range = (_collective_max - _collective_min)*0.001f;

    // feed power estimate into main rotor controller
    _rotor.set_collective(fabsf(collective_out));

    // scale collective to -1 to 1
    collective_out = collective_out*2-1;

    // reserve some collective for attitude control
    collective_out *= collective_range;

    // rotate virtual forward
    // as controller targets rates we must convert back to angle
    // with PID's of zero and ff of 1 this should give a 1:1 match of desired to actual
    const uint64_t now = AP_HAL::micros64();
    _last_update_us = now;

    // give the AHRS unspiner the desired yaw rate and get the current rotation relative to virtual forwards
    const float yaw_angle = _ahrs.rotation_angle;
    // set yaw rate in cds
    _ahrs.set_yaw_rate(_yaw_in * SERVO_OUTPUT_RANGE);

    /*
    Calculate 'Cyclic' output
    offset to account for servo response time and other forces can be done via AHRS rotation angles
    aerodynamically it should be zero, when also considering gyroscopic forces it should be +-90
    depending on craft rotation direction, probably it will be somewhere between
    */
    const float desired_travel_direction = atan2f(_roll_in,_pitch_in); //atan2f(_pitch_in,_roll_in) + radians(180);
    const float desired_travel_magnitude = sqrt(powf(_roll_in,2.0f) + powf(_pitch_in,2.0f));
    const float cyclic = cosf(wrap_PI(yaw_angle - desired_travel_direction + radians(_phase_offset)));

    /*
     apply 'Cyclic' output to aileron
     maybe also the motor?
     maybe should do some speed scaling based on rotation rate?
    */
    _aileron = cyclic * desired_travel_magnitude;

    // Apply collective
    _aileron += collective_out;

    // rotation rate scaling
    const float rpm = _ahrs.rpm;
    float scaleing = 1.5f;
    if (!is_zero(rpm)) {
        scaleing = MIN(scaleing,_hover_rpm / rpm);
    }

    _aileron = constrain_float(_aileron * scaleing,-1.0f,1.0f);

    /*
    output to LED sync pin for orientation
    would quite like to do this at more than 400hz, if rotating fast can be 10's of degrees off
    may reach EKF or aileron servo speed limit first so it's OK for the time being
    */
    if (sync_on) {
        const float sync = wrap_PI(yaw_angle + _sync_phase_offset);
        if (is_positive(sync)) {
            hal.gpio->write(_sync_pin, 1);
        } else {
            hal.gpio->write(_sync_pin, 0);
        }
    }
}

void AP_MotorsHeli_Mono::output_to_motors()
{
    if (!_flags.initialised_ok) {
        return;
    }

    rc_write_angle(AP_MOTORS_MOT_1, _aileron * SERVO_OUTPUT_RANGE);

    switch (_spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            update_motor_control(ROTOR_CONTROL_STOP);
            break;
        case GROUND_IDLE:
            // sends idle output to motors when armed. rotor could be static or turning (autorotation)
            update_motor_control(ROTOR_CONTROL_IDLE);
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
            // set motor output based on thrust requests
            update_motor_control(ROTOR_CONTROL_ACTIVE);
            break;
        case SPOOL_DOWN:
            // sends idle output to motors and wait for rotor to stop
            update_motor_control(ROTOR_CONTROL_IDLE);
            break;
    }
}

// servo_test - move servos through full range of movement
void AP_MotorsHeli_Mono::servo_test()
{
    // not implemented
}
