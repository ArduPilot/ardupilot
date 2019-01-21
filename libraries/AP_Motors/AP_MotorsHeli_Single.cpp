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
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsHeli_Single.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Single::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // @Param: SV1_POS
    // @DisplayName: Servo 1 Position
    // @Description: Angular location of swash servo #1 - only used for H3 swash type
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV1_POS", 1, AP_MotorsHeli_Single, _servo1_pos, AP_MOTORS_HELI_SINGLE_SERVO1_POS),

    // @Param: SV2_POS
    // @DisplayName: Servo 2 Position
    // @Description: Angular location of swash servo #2 - only used for H3 swash type
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV2_POS", 2, AP_MotorsHeli_Single, _servo2_pos, AP_MOTORS_HELI_SINGLE_SERVO2_POS),

    // @Param: SV3_POS
    // @DisplayName: Servo 3 Position
    // @Description: Angular location of swash servo #3 - only used for H3 swash type
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV3_POS", 3, AP_MotorsHeli_Single, _servo3_pos, AP_MOTORS_HELI_SINGLE_SERVO3_POS),

    // @Param: TAIL_TYPE
    // @DisplayName: Tail Type
    // @Description: Tail type selection.  Simpler yaw controller used if external gyro is selected
    // @Values: 0:Servo only,1:Servo with ExtGyro,2:DirectDrive VarPitch,3:DirectDrive FixedPitch
    // @User: Standard
    AP_GROUPINFO("TAIL_TYPE", 4, AP_MotorsHeli_Single, _tail_type, AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO),

    // @Param: SWASH_TYPE
    // @DisplayName: Swash Type
    // @Description: Swash Type Setting
    // @Values: 0:H3 CCPM Adjustable, 1:H1 Straight Swash, 2:H3_140 CCPM
    // @User: Standard
    AP_GROUPINFO("SWASH_TYPE", 5, AP_MotorsHeli_Single, _swash_type, AP_MOTORS_HELI_SINGLE_SWASH_H3),

    // @Param: GYR_GAIN
    // @DisplayName: External Gyro Gain
    // @Description: PWM in microseconds sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GYR_GAIN", 6, AP_MotorsHeli_Single, _ext_gyro_gain_std, AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN),

    // @Param: PHANG
    // @DisplayName: Swashplate Phase Angle Compensation
    // @Description: Only for H3 swashplate.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -30 30
    // @Units: deg
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG", 7, AP_MotorsHeli_Single, _phase_angle, 0),

    // @Param: COLYAW
    // @DisplayName: Collective-Yaw Mixing
    // @Description: Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    // @Range: -10 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("COLYAW", 8,  AP_MotorsHeli_Single, _collective_yaw_effect, 0),

    // @Param: FLYBAR_MODE
    // @DisplayName: Flybar Mode Selector
    // @Description: Flybar present or not.  Affects attitude controller used during ACRO flight mode
    // @Values: 0:NoFlybar,1:Flybar
    // @User: Standard
    AP_GROUPINFO("FLYBAR_MODE", 9, AP_MotorsHeli_Single, _flybar_mode, AP_MOTORS_HELI_NOFLYBAR),

    // @Param: TAIL_SPEED
    // @DisplayName: Direct Drive VarPitch Tail ESC speed
    // @Description: Direct Drive VarPitch Tail ESC speed in PWM microseconds.  Only used when TailType is DirectDrive VarPitch
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TAIL_SPEED", 10, AP_MotorsHeli_Single, _direct_drive_tailspeed, AP_MOTORS_HELI_SINGLE_DDVP_SPEED_DEFAULT),

    // @Param: GYR_GAIN_ACRO
    // @DisplayName: External Gyro Gain for ACRO
    // @Description: PWM in microseconds sent to external gyro on ch7 when tail type is Servo w/ ExtGyro. A value of zero means to use H_GYR_GAIN
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GYR_GAIN_ACRO", 11, AP_MotorsHeli_Single,  _ext_gyro_gain_acro, 0),

    // Indices 16-18 were used by RSC_PWM_MIN, RSC_PWM_MAX and RSC_PWM_REV and should not be used

    // @Param: COL_CTRL_DIR
    // @DisplayName: Collective Control Direction
    // @Description: Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("COL_CTRL_DIR", 19, AP_MotorsHeli_Single, _collective_direction, AP_MOTORS_HELI_SINGLE_COLLECTIVE_DIRECTION_NORMAL),

    // parameters up to and including 29 are reserved for tradheli

    AP_GROUPEND
};

#define YAW_SERVO_MAX_ANGLE 4500

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Single::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = 
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4;
    rc_set_freq(mask, _speed_hz);
}

// init_outputs - initialise Servo/PWM ranges and endpoints
bool AP_MotorsHeli_Single::init_outputs()
{
    if (!_flags.initialised_ok) {
        // map primary swash servos
        for (uint8_t i=0; i<AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS; i++) {
            add_motor_num(CH_1+i);
        }
        // yaw servo
        add_motor_num(CH_4);

        // initialize main rotor servo
        _main_rotor.init_servo();

        if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
            _tail_rotor.init_servo();
        } else if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
            // external gyro output
            add_motor_num(AP_MOTORS_HELI_SINGLE_EXTGYRO);
        }
    }

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {            
        // External Gyro uses PWM output thus servo endpoints are forced
        SRV_Channels::set_output_min_max(SRV_Channels::get_motor_function(AP_MOTORS_HELI_SINGLE_EXTGYRO), 1000, 2000);
    }

    // reset swash servo range and endpoints
    for (uint8_t i=0; i<AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS; i++) {
        reset_swash_servo(SRV_Channels::get_motor_function(i));
    }

    // yaw servo is an angle from -4500 to 4500
    SRV_Channels::set_angle(SRV_Channel::k_motor4, YAW_SERVO_MAX_ANGLE);

    _flags.initialised_ok = true;

    return true;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Single::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // swash servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // swash servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // swash servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // external gyro & tail servo
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
                if (_acro_tail && _ext_gyro_gain_acro > 0) {
                    rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, _ext_gyro_gain_acro);
                } else {
                    rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, _ext_gyro_gain_std);
                }
            }
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // main rotor
            rc_write(AP_MOTORS_HELI_SINGLE_RSC, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

// set_desired_rotor_speed
void AP_MotorsHeli_Single::set_desired_rotor_speed(float desired_speed)
{
    _main_rotor.set_desired_speed(desired_speed);

    // always send desired speed to tail rotor control, will do nothing if not DDVP not enabled
    _tail_rotor.set_desired_speed(_direct_drive_tailspeed*0.001f);
}

// calculate_scalars - recalculates various scalers used.
void AP_MotorsHeli_Single::calculate_armed_scalars()
{
    float thrcrv[5];
    for (uint8_t i = 0; i < 5; i++) {
        thrcrv[i]=_rsc_thrcrv[i]*0.001f;
    }
    _main_rotor.set_ramp_time(_rsc_ramp_time);
    _main_rotor.set_runup_time(_rsc_runup_time);
    _main_rotor.set_critical_speed(_rsc_critical*0.001f);
    _main_rotor.set_idle_output(_rsc_idle_output*0.001f);
    _main_rotor.set_throttle_curve(thrcrv, (uint16_t)_rsc_slewrate.get());
}


// calculate_scalars - recalculates various scalers used.
void AP_MotorsHeli_Single::calculate_scalars()
{
    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min = AP_MOTORS_HELI_COLLECTIVE_MIN;
        _collective_max = AP_MOTORS_HELI_COLLECTIVE_MAX;
    }
    _collective_mid = constrain_int16(_collective_mid, _collective_min, _collective_max);

    // calculate collective mid point as a number from 0 to 1
    _collective_mid_pct = ((float)(_collective_mid-_collective_min))/((float)(_collective_max-_collective_min));

    // calculate factors based on swash type and servo position
    calculate_roll_pitch_collective_factors();

    // send setpoints to main rotor controller and trigger recalculation of scalars
    _main_rotor.set_control_mode(static_cast<RotorControlMode>(_rsc_mode.get()));
    calculate_armed_scalars();

    // send setpoints to DDVP rotor controller and trigger recalculation of scalars
    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        _tail_rotor.set_control_mode(ROTOR_CONTROL_MODE_SPEED_SETPOINT);
        _tail_rotor.set_ramp_time(_rsc_ramp_time);
        _tail_rotor.set_runup_time(_rsc_runup_time);
        _tail_rotor.set_critical_speed(_rsc_critical*0.001f);
        _tail_rotor.set_idle_output(_rsc_idle_output*0.001f);
    } else {
        _tail_rotor.set_control_mode(ROTOR_CONTROL_MODE_DISABLED);
        _tail_rotor.set_ramp_time(0);
        _tail_rotor.set_runup_time(0);
        _tail_rotor.set_critical_speed(0);
        _tail_rotor.set_idle_output(0);
    }
}

// CCPM Mixers - calculate mixing scale factors by swashplate type
void AP_MotorsHeli_Single::calculate_roll_pitch_collective_factors()
{
    if (_swash_type == AP_MOTORS_HELI_SINGLE_SWASH_H3) {                  //Three-Servo adjustable CCPM mixer factors
        // aileron factors
        _rollFactor[CH_1] = cosf(radians(_servo1_pos + 90 - _phase_angle));
        _rollFactor[CH_2] = cosf(radians(_servo2_pos + 90 - _phase_angle));
        _rollFactor[CH_3] = cosf(radians(_servo3_pos + 90 - _phase_angle));

        // elevator factors
        _pitchFactor[CH_1] = cosf(radians(_servo1_pos - _phase_angle));
        _pitchFactor[CH_2] = cosf(radians(_servo2_pos - _phase_angle));
        _pitchFactor[CH_3] = cosf(radians(_servo3_pos - _phase_angle));

        // collective factors
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;
    } else if (_swash_type == AP_MOTORS_HELI_SINGLE_SWASH_H3_140) {       //Three-Servo H3-140 CCPM mixer factors
        // aileron factors
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = -1;
        _rollFactor[CH_3] = 0;

        // elevator factors
        _pitchFactor[CH_1] = 1;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = -1;

        // collective factors
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;
    } else {                                                              //H1 straight outputs, no mixing
        // aileron factors
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = 0;
        _rollFactor[CH_3] = 0;

        // elevator factors
        _pitchFactor[CH_1] = 0;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = 0;

        // collective factors
        _collectiveFactor[CH_1] = 0;
        _collectiveFactor[CH_2] = 0;
        _collectiveFactor[CH_3] = 1;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeli_Single::get_motor_mask()
{
    // heli uses channels 1,2,3,4 and 8
    // setup fast channels
    uint32_t mask = 1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << AP_MOTORS_HELI_SINGLE_RSC;

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
        mask |= 1U << AP_MOTORS_HELI_SINGLE_EXTGYRO;
    }

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        mask |= 1U << AP_MOTORS_HELI_SINGLE_TAILRSC;
    }

    return rc_map_mask(mask);
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Single::update_motor_control(RotorControlState state)
{
    // Send state update to motors
    _tail_rotor.output(state);
    _main_rotor.output(state);

    if (state == ROTOR_CONTROL_STOP){
        // set engine run enable aux output to not run position to kill engine when disarmed
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    } else {
        // else if armed, set engine run enable output to run position
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    }

    // Check if both rotors are run-up, tail rotor controller always returns true if not enabled
    _heliflags.rotor_runup_complete = ( _main_rotor.is_runup_complete() && _tail_rotor.is_runup_complete() );
}

//
// move_actuators - moves swash plate and tail rotor
//                 - expected ranges:
//                       roll : -1 ~ +1
//                       pitch: -1 ~ +1
//                       collective: 0 ~ 1
//                       yaw:   -1 ~ +1
//
void AP_MotorsHeli_Single::move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)
{
    float yaw_offset = 0.0f;

    // initialize limits flag
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    if (_heliflags.inverted_flight) {
        coll_in = 1 - coll_in;
    }
 
    // rescale roll_out and pitch_out into the min and max ranges to provide linear motion
    // across the input range instead of stopping when the input hits the constrain value
    // these calculations are based on an assumption of the user specified cyclic_max
    // coming into this equation at 4500 or less
    float total_out = norm(pitch_out, roll_out);

    if (total_out > (_cyclic_max/4500.0f)) {
        float ratio = (float)(_cyclic_max/4500.0f) / total_out;
        roll_out *= ratio;
        pitch_out *= ratio;
        limit.roll_pitch = true;
    }

    // constrain collective input
    float collective_out = coll_in;
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
        collective_out = (_land_collective_min*0.001f);
        limit.throttle_lower = true;
    }

    // if servo output not in manual mode, process pre-compensation factors
    if (_servo_mode == SERVO_CONTROL_MODE_AUTOMATED) {
        // rudder feed forward based on collective
        // the feed-forward is not required when the motor is stopped or at idle, and thus not creating torque
        // also not required if we are using external gyro
        if ((_main_rotor.get_control_output() > _main_rotor.get_idle_output()) && _tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
            // sanity check collective_yaw_effect
            _collective_yaw_effect = constrain_float(_collective_yaw_effect, -AP_MOTORS_HELI_SINGLE_COLYAW_RANGE, AP_MOTORS_HELI_SINGLE_COLYAW_RANGE);
            // the 4.5 scaling factor is to bring the values in line with previous releases
            yaw_offset = _collective_yaw_effect * fabsf(collective_out - _collective_mid_pct) / 4.5f;
        }
    } else {
        yaw_offset = 0.0f;
    }

    // feed power estimate into main rotor controller
    // ToDo: include tail rotor power?
    // ToDo: add main rotor cyclic power?
    _main_rotor.set_collective(fabsf(collective_out));

    // scale collective pitch for swashplate servos
    float collective_scalar = ((float)(_collective_max-_collective_min))*0.001f;
    float collective_out_scaled = collective_out * collective_scalar + (_collective_min - 1000)*0.001f;

    // Collective control direction. Swash moves up for negative collective pitch, down for positive collective pitch
    if (_collective_direction == AP_MOTORS_HELI_SINGLE_COLLECTIVE_DIRECTION_REVERSED){
        collective_out_scaled = 1 - collective_out_scaled;
    }
    _servo1_out = ((_rollFactor[CH_1] * roll_out) + (_pitchFactor[CH_1] * pitch_out))*0.45f + _collectiveFactor[CH_1] * collective_out_scaled;
    _servo2_out = ((_rollFactor[CH_2] * roll_out) + (_pitchFactor[CH_2] * pitch_out))*0.45f + _collectiveFactor[CH_2] * collective_out_scaled;
    if (_swash_type == AP_MOTORS_HELI_SINGLE_SWASH_H1) {
        _servo1_out += 0.5f;
        _servo2_out += 0.5f;
    }
    _servo3_out = ((_rollFactor[CH_3] * roll_out) + (_pitchFactor[CH_3] * pitch_out))*0.45f + _collectiveFactor[CH_3] * collective_out_scaled;

    // rescale from -1..1, so we can use the pwm calc that includes trim
    _servo1_out = 2*_servo1_out - 1;
    _servo2_out = 2*_servo2_out - 1;
    _servo3_out = 2*_servo3_out - 1;

    // update the yaw rate using the tail rotor/servo
    move_yaw(yaw_out + yaw_offset);
}

// move_yaw
void AP_MotorsHeli_Single::move_yaw(float yaw_out)
{
    // sanity check yaw_out
    if (yaw_out < -1.0f) {
        yaw_out = -1.0f;
        limit.yaw = true;
    }
    if (yaw_out > 1.0f) {
        yaw_out = 1.0f;
        limit.yaw = true;
    }

    _servo4_out = yaw_out;
}

void AP_MotorsHeli_Single::output_to_motors()
{
    if (!_flags.initialised_ok) {
        return;
    }

    // actually move the servos.  PWM is sent based on nominal 1500 center.  servo output shifts center based on trim value.
    rc_write_swash(AP_MOTORS_MOT_1, _servo1_out);
    rc_write_swash(AP_MOTORS_MOT_2, _servo2_out);
    rc_write_swash(AP_MOTORS_MOT_3, _servo3_out);
    if (_tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH){
        rc_write_angle(AP_MOTORS_MOT_4, _servo4_out * YAW_SERVO_MAX_ANGLE);
    }
    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
        // output gain to exernal gyro
        if (_acro_tail && _ext_gyro_gain_acro > 0) {
            rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, 1000 + _ext_gyro_gain_acro);
        } else {
            rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, 1000 + _ext_gyro_gain_std);
        }
    }

    switch (_spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            update_motor_control(ROTOR_CONTROL_STOP);
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH){
                rc_write_angle(AP_MOTORS_MOT_4, -YAW_SERVO_MAX_ANGLE);
            }
            break;
        case GROUND_IDLE:
            // sends idle output to motors when armed. rotor could be static or turning (autorotation)
            update_motor_control(ROTOR_CONTROL_IDLE);
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH){
                rc_write_angle(AP_MOTORS_MOT_4, -YAW_SERVO_MAX_ANGLE);
            }
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
            // set motor output based on thrust requests
            update_motor_control(ROTOR_CONTROL_ACTIVE);
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH){
                // constrain output so that motor never fully stops
                 _servo4_out = constrain_float(_servo4_out, -0.9f, 1.0f);
                // output yaw servo to tail rsc
                rc_write_angle(AP_MOTORS_MOT_4, _servo4_out * YAW_SERVO_MAX_ANGLE);
            }
            break;
        case SPOOL_DOWN:
            // sends idle output to motors and wait for rotor to stop
            update_motor_control(ROTOR_CONTROL_IDLE);
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH){
                rc_write_angle(AP_MOTORS_MOT_4, -YAW_SERVO_MAX_ANGLE);
            }
            break;

    }
}

// servo_test - move servos through full range of movement
void AP_MotorsHeli_Single::servo_test()
{
    _servo_test_cycle_time += 1.0f / _loop_rate;

    if ((_servo_test_cycle_time >= 0.0f && _servo_test_cycle_time < 0.5f)||                                   // Tilt swash back
        (_servo_test_cycle_time >= 6.0f && _servo_test_cycle_time < 6.5f)){
        _pitch_test += (1.0f / (_loop_rate / 2.0f));
        _oscillate_angle += 8 * M_PI / _loop_rate;
        _yaw_test = 0.5f * sinf(_oscillate_angle);
    } else if ((_servo_test_cycle_time >= 0.5f && _servo_test_cycle_time < 4.5f)||                            // Roll swash around
               (_servo_test_cycle_time >= 6.5f && _servo_test_cycle_time < 10.5f)){
        _oscillate_angle += M_PI / (2 * _loop_rate);
        _roll_test = sinf(_oscillate_angle);
        _pitch_test = cosf(_oscillate_angle);
        _yaw_test = sinf(_oscillate_angle);
    } else if ((_servo_test_cycle_time >= 4.5f && _servo_test_cycle_time < 5.0f)||                            // Return swash to level
               (_servo_test_cycle_time >= 10.5f && _servo_test_cycle_time < 11.0f)){
        _pitch_test -= (1.0f / (_loop_rate / 2.0f));
        _oscillate_angle += 8 * M_PI / _loop_rate;
        _yaw_test = 0.5f * sinf(_oscillate_angle);
    } else if (_servo_test_cycle_time >= 5.0f && _servo_test_cycle_time < 6.0f){                              // Raise swash to top
        _collective_test += (1.0f / _loop_rate);
        _oscillate_angle += 2 * M_PI / _loop_rate;
        _yaw_test = sinf(_oscillate_angle);
    } else if (_servo_test_cycle_time >= 11.0f && _servo_test_cycle_time < 12.0f){                            // Lower swash to bottom
        _collective_test -= (1.0f / _loop_rate);
        _oscillate_angle += 2 * M_PI / _loop_rate;
        _yaw_test = sinf(_oscillate_angle);
    } else {                                                                                                  // reset cycle
        _servo_test_cycle_time = 0.0f;
        _oscillate_angle = 0.0f;
        _collective_test = 0.0f;
        _roll_test = 0.0f;
        _pitch_test = 0.0f;
        _yaw_test = 0.0f;
        // decrement servo test cycle counter at the end of the cycle
        if (_servo_test_cycle_counter > 0){
            _servo_test_cycle_counter--;
        }
    }

    // over-ride servo commands to move servos through defined ranges
    _throttle_filter.reset(constrain_float(_collective_test, 0.0f, 1.0f));
    _roll_in = constrain_float(_roll_test, -1.0f, 1.0f);
    _pitch_in = constrain_float(_pitch_test, -1.0f, 1.0f);
    _yaw_in = constrain_float(_yaw_test, -1.0f, 1.0f);
}

// parameter_check - check if helicopter specific parameters are sensible
bool AP_MotorsHeli_Single::parameter_check(bool display_msg) const
{
    // returns false if Phase Angle is outside of range
    if ((_phase_angle > 30) || (_phase_angle < -30)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_PHANG out of range");
        }
        return false;
    }

    // returns false if Acro External Gyro Gain is outside of range
    if ((_ext_gyro_gain_acro < 0) || (_ext_gyro_gain_acro > 1000)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_GYR_GAIN_ACRO out of range");
        }
        return false;
    }

    // returns false if Standard External Gyro Gain is outside of range
    if ((_ext_gyro_gain_std < 0) || (_ext_gyro_gain_std > 1000)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_GYR_GAIN out of range");
        }
        return false;
    }

    // check parent class parameters
    return AP_MotorsHeli::parameter_check(display_msg);
}
