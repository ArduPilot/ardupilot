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
 *       AP_MotorsHeliSingle.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <stdlib.h>
#include <AP_HAL.h>
#include "AP_MotorsHeliSingle.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeliSingle::var_info[] PROGMEM = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // @Param: SV1_POS
    // @DisplayName: Servo 1 Position
    // @Description: Angular location of swash servo #1
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV1_POS", 1, AP_MotorsHeliSingle, _servo1_pos, AP_MOTORS_HELI_SINGLE_SERVO1_POS),

    // @Param: SV2_POS
    // @DisplayName: Servo 2 Position
    // @Description: Angular location of swash servo #2
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV2_POS", 2, AP_MotorsHeliSingle, _servo2_pos, AP_MOTORS_HELI_SINGLE_SERVO2_POS),

    // @Param: SV3_POS
    // @DisplayName: Servo 3 Position
    // @Description: Angular location of swash servo #3
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV3_POS", 3, AP_MotorsHeliSingle, _servo3_pos, AP_MOTORS_HELI_SINGLE_SERVO3_POS),

    // @Param: TAIL_TYPE
    // @DisplayName: Tail Type
    // @Description: Tail type selection.  Simpler yaw controller used if external gyro is selected
    // @Values: 0:Servo only,1:Servo with ExtGyro,2:DirectDrive VarPitch,3:DirectDrive FixedPitch
    // @User: Standard
    AP_GROUPINFO("TAIL_TYPE", 9, AP_MotorsHeliSingle, _tail_type, AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO),

    // @Param: SWASH_TYPE
    // @DisplayName: Swash Type
    // @Description: Swash Type Setting - either 3-servo CCPM or H1 Mechanical Mixing
    // @Values: 0:3-Servo CCPM, 1:H1 Mechanical Mixing
    // @User: Standard
    AP_GROUPINFO("SWASH_TYPE", 10, AP_MotorsHeliSingle, _swash_type, AP_MOTORS_HELI_SINGLE_SWASH_CCPM),

    // @Param: GYR_GAIN
    // @DisplayName: External Gyro Gain
    // @Description: PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GYR_GAIN", 11, AP_MotorsHeliSingle, _ext_gyro_gain, AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN),

    // @Param: PHANG
    // @DisplayName: Swashplate Phase Angle Compensation
    // @Description: Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -90 90
    // @Units: Degrees
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG", 13, AP_MotorsHeliSingle, _phase_angle, 0),

    // @Param: COLYAW
    // @DisplayName: Collective-Yaw Mixing
    // @Description: Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    // @Range: -10 10
    AP_GROUPINFO("COLYAW", 14, AP_MotorsHeliSingle, _collective_yaw_effect, 0),

    // 17 was RSC_RAMP_RATE which has been replaced by RSC_RAMP_TIME

    // @Param: FLYBAR_MODE
    // @DisplayName: Flybar Mode Selector
    // @Description: Flybar present or not.  Affects attitude controller used during ACRO flight mode
    // @Range: 0:NoFlybar 1:Flybar
    // @User: Standard
    AP_GROUPINFO("FLYBAR_MODE", 18, AP_MotorsHeliSingle, _flybar_mode, AP_MOTORS_HELI_NOFLYBAR),

    // 19,20 - was STAB_COL_MIN, STAB_COL_MAX now moved to main code's parameter list  

    // @Param: TAIL_SPEED
    // @DisplayName: Direct Drive VarPitch Tail ESC speed
    // @Description: Direct Drive VarPitch Tail ESC speed.  Only used when TailType is DirectDrive VarPitch
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TAIL_SPEED", 24, AP_MotorsHeliSingle, _direct_drive_tailspeed, AP_MOTORS_HELI_SINGLE_DDTAIL_DEFAULT),

    AP_GROUPEND
};

//
// public methods
//

// init
void AP_MotorsHeliSingle::Init()
{
    AP_MotorsHeli::Init();

    // disable channel 8 from being used by RC_Channel_aux
    RC_Channel_aux::disable_aux_channel(_motor_to_channel_map[AP_MOTORS_HELI_SINGLE_AUX]);
}

// set update rate to motors - a value in hertz
void AP_MotorsHeliSingle::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = 
	    1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]) |
	    1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]) |
	    1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]) |
	    1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]);

    hal.rcout->set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsHeliSingle::enable()
{
    AP_MotorsHeli::enable();

    // enable output channels
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]));    // swash servo 1
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]));    // swash servo 2
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]));    // swash servo 3
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]));    // yaw
    hal.rcout->enable_ch(AP_MOTORS_HELI_SINGLE_AUX);                                 // output for gyro gain or direct drive variable pitch tail motor
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeliSingle::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!_flags.armed) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // swash servo 1
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), pwm);
            break;
        case 2:
            // swash servo 2
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), pwm);
            break;
        case 3:
            // swash servo 3
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), pwm);
            break;
        case 4:
            // external gyro & tail servo
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
                write_aux(_ext_gyro_gain);
            }
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), pwm);
            break;
        case 5:
            // main rotor
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_HELI_RSC]), pwm);
            break;
        default:
            // do nothing
            break;
    }
}


// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeliSingle::get_motor_mask()
{
    // heli uses channels 1,2,3,4,7 and 8
    return (1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << AP_MOTORS_HELI_SINGLE_AUX | 1U << AP_MOTORS_HELI_RSC);
}

//
// protected methods
//

// init_servos
void AP_MotorsHeliSingle::init_servos ()
{
    init_swash_servo (_servo_1);
    init_swash_servo (_servo_2);
    init_swash_servo (_servo_3);

    _servo_4.set_angle(4500);
}

// reset_servos
void AP_MotorsHeliSingle::reset_servos ()
{
    reset_swash_servo (_servo_1);
    reset_swash_servo (_servo_2);
    reset_swash_servo (_servo_3);
}

// calculate_swash_factors - calculate factors based on swash type and servo position
void AP_MotorsHeliSingle::calculate_swash_factors()
{
    if (_swash_type == AP_MOTORS_HELI_SINGLE_SWASH_CCPM) { // perform control mixing

        // roll factors
        _roll_factors[CH_1] = cosf(radians(_servo1_pos + 90 - (_phase_angle + _delta_phase_angle)));
        _roll_factors[CH_2] = cosf(radians(_servo2_pos + 90 - (_phase_angle + _delta_phase_angle)));
        _roll_factors[CH_3] = cosf(radians(_servo3_pos + 90 - (_phase_angle + _delta_phase_angle)));

        // pitch factors
        _pitch_factors[CH_1] = cosf(radians(_servo1_pos - (_phase_angle + _delta_phase_angle)));
        _pitch_factors[CH_2] = cosf(radians(_servo2_pos - (_phase_angle + _delta_phase_angle)));
        _pitch_factors[CH_3] = cosf(radians(_servo3_pos - (_phase_angle + _delta_phase_angle)));

        // collective factors
        _collective_factors[CH_1] = 1;
        _collective_factors[CH_2] = 1;
        _collective_factors[CH_3] = 1;

    }else{ // AP_MOTORS_HELI_SINGLE_SWASH_H1, keep servo outputs seperated

        // roll factors
        _roll_factors[CH_1] = 1;
        _roll_factors[CH_2] = 0;
        _roll_factors[CH_3] = 0;

        // pitch factors
        _pitch_factors[CH_1] = 0;
        _pitch_factors[CH_2] = 1;
        _pitch_factors[CH_3] = 0;

        // collective factors
        _collective_factors[CH_1] = 0;
        _collective_factors[CH_2] = 0;
        _collective_factors[CH_3] = 1;
    }
}

//
// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 0 ~ 1000
//                       yaw:   -4500 ~ 4500
//
void AP_MotorsHeliSingle::move_swash(int16_t roll_out, int16_t pitch_out, int16_t collective_in, int16_t yaw_out)
{
    int16_t yaw_offset = 0;
    int16_t collective_out_scaled;

    // initialize limits flag
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    if (_servo_manual == 1) {      // are we in manual servo mode? (i.e. swash set-up mode)?
        // check if we need to free up the swash
        if (_heliflags.swash_initialised) {
            reset_swash();
        }
        // To-Do:  This equation seems to be wrong.  It probably restricts swash movement so that swash setup doesn't work right.
        // _collective_scaler should probably not be used or set to 1?
        collective_out_scaled = collective_in * _collective_scaler + _rc_throttle.radio_min - 1000;
    }else{      // regular flight mode

        // check if we need to reinitialise the swash
        if (!_heliflags.swash_initialised) {
            init_swash();
        }

        // rescale roll_out and pitch-out into the min and max ranges to provide linear motion
        // across the input range instead of stopping when the input hits the constrain value
        // these calculations are based on an assumption of the user specified roll_max and pitch_max
        // coming into this equation at 4500 or less, and based on the original assumption of the
        // total _servo_x.servo_out range being -4500 to 4500.
        roll_out = roll_out * _roll_scaler;
        if (roll_out < -_roll_max) {
            roll_out = -_roll_max;
            limit.roll_pitch = true;
        }
        if (roll_out > _roll_max) {
            roll_out = _roll_max;
            limit.roll_pitch = true;
        }

        // scale pitch and update limits
        pitch_out = pitch_out * _pitch_scaler;
        if (pitch_out < -_pitch_max) {
            pitch_out = -_pitch_max;
            limit.roll_pitch = true;
        }
        if (pitch_out > _pitch_max) {
            pitch_out = _pitch_max;
            limit.roll_pitch = true;
        }

        // constrain collective input
        _collective_out = collective_in;
        if (_collective_out <= 0) {
            _collective_out = 0;
            limit.throttle_lower = true;
        }
        if (_collective_out >= 1000) {
            _collective_out = 1000;
            limit.throttle_upper = true;
        }

        // ensure not below landed/landing collective
        if (_heliflags.landing_collective && _collective_out < _land_collective_min) {
            _collective_out = _land_collective_min;
            limit.throttle_lower = true;
        }

        // scale collective pitch
        collective_out_scaled = _collective_out * _collective_scaler + _collective_min - 1000;
    
        // rudder feed forward based on collective
        // the feed-forward is not required when the motor is shut down and not creating torque
        // also not required if we are using external gyro
        if ((_rotor_desired > 0) && _tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
            yaw_offset = _collective_yaw_effect * abs(_collective_out - _collective_mid_pwm);
        }
    }

    // swashplate servos
    _servo_1.servo_out = (_roll_factors[CH_1] * roll_out + _pitch_factors[CH_1] * pitch_out)/10 + _collective_factors[CH_1] * collective_out_scaled + (_servo_1.radio_trim-1500);
    _servo_2.servo_out = (_roll_factors[CH_2] * roll_out + _pitch_factors[CH_2] * pitch_out)/10 + _collective_factors[CH_2] * collective_out_scaled + (_servo_2.radio_trim-1500);
    if (_swash_type == AP_MOTORS_HELI_SINGLE_SWASH_H1) {
        _servo_1.servo_out += 500;
        _servo_2.servo_out += 500;
    }
    _servo_3.servo_out = (_roll_factors[CH_3] * roll_out + _pitch_factors[CH_3] * pitch_out)/10 + _collective_factors[CH_3] * collective_out_scaled + (_servo_3.radio_trim-1500);
    _servo_4.servo_out = yaw_out + yaw_offset;

    // constrain yaw and update limits
    if (_servo_4.servo_out < -4500) {
        _servo_4.servo_out = -4500;
        limit.yaw = true;
    }
    if (_servo_4.servo_out > 4500) {
        _servo_4.servo_out = 4500;
        limit.yaw = true;
    }

    // use servo_out to calculate pwm_out and radio_out
    _servo_1.calc_pwm();
    _servo_2.calc_pwm();
    _servo_3.calc_pwm();
    _servo_4.calc_pwm();

    // actually move the servos
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), _servo_1.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), _servo_2.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), _servo_3.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), _servo_4.radio_out);

    // output gain to exernal gyro
    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
        write_aux(_ext_gyro_gain);
    }
}

// rsc_control - update value to send to tail and main rotor's ESC
// desired_rotor_speed is a desired speed from 0 to 1000
void AP_MotorsHeliSingle::rsc_control()
{
    // if disarmed output minimums
    if (!armed()) {
        // shut down tail rotor
        if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH) {
            _tail_direct_drive_out = 0;
            write_aux(_tail_direct_drive_out);
        }
        
        // shut down main rotor
        if (_rsc_mode != AP_MOTORS_HELI_RSC_MODE_NONE) {
            _rotor_out = 0;
            _rotor_speed_estimate = 0;
            write_rsc(_rotor_out);
        }

        return;
    }

    // ramp up or down main rotor and tail
    if (_rotor_desired > 0) {
        // ramp up tail rotor (this does nothing if not using direct drive variable pitch tail)
         tail_ramp(_direct_drive_tailspeed);
        // note: this always returns true if not using direct drive variable pitch tail
        if (tail_rotor_runup_complete()) {
            rotor_ramp(_rotor_desired);
        }
    }else{
        // shutting down main rotor
        rotor_ramp(0);
        // shut-down tail rotor.  Note: this does nothing if not using direct drive vairable pitch tail        
        tail_ramp(0);
    }

    // direct drive fixed pitch tail servo gets copy of yaw servo out (ch4) while main rotor is running
    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH) {
        // output fixed-pitch speed control if Ch8 is high
        if (_rotor_desired > 0 || _rotor_speed_estimate > 0) {
            // copy yaw output to tail esc
            write_aux(_servo_4.servo_out);
        }else{
            write_aux(0);
        }
    }
}

//
// private methods
//

// tail_ramp - ramps tail motor towards target.  Only used for direct drive variable pitch tails
// results put into _tail_direct_drive_out and sent to ESC
void AP_MotorsHeliSingle::tail_ramp(int16_t tail_target)
{
    // return immediately if not ramping required
    if (_tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        _tail_direct_drive_out = tail_target;
        return;
    }

    // range check tail_target
    tail_target = constrain_int16(tail_target,0,1000);

    // ramp towards target
    if (_tail_direct_drive_out < tail_target) {
        _tail_direct_drive_out += AP_MOTORS_HELI_SINGLE_TAIL_RAMP_INCREMENT;
        if (_tail_direct_drive_out >= tail_target) {
            _tail_direct_drive_out = tail_target;
        }
    }else if(_tail_direct_drive_out > tail_target) {
        _tail_direct_drive_out -= AP_MOTORS_HELI_SINGLE_TAIL_RAMP_INCREMENT;
        if (_tail_direct_drive_out < tail_target) {
            _tail_direct_drive_out = tail_target;
        }
    }

    // output to tail servo
    write_aux(_tail_direct_drive_out);
}

// return true if the tail rotor is up to speed
bool AP_MotorsHeliSingle::tail_rotor_runup_complete()
{
    // always return true if not using direct drive variable pitch tails
    if (_tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        return true;
    }

    // check speed
    return (armed() && _tail_direct_drive_out >= _direct_drive_tailspeed);
}

// write_aux - outputs pwm onto output aux channel (ch7)
// servo_out parameter is of the range 0 ~ 1000
void AP_MotorsHeliSingle::write_aux(int16_t servo_out)
{
    _servo_aux.servo_out = servo_out;
    _servo_aux.calc_pwm();
    hal.rcout->write(AP_MOTORS_HELI_SINGLE_AUX, _servo_aux.radio_out);
}

// set_delta_phase_angle for setting variable phase angle compensation and force
// recalculation of collective factors
void AP_MotorsHeliSingle::set_delta_phase_angle(int16_t angle)
{
    angle = constrain_int16(angle, -90, 90);
    _delta_phase_angle = angle;
    calculate_swash_factors();
}
