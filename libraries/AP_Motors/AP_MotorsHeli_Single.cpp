// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#include <AP_HAL.h>
#include "AP_MotorsHeli_Single.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Single::var_info[] PROGMEM = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // @Param: SV1_POS
    // @DisplayName: Servo 1 Position
    // @Description: Angular location of swash servo #1
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV1_POS", 1, AP_MotorsHeli_Single, _servo1_pos, AP_MOTORS_HELI_SINGLE_SERVO1_POS),

    // @Param: SV2_POS
    // @DisplayName: Servo 2 Position
    // @Description: Angular location of swash servo #2
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV2_POS", 2, AP_MotorsHeli_Single, _servo2_pos, AP_MOTORS_HELI_SINGLE_SERVO2_POS),

    // @Param: SV3_POS
    // @DisplayName: Servo 3 Position
    // @Description: Angular location of swash servo #3
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV3_POS", 3, AP_MotorsHeli_Single, _servo3_pos, AP_MOTORS_HELI_SINGLE_SERVO3_POS),

    // @Param: TAIL_TYPE
    // @DisplayName: Tail Type
    // @Description: Tail type selection.  Simpler yaw controller used if external gyro is selected
    // @Values: 0:Servo only,1:Servo with ExtGyro,2:DirectDrive VarPitch,3:DirectDrive FixedPitch
    // @User: Standard
    AP_GROUPINFO("TAIL_TYPE", 9, AP_MotorsHeli_Single, _tail_type, AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO),

    // @Param: SWASH_TYPE
    // @DisplayName: Swash Type
    // @Description: Swash Type Setting - either 3-servo CCPM or H1 Mechanical Mixing
    // @Values: 0:3-Servo CCPM, 1:H1 Mechanical Mixing
    // @User: Standard
    AP_GROUPINFO("SWASH_TYPE", 10, AP_MotorsHeli_Single, _swash_type, AP_MOTORS_HELI_SINGLE_SWASH_CCPM),

    // @Param: GYR_GAIN
    // @DisplayName: External Gyro Gain
    // @Description: PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GYR_GAIN", 11, AP_MotorsHeli_Single, _ext_gyro_gain, AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN),

    // @Param: PHANG
    // @DisplayName: Swashplate Phase Angle Compensation
    // @Description: Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -90 90
    // @Units: Degrees
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG", 13, AP_MotorsHeli_Single, _phase_angle, 0),

    // @Param: COLYAW
    // @DisplayName: Collective-Yaw Mixing
    // @Description: Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    // @Range: -10 10
    AP_GROUPINFO("COLYAW", 14, AP_MotorsHeli_Single, _collective_yaw_effect, 0),

    // 17 was RSC_RAMP_RATE which has been replaced by RSC_RAMP_TIME

    // @Param: FLYBAR_MODE
    // @DisplayName: Flybar Mode Selector
    // @Description: Flybar present or not.  Affects attitude controller used during ACRO flight mode
    // @Range: 0:NoFlybar 1:Flybar
    // @User: Standard
    AP_GROUPINFO("FLYBAR_MODE", 18, AP_MotorsHeli_Single, _flybar_mode, AP_MOTORS_HELI_NOFLYBAR),

    // 19,20 - was STAB_COL_MIN, STAB_COL_MAX now moved to main code's parameter list

    // @Param: TAIL_SPEED
    // @DisplayName: Direct Drive VarPitch Tail ESC speed
    // @Description: Direct Drive VarPitch Tail ESC speed.  Only used when TailType is DirectDrive VarPitch
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TAIL_SPEED", 24, AP_MotorsHeli_Single, _tail_direct_drive_speed, AP_MOTORS_HELI_SINGLE_TAIL_DIRECTDRIVE_SPEED_DEFAULT),

    AP_GROUPEND
};

//
// public methods
//

// init
void AP_MotorsHeli_Single::Init()
{
    AP_MotorsHeli::Init();

    // disable main and tail RSC channels from being used by RC_Channel_aux
    RC_Channel_aux::disable_aux_channel(_motor_to_channel_map[AP_MOTORS_HELI_SINGLE_RSC]);
    RC_Channel_aux::disable_aux_channel(_motor_to_channel_map[AP_MOTORS_HELI_SINGLE_AUX]);
}

// set update rate to servos - a value in hertz
void AP_MotorsHeli_Single::set_update_rate( uint16_t speed_hz )
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

bool AP_MotorsHeli_Single::allow_arming()
{
    return _main_rotor.get_speed_target() == 0;
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsHeli_Single::enable()
{
    // enable output channels
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]));    // swash servo 1
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]));    // swash servo 2
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]));    // swash servo 3
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]));    // yaw servo

    hal.rcout->enable_ch(AP_MOTORS_HELI_SINGLE_RSC);
    hal.rcout->enable_ch(AP_MOTORS_HELI_SINGLE_AUX);
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Single::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
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
        hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_HELI_SINGLE_RSC]), pwm);
        break;
    default:
        // do nothing
        break;
    }
}


// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeli_Single::get_motor_mask()
{
    // heli uses channels 1,2,3,4,7 and 8
    return (1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << AP_MOTORS_HELI_SINGLE_AUX | 1U << AP_MOTORS_HELI_SINGLE_RSC);
}

// recalc_scalers
void AP_MotorsHeli_Single::recalc_scalers()
{
    AP_MotorsHeli::recalc_scalers();

    _main_rotor.set_ramp_time(_rsc_ramp_time);
    _main_rotor.set_runup_time(_rsc_runup_time);
    _main_rotor.recalc_scalers();

    if (_rsc_mode == AP_MOTORS_HELI_RSC_MODE_NONE) {
        // no ramping required
        _tail_rotor.set_ramp_time(0);
        _tail_rotor.set_runup_time(0);
    } else {
        _tail_rotor.set_ramp_time(_rsc_ramp_time);
        _tail_rotor.set_runup_time(_rsc_runup_time);
    }

    _tail_rotor.recalc_scalers();
}

//
// protected methods
//

// init_servos
void AP_MotorsHeli_Single::init_servos ()
{
    init_swash_servo (_servo_1);
    init_swash_servo (_servo_2);
    init_swash_servo (_servo_3);

    _servo_yaw.set_angle(4500);
}

// reset_servos
void AP_MotorsHeli_Single::reset_servos ()
{
    reset_swash_servo (_servo_1);
    reset_swash_servo (_servo_2);
    reset_swash_servo (_servo_3);
}

// calculate_swash_factors - calculate factors based on swash type and servo position
void AP_MotorsHeli_Single::calculate_swash_factors()
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

// set_dt
void AP_MotorsHeli_Single::set_dt(float dt)
{
    _main_rotor.set_dt(dt);
    _tail_rotor.set_dt(dt);
}

void AP_MotorsHeli_Single::set_speed_target(int16_t speed_target)
{
    _main_rotor.set_speed_target(speed_target);

    if (speed_target > 0 && _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        _tail_rotor.set_speed_target(_tail_direct_drive_speed);
    } else {
        _tail_rotor.set_speed_target(0);
    }
}

// output_armed
void AP_MotorsHeli_Single::output_armed_stabilizing()
{
    AP_MotorsHeli::output_armed();

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        _tail_rotor.output_armed();

        if (!_tail_rotor.is_runup_complete())
        {
            _heliflags.motor_runup_complete = false;
            return;
        }
    }

    _main_rotor.output_armed();

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        _heliflags.motor_runup_complete = _main_rotor.is_runup_complete() && _tail_rotor.is_runup_complete();
    } else {
        _heliflags.motor_runup_complete = _main_rotor.is_runup_complete();
    }
}

// output_disarmed
void AP_MotorsHeli_Single::output_disarmed()
{
    AP_MotorsHeli::output_disarmed();

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        _tail_rotor.output_disarmed();
    }

    _main_rotor.output_disarmed();

    _heliflags.motor_runup_complete = false;
}

// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 0 ~ 1000
//                       yaw:   -4500 ~ 4500
//
void AP_MotorsHeli_Single::move_swash(int16_t roll_out, int16_t pitch_out, int16_t collective_in, int16_t yaw_out)
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
        if ((_main_rotor.get_speed_target() > 0) && _tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
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

    // use servo_out to calculate pwm_out and radio_out
    _servo_1.calc_pwm();
    _servo_2.calc_pwm();
    _servo_3.calc_pwm();

    // actually move the servos
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), _servo_1.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), _servo_2.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), _servo_3.radio_out);

    // update the yaw rate using the tail rotor/servo
    output_yaw(yaw_out + yaw_offset);
}

// output_yaw - update the yaw rate
void AP_MotorsHeli_Single::output_yaw(int16_t yaw_out)
{
    _servo_yaw.servo_out = constrain_int16(yaw_out, -4500, 4500);

    if (_servo_yaw.servo_out != yaw_out) {
        limit.yaw = true;
    }

    _servo_yaw.calc_pwm();

    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), _servo_yaw.radio_out);

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
        // output gain to exernal gyro
        write_aux(_ext_gyro_gain);
    } else if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH && _main_rotor.get_speed_target() > 0) {
        // output yaw servo to tail rsc
        write_aux(_servo_yaw.servo_out);
    }
}

// write_aux - outputs pwm onto output aux channel (ch7)
// servo_out parameter is of the range 0 ~ 1000
void AP_MotorsHeli_Single::write_aux(int16_t servo_out)
{
    _servo_aux.servo_out = servo_out;
    _servo_aux.calc_pwm();

    hal.rcout->write(AP_MOTORS_HELI_SINGLE_AUX, _servo_aux.radio_out);
}