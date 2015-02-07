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
 
#include "AP_MotorsHeli_Dual.h"

extern const AP_HAL::HAL& hal;

// these params include those from AP_MotorsHeli and the additional params are
// offset by 100 in order not to collide w. the ones defined in AP_MotorsHeli
// or in AP_MotorsHeliSingle
const AP_Param::GroupInfo AP_MotorsHeli_Dual::var_info[] PROGMEM = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // @Param: SV1_POS
    // @DisplayName: Servo 1 Position
    // @Description: Angular location of swash servo #1
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV1_POS", 1, AP_MotorsHeli_Dual, _servo1_pos, AP_MOTORS_HELI_DUAL_SERVO1_POS),

    // @Param: SV2_POS
    // @DisplayName: Servo 2 Position
    // @Description: Angular location of swash servo #2
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV2_POS", 2, AP_MotorsHeli_Dual, _servo2_pos,  AP_MOTORS_HELI_DUAL_SERVO2_POS),

    // @Param: SV3_POS
    // @DisplayName: Servo 3 Position
    // @Description: Angular location of swash servo #3
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV3_POS", 3, AP_MotorsHeli_Dual, _servo3_pos,  AP_MOTORS_HELI_DUAL_SERVO3_POS),

    // @Param: SV4_POS
    // @DisplayName: Servo 4 Position
    // @Description: Angular location of swash servo #4
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV4_POS", 4, AP_MotorsHeli_Dual,  _servo4_pos, AP_MOTORS_HELI_DUAL_SERVO4_POS),

    // @Param: SV5_POS
    // @DisplayName: Servo 5 Position
    // @Description: Angular location of swash servo #5
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV5_POS", 5, AP_MotorsHeli_Dual, _servo5_pos, AP_MOTORS_HELI_DUAL_SERVO5_POS),

    // @Param: SV6_POS
    // @DisplayName: Servo 6 Position
    // @Description: Angular location of swash servo #6
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV6_POS", 6, AP_MotorsHeli_Dual, _servo6_pos, AP_MOTORS_HELI_DUAL_SERVO6_POS),

    // @Param: PHANG1
    // @DisplayName: Swashplate 1 Phase Angle Compensation
    // @Description: Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -90 90
    // @Units: Degrees
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG1", 7, AP_MotorsHeli_Dual, _swash1_phase_angle,   0),

    // @Param: PHANG2
    // @DisplayName: Swashplate 2 Phase Angle Compensation
    // @Description: Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -90 90
    // @Units: Degrees
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG2", 8, AP_MotorsHeli_Dual, _swash2_phase_angle,   0),

    // @Param: DUAL_MODE
    // @DisplayName: Dual Mode
    // @Description: Sets the dual mode of the heli, either as tandem or as transverse.
    // @Values: 0:Longitudinal, 1:Transverse
    // @User: Standard
    AP_GROUPINFO("DUAL_MODE", 9, AP_MotorsHeli_Dual, _dual_mode, AP_MOTORS_HELI_DUAL_MODE_TANDEM),

    // @Param: DCP_SCALER
    // @DisplayName: Differential-Collective-Pitch Scaler
    // @Description: Scaling factor applied to the differential-collective-pitch
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("DCP_SCALER", 10, AP_MotorsHeli_Dual, _dcp_scaler, AP_MOTORS_HELI_DUAL_DCP_SCALER),

    // @Param: DCP_YAW
    // @DisplayName: Differential-Collective-Pitch Yaw Mixing
    // @Description: Feed-forward compensation to automatically add yaw input when differential collective pitch is applied.
    // @Range: -10 10
    AP_GROUPINFO("DCP_YAW", 11, AP_MotorsHeli_Dual, _dcp_yaw_effect, 0),

    // @Param: YAW_MAX
    // @DisplayName: Swash Yaw Angle Max
    // @Description: Maximum yaw angle of the swash plate
    // @Range: 0 18000
    // @Units: Centi-Degrees
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("YAW_MAX", 12, AP_MotorsHeli_Dual, _yaw_max, AP_MOTORS_HELI_DUAL_SWASH_YAW_MAX),

    AP_GROUPEND
};

// init
void AP_MotorsHeli_Dual::Init()
{
    AP_MotorsHeli::Init();

    // disable RSC channel from being used by RC_Channel_aux
    RC_Channel_aux::disable_aux_channel(_motor_to_channel_map[AP_MOTORS_HELI_DUAL_RSC]);
}

//
// public methods
//

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Dual::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask =
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_5]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_6]);

    hal.rcout->set_freq(mask, _speed_hz);
}

bool AP_MotorsHeli_Dual::allow_arming()
{
    return _rotor.is_runup_complete();
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsHeli_Dual::enable()
{
    // enable output channels
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]));    // swash servo 1
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]));    // swash servo 2
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]));    // swash servo 3
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]));    // swash servo 4
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_5]));    // swash servo 5
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_6]));    // swash servo 6

    hal.rcout->enable_ch(AP_MOTORS_HELI_DUAL_RSC);
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Dual::output_test(uint8_t motor_seq, int16_t pwm)
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
        // swash servo 4
        hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), pwm);
        break;
    case 5:
        // swash servo 5
        hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_5]), pwm);
        break;
    case 6:
        // swash servo 6
        hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_6]), pwm);
        break;
    case 7:
        // main rotor
        hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_HELI_DUAL_RSC]), pwm);
        break;
    default:
        // do nothing
        break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeli_Dual::get_motor_mask()
{
    // dual heli uses channels 1,2,3,4,5,6 and 8
    return (1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << 4 | 1U << 5 | 1U << 6 | 1U << AP_MOTORS_HELI_DUAL_RSC);
}

//
// protected methods
//

// init_swash
void AP_MotorsHeli_Dual::init_swash()
{
    AP_MotorsHeli::init_swash();
}

// init_servos
void AP_MotorsHeli_Dual::init_servos()
{
    init_swash_servo (_servo_1);
    init_swash_servo (_servo_2);
    init_swash_servo (_servo_3);
    init_swash_servo (_servo_4);
    init_swash_servo (_servo_5);
    init_swash_servo (_servo_6);
}

// reset_swash
void AP_MotorsHeli_Dual::reset_swash()
{
    AP_MotorsHeli::reset_swash();

    _yaw_scaler = 1.0f;
}

// reset_servos
void AP_MotorsHeli_Dual::reset_servos()
{
    reset_swash_servo (_servo_1);
    reset_swash_servo (_servo_2);
    reset_swash_servo (_servo_3);
    reset_swash_servo (_servo_4);
    reset_swash_servo (_servo_5);
    reset_swash_servo (_servo_6);
}

// calculate_swash_factors - calculate factors based on swash type and servo position
void AP_MotorsHeli_Dual::calculate_swash_factors()
{
    if (_dual_mode == AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE) {
        // roll factors
        _roll_factors[CH_1] = _dcp_scaler;
        _roll_factors[CH_2] = _dcp_scaler;
        _roll_factors[CH_3] = _dcp_scaler;

        _roll_factors[CH_4] = -_dcp_scaler;
        _roll_factors[CH_5] = -_dcp_scaler;
        _roll_factors[CH_6] = -_dcp_scaler;

        // pitch factors
        _pitch_factors[CH_1] = cosf(radians(_servo1_pos - _swash1_phase_angle));
        _pitch_factors[CH_2] = cosf(radians(_servo2_pos - _swash1_phase_angle));
        _pitch_factors[CH_3] = cosf(radians(_servo3_pos - _swash1_phase_angle));

        _pitch_factors[CH_4] = cosf(radians(_servo4_pos - _swash2_phase_angle));
        _pitch_factors[CH_5] = cosf(radians(_servo5_pos - _swash2_phase_angle));
        _pitch_factors[CH_6] = cosf(radians(_servo6_pos - _swash2_phase_angle));

        // yaw factors
        _yaw_factors[CH_1] = cosf(radians(_servo1_pos + 180 - _swash1_phase_angle));
        _yaw_factors[CH_2] = cosf(radians(_servo2_pos + 180 - _swash1_phase_angle));
        _yaw_factors[CH_3] = cosf(radians(_servo3_pos + 180 - _swash1_phase_angle));

        _yaw_factors[CH_4] = cosf(radians(_servo4_pos - _swash2_phase_angle));
        _yaw_factors[CH_5] = cosf(radians(_servo5_pos - _swash2_phase_angle));
        _yaw_factors[CH_6] = cosf(radians(_servo6_pos - _swash2_phase_angle));
    } else { // AP_MOTORS_HELI_DUAL_MODE_TANDEM
        // roll factors
        _roll_factors[CH_1] = cosf(radians(_servo1_pos + 90 - _swash1_phase_angle));
        _roll_factors[CH_2] = cosf(radians(_servo2_pos + 90 - _swash1_phase_angle));
        _roll_factors[CH_3] = cosf(radians(_servo3_pos + 90 - _swash1_phase_angle));

        _roll_factors[CH_4] = cosf(radians(_servo4_pos + 90 - _swash2_phase_angle));
        _roll_factors[CH_5] = cosf(radians(_servo5_pos + 90 - _swash2_phase_angle));
        _roll_factors[CH_6] = cosf(radians(_servo6_pos + 90 - _swash2_phase_angle));

        // pitch factors
        _pitch_factors[CH_1] = _dcp_scaler;
        _pitch_factors[CH_2] = _dcp_scaler;
        _pitch_factors[CH_3] = _dcp_scaler;

        _pitch_factors[CH_4] = -_dcp_scaler;
        _pitch_factors[CH_5] = -_dcp_scaler;
        _pitch_factors[CH_6] = -_dcp_scaler;

        // yaw factors
        _yaw_factors[CH_1] = cosf(radians(_servo1_pos + 90 - _swash1_phase_angle));
        _yaw_factors[CH_2] = cosf(radians(_servo2_pos + 90 - _swash1_phase_angle));
        _yaw_factors[CH_3] = cosf(radians(_servo3_pos + 90 - _swash1_phase_angle));

        _yaw_factors[CH_4] = cosf(radians(_servo4_pos + 270 - _swash2_phase_angle));
        _yaw_factors[CH_5] = cosf(radians(_servo5_pos + 270 - _swash2_phase_angle));
        _yaw_factors[CH_6] = cosf(radians(_servo6_pos + 270 - _swash2_phase_angle));
    }

    // collective factors
    _collective_factors[CH_1] = 1;
    _collective_factors[CH_2] = 1;
    _collective_factors[CH_3] = 1;

    _collective_factors[CH_4] = 1;
    _collective_factors[CH_5] = 1;
    _collective_factors[CH_6] = 1;
}

void AP_MotorsHeli_Dual::set_dt(float dt)
{
    _rotor.set_dt(dt);
}

void AP_MotorsHeli_Dual::recalc_scalers()
{
    AP_MotorsHeli::recalc_scalers();

    _yaw_scaler = (float)_yaw_max/4500.0f;

    _rotor.set_ramp_time(_rsc_ramp_time);
    _rotor.set_runup_time(_rsc_runup_time);

    _rotor.recalc_scalers();
}

void AP_MotorsHeli_Dual::set_speed_target(int16_t speed_target)
{
    _rotor.set_speed_target(speed_target);
}

void AP_MotorsHeli_Dual::output_armed()
{
    AP_MotorsHeli::output_armed();

    _rotor.output();

    _heliflags.motor_runup_complete = _rotor.is_runup_complete();
}

void AP_MotorsHeli_Dual::output_disarmed()
{
    AP_MotorsHeli::output_disarmed();

    _rotor.output();

    _heliflags.motor_runup_complete = false;
}

//
// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 0 ~ 1000
//                       yaw:   -4500 ~ 4500
//
void AP_MotorsHeli_Dual::move_swash(int16_t roll_out, int16_t pitch_out, int16_t collective_in, int16_t yaw_out)
{
    int16_t collective_out_scaled;
    int16_t yaw_compensation;

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

        // add differential collective pitch yaw compensation
        if (_dual_mode == AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE) {
            yaw_compensation = _dcp_yaw_effect * roll_out;
        } else { // AP_MOTORS_HELI_DUAL_MODE_TANDEM
            yaw_compensation = _dcp_yaw_effect * pitch_out;
        }
        yaw_out = yaw_out + yaw_compensation;

        // scale yaw and update limits
        yaw_out = yaw_out * _yaw_scaler;
        if (yaw_out < -_yaw_max) {
            yaw_out = -_yaw_max;
            limit.yaw = true;
        }
        if (yaw_out > _yaw_max) {
            yaw_out = _yaw_max;
            limit.yaw = true;
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
    }

    // swashplate servos
    _servo_1.servo_out = (_roll_factors[CH_1] * roll_out + _pitch_factors[CH_1] * pitch_out + _yaw_factors[CH_1] * yaw_out)/10 + _collective_factors[CH_1] * collective_out_scaled + (_servo_1.radio_trim-1500);
    _servo_2.servo_out = (_roll_factors[CH_2] * roll_out + _pitch_factors[CH_2] * pitch_out + _yaw_factors[CH_2] * yaw_out)/10 + _collective_factors[CH_2] * collective_out_scaled + (_servo_2.radio_trim-1500);
    _servo_3.servo_out = (_roll_factors[CH_3] * roll_out + _pitch_factors[CH_3] * pitch_out + _yaw_factors[CH_3] * yaw_out)/10 + _collective_factors[CH_3] * collective_out_scaled + (_servo_3.radio_trim-1500);
    _servo_4.servo_out = (_roll_factors[CH_4] * roll_out + _pitch_factors[CH_4] * pitch_out + _yaw_factors[CH_4] * yaw_out)/10 + _collective_factors[CH_4] * collective_out_scaled + (_servo_4.radio_trim-1500);
    _servo_5.servo_out = (_roll_factors[CH_5] * roll_out + _pitch_factors[CH_5] * pitch_out + _yaw_factors[CH_5] * yaw_out)/10 + _collective_factors[CH_5] * collective_out_scaled + (_servo_5.radio_trim-1500);
    _servo_6.servo_out = (_roll_factors[CH_6] * roll_out + _pitch_factors[CH_6] * pitch_out + _yaw_factors[CH_6] * yaw_out)/10 + _collective_factors[CH_6] * collective_out_scaled + (_servo_6.radio_trim-1500);

    // use servo_out to calculate pwm_out and radio_out
    _servo_1.calc_pwm();
    _servo_2.calc_pwm();
    _servo_3.calc_pwm();
    _servo_4.calc_pwm();
    _servo_5.calc_pwm();
    _servo_6.calc_pwm();

    // actually move the servos
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), _servo_1.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), _servo_2.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), _servo_3.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), _servo_4.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_5]), _servo_5.radio_out);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_6]), _servo_6.radio_out);
}