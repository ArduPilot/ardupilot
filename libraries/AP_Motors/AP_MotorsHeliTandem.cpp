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
 *       AP_MotorsHeliTandem.cpp - ArduCopter motors library
 *
 */
#include <stdlib.h>
#include <AP_HAL.h>
#include "AP_MotorsHeliTandem.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeliTandem::var_info[] PROGMEM = {
    // @Param: SV1_POS
    // @DisplayName: Servo 1 Position
    // @Description: Angular location of swash servo #1
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV1_POS", 1, AP_MotorsHeliTandem, _servo1_pos, AP_MOTORS_HELI_TANDEM_SERVO1_POS),

    // @Param: SV2_POS
    // @DisplayName: Servo 2 Position
    // @Description: Angular location of swash servo #2
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV2_POS", 2, AP_MotorsHeliTandem, _servo2_pos,  AP_MOTORS_HELI_TANDEM_SERVO2_POS),

    // @Param: SV3_POS
    // @DisplayName: Servo 3 Position
    // @Description: Angular location of swash servo #3
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV3_POS", 3, AP_MotorsHeliTandem, _servo3_pos,  AP_MOTORS_HELI_TANDEM_SERVO3_POS),

    // @Param: SV4_POS
    // @DisplayName: Servo 4 Position
    // @Description: Angular location of swash servo #4
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV4_POS", 4, AP_MotorsHeliTandem,  _servo4_pos, AP_MOTORS_HELI_TANDEM_SERVO4_POS),

    // @Param: SV5_POS
    // @DisplayName: Servo 5 Position
    // @Description: Angular location of swash servo #5
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV5_POS", 5, AP_MotorsHeliTandem, _servo5_pos, AP_MOTORS_HELI_TANDEM_SERVO5_POS),

    // @Param: SV6_POS
    // @DisplayName: Servo 6 Position
    // @Description: Angular location of swash servo #6
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV6_POS", 6, AP_MotorsHeliTandem, _servo6_pos, AP_MOTORS_HELI_TANDEM_SERVO6_POS),

    // @Param: ROL_MAX
    // @DisplayName: Swash Roll Angle Max
    // @Description: Maximum roll angle of the swash plate
    // @Range: 0 18000
    // @Units: Centi-Degrees
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("ROL_MAX", 10, AP_MotorsHeliTandem, _roll_max, AP_MOTORS_HELI_TANDEM_SWASH_ROLL_MAX),

    // @Param: PIT_MAX
    // @DisplayName: Swash Pitch Angle Max
    // @Description: Maximum pitch angle of the swash plate
    // @Range: 0 18000
    // @Units: Centi-Degrees
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("PIT_MAX", 11, AP_MotorsHeliTandem, _pitch_max, AP_MOTORS_HELI_TANDEM_SWASH_PITCH_MAX),

    // @Param: YAW_MAX
    // @DisplayName: Swash Yaw Angle Max
    // @Description: Maximum yaw angle of the swash plate
    // @Range: 0 18000
    // @Units: Centi-Degrees
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("YAW_MAX", 12, AP_MotorsHeliTandem, _yaw_max, AP_MOTORS_HELI_TANDEM_SWASH_YAW_MAX),

    // @Param: COL_MIN
    // @DisplayName: Collective Pitch Minimum
    // @Description: Lowest possible servo position for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MIN", 13, AP_MotorsHeliTandem, _collective_min, AP_MOTORS_HELI_TANDEM_COLLECTIVE_MIN),

    // @Param: COL_MAX
    // @DisplayName: Collective Pitch Maximum
    // @Description: Highest possible servo position for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MAX", 14, AP_MotorsHeliTandem, _collective_max, AP_MOTORS_HELI_TANDEM_COLLECTIVE_MAX),

    // @Param: COL_MID
    // @DisplayName: Collective Pitch Mid-Point
    // @Description: Swash servo position corresponding to zero collective pitch (or zero lift for Assymetrical blades)
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MID", 15, AP_MotorsHeliTandem, _collective_mid, AP_MOTORS_HELI_TANDEM_COLLECTIVE_MID),

    // @Param: SV_MAN
    // @DisplayName: Manual Servo Mode
    // @Description: Pass radio inputs directly to servos for set-up. Do not set this manually!
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SV_MAN", 16, AP_MotorsHeliTandem, _servo_manual,  0),

    // @Param: PHANG1
    // @DisplayName: Swashplate 1 Phase Angle Compensation
    // @Description: Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -90 90
    // @Units: Degrees
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG1", 17, AP_MotorsHeliTandem, _swash1_phase_angle,   0),

    // @Param: PHANG2
    // @DisplayName: Swashplate 2 Phase Angle Compensation
    // @Description: Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -90 90
    // @Units: Degrees
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG2", 18, AP_MotorsHeliTandem, _swash2_phase_angle,   0),

    // @Param: GOV_SETPOINT
    // @DisplayName: External Motor Governor Setpoint
    // @Description: PWM passed to the external motor governor when external governor is enabled
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_SETPOINT", 19, AP_MotorsHeliTandem, _rsc_setpoint, AP_MOTORS_HELI_TANDEM_RSC_SETPOINT),

    // @Param: RSC_MODE
    // @DisplayName: Rotor Speed Control Mode
    // @Description: Controls the source of the desired rotor speed, either ch8 or RSC_SETPOINT
    // @Values: 0:None, 1:Ch8 Input, 2:SetPoint
    // @User: Standard
    AP_GROUPINFO("RSC_MODE", 20, AP_MotorsHeliTandem, _rsc_mode, AP_MOTORS_HELI_TANDEM_RSC_MODE_CH8_PASSTHROUGH),

    // @Param: TANDEM_MODE
    // @DisplayName: Tandem Mode
    // @Description: Sets the tandem mode of the heli, either as longitudinal or as transverse.
    // @Values: 0:Longitudinal, 1:Transverse
    // @User: Standard
    AP_GROUPINFO("TANDEM_MODE", 21, AP_MotorsHeliTandem, _tandem_mode, AP_MOTORS_HELI_TANDEM_MODE_LONGITUDINAL),

    // @Param: LAND_COL_MIN
    // @DisplayName: Landing Collective Minimum
    // @Description: Minimum collective position while landed or landing
    // @Range: 0 500
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LAND_COL_MIN", 22, AP_MotorsHeliTandem, _land_collective_min, AP_MOTORS_HELI_TANDEM_LAND_COLLECTIVE_MIN),

    // @Param: RSC_RAMP_TIME
    // @DisplayName: RSC Ramp Time
    // @Description: Time in seconds for the output to the main rotor's ESC to reach full speed
    // @Range: 0 60
    // @Units: Seconds
    // @User: Standard
    AP_GROUPINFO("RSC_RAMP_TIME", 23, AP_MotorsHeliTandem,_rsc_ramp_time, AP_MOTORS_HELI_TANDEM_RSC_RAMP_TIME),

    // @Param: RSC_RUNUP_TIME
    // @DisplayName: RSC Runup Time
    // @Description: Time in seconds for the main rotor to reach full speed.  Must be longer than RSC_RAMP_TIME
    // @Range: 0 60
    // @Units: Seconds
    // @User: Standard
    AP_GROUPINFO("RSC_RUNUP_TIME", 24, AP_MotorsHeliTandem,_rsc_runup_time, AP_MOTORS_HELI_TANDEM_RSC_RUNUP_TIME),

    // @Param: DCP_SCALER
    // @DisplayName: Differential-Collective-Pitch Scaler
    // @Description: Scaling factor applied to the differential-collective-pitch 
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("DCP_SCALER", 25, AP_MotorsHeliTandem, _dcp_scaler, AP_MOTORS_HELI_TANDEM_DCP_SCALER),

    AP_GROUPEND
};

//
// public methods
//

// init
void AP_MotorsHeliTandem::Init()
{
    // set update rate
    set_update_rate(_speed_hz);

    // ensure inputs are not passed through to servos
    _servo_manual = 0;

    // initialise some scalers
    recalc_scalers();

    // initialise swash plate
    init_swash();

    // disable channels 8 from being used by RC_Channel_aux
    RC_Channel_aux::disable_aux_channel(_motor_to_channel_map[AP_MOTORS_HELI_TANDEM_RSC]);
}

// set update rate to motors - a value in hertz
void AP_MotorsHeliTandem::set_update_rate( uint16_t speed_hz )
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

// enable - starts allowing signals to be sent to motors
void AP_MotorsHeliTandem::enable()
{
    // enable output channels
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]));    // swash servo 1
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]));    // swash servo 2
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]));    // swash servo 3
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]));    // swash servo 4
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_5]));    // swash servo 5
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_6]));    // swash servo 6
    hal.rcout->enable_ch(AP_MOTORS_HELI_TANDEM_RSC);                               // output for main rotor esc
}

// output_min - sends minimum values out to the motors
void AP_MotorsHeliTandem::output_min()
{
    // move swash to mid
    move_swash(0,0,500,0);

    // override limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = false;
}


// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeliTandem::output_test(uint8_t motor_seq, int16_t pwm)
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
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_HELI_TANDEM_RSC]), pwm);
            break;
        default:
            // do nothing
            break;
    }
}

// allow_arming - returns true if main rotor is spinning and it is ok to arm
bool AP_MotorsHeliTandem::allow_arming() const
{
    // ensure main rotor has started
    if (_rsc_mode != AP_MOTORS_HELI_TANDEM_RSC_MODE_NONE && _servo_rsc.control_in > 0) {
        return false;
    }

    // all other cases it is ok to arm
    return true;
}

// set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1000
void AP_MotorsHeliTandem::set_desired_rotor_speed(int16_t desired_speed)
{
    _rotor_desired = desired_speed;
}

// return true if the main rotor is up to speed
bool AP_MotorsHeliTandem::motor_runup_complete() const
{
    // if we have no control of motors, assume pilot has spun them up
    if (_rsc_mode == AP_MOTORS_HELI_TANDEM_RSC_MODE_NONE) {
        return true;
    }

    return _heliflags.motor_runup_complete;
}

// recalc_scalers - recalculates various scalers used.  Should be called at about 1hz to allow users to see effect of changing parameters
void AP_MotorsHeliTandem::recalc_scalers()
{
    // recalculate rotor ramp up increment
    if (_rsc_ramp_time <= 0) {
        _rsc_ramp_time = 1;
    }
    _rsc_ramp_increment = 1000.0f / (_rsc_ramp_time / _dt);

    // recalculate rotor runup increment
    if (_rsc_runup_time <= 0 ) {
        _rsc_runup_time = 1;
    }
    if (_rsc_runup_time < _rsc_ramp_time) {
        _rsc_runup_time = _rsc_ramp_time;
    }
    _rsc_runup_increment = 1000.0f / (_rsc_runup_time * 100.0f);
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeliTandem::get_motor_mask()
{
    // heli uses channels 1,2,3,4,7 and 8
    return (1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << 4 | 1U << 5 | 1U << 6 | 1U << AP_MOTORS_HELI_TANDEM_RSC);
}

//
// protected methods
//

// output_armed - sends commands to the motors
void AP_MotorsHeliTandem::output_armed()
{
    // if manual override (i.e. when setting up swash), pass pilot commands straight through to swash
    if (_servo_manual == 1) {
        _rc_roll.servo_out = _rc_roll.control_in;
        _rc_pitch.servo_out = _rc_pitch.control_in;
        _rc_throttle.servo_out = _rc_throttle.control_in;
        _rc_yaw.servo_out = _rc_yaw.control_in;
    }

    _rc_roll.calc_pwm();
    _rc_pitch.calc_pwm();
    _rc_throttle.calc_pwm();
    _rc_yaw.calc_pwm();

    move_swash(_rc_roll.servo_out, _rc_pitch.servo_out, _rc_throttle.servo_out, _rc_yaw.servo_out);

    // update rotor and direct drive esc speeds
    rsc_control();
}

// output_disarmed - sends commands to the motors
void AP_MotorsHeliTandem::output_disarmed()
{
    // for helis - armed or disarmed we allow servos to move
    output_armed();
}

//
// private methods
//

// reset_swash - free up swash for maximum movements. Used for set-up
void AP_MotorsHeliTandem::reset_swash()
{
    // free up servo ranges
    _servo_1.radio_min = 1000;
    _servo_1.radio_max = 2000;
    _servo_2.radio_min = 1000;
    _servo_2.radio_max = 2000;
    _servo_3.radio_min = 1000;
    _servo_3.radio_max = 2000;
    _servo_4.radio_min = 1000;
    _servo_4.radio_max = 2000;
    _servo_5.radio_min = 1000;
    _servo_5.radio_max = 2000;
    _servo_6.radio_min = 1000;
    _servo_6.radio_max = 2000;

    // calculate factors based on swash type and servo position
    calculate_roll_pitch_collective_factors();

    // set roll, pitch, yaw and throttle scaling
    _roll_scaler = 1.0f;
    _pitch_scaler = 1.0f;
    _yaw_scaler = 1.0f;
    _collective_scalar = ((float)(_rc_throttle.radio_max - _rc_throttle.radio_min))/1000.0f;
	_collective_scalar_manual = 1.0f;

    // we must be in set-up mode so mark swash as uninitialised
    _heliflags.swash_initialised = false;
}

// init_swash - initialise the swash plate
void AP_MotorsHeliTandem::init_swash()
{
    // swash servo initialisation
    _servo_1.set_range(0,1000);
    _servo_2.set_range(0,1000);
    _servo_3.set_range(0,1000);
    _servo_4.set_range(0,1000);
    _servo_5.set_range(0,1000);
    _servo_6.set_range(0,1000);

    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min = 1000;
        _collective_max = 2000;
    }
    _collective_mid = constrain_int16(_collective_mid, _collective_min, _collective_max);

    // calculate collective mid point as a number from 0 to 1000
    _collective_mid_pwm = ((float)(_collective_mid-_collective_min))/((float)(_collective_max-_collective_min))*1000.0f;

    // determine roll, pitch and collective input scaling
    _roll_scaler = (float)_roll_max/4500.0f;
    _pitch_scaler = (float)_pitch_max/4500.0f;
    _yaw_scaler = (float)_yaw_max/4500.0f;

    _collective_scalar = ((float)(_collective_max-_collective_min))/1000.0f;

    // calculate factors based on swash type and servo position
    calculate_roll_pitch_collective_factors();

    // servo min/max values
    _servo_1.radio_min = 1000;
    _servo_1.radio_max = 2000;
    _servo_2.radio_min = 1000;
    _servo_2.radio_max = 2000;
    _servo_3.radio_min = 1000;
    _servo_3.radio_max = 2000;
    _servo_4.radio_min = 1000;
    _servo_4.radio_max = 2000;
    _servo_5.radio_min = 1000;
    _servo_5.radio_max = 2000;
    _servo_6.radio_min = 1000;
    _servo_6.radio_max = 2000;

    // mark swash as initialised
    _heliflags.swash_initialised = true;
}

// calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
void AP_MotorsHeliTandem::calculate_roll_pitch_collective_factors()
{    
    if (_tandem_mode == AP_MOTORS_HELI_TANDEM_MODE_TRANSVERSE) {
        // roll factors
        _rollFactor[CH_1] = _dcp_scaler;
        _rollFactor[CH_2] = _dcp_scaler;
        _rollFactor[CH_3] = _dcp_scaler;

        _rollFactor[CH_4] = -_dcp_scaler;
        _rollFactor[CH_5] = -_dcp_scaler;
        _rollFactor[CH_6] = -_dcp_scaler;

         // pitch factors
        _pitchFactor[CH_1] = cosf(radians(_servo1_pos - _swash1_phase_angle));
        _pitchFactor[CH_2] = cosf(radians(_servo2_pos - _swash1_phase_angle));
        _pitchFactor[CH_3] = cosf(radians(_servo3_pos - _swash1_phase_angle));

        _pitchFactor[CH_4] = cosf(radians(_servo4_pos - _swash2_phase_angle));
        _pitchFactor[CH_5] = cosf(radians(_servo5_pos - _swash2_phase_angle));
        _pitchFactor[CH_6] = cosf(radians(_servo6_pos - _swash2_phase_angle));

         // yaw factors
        _yawFactor[CH_1] = cosf(radians(_servo1_pos + 180 - _swash1_phase_angle));
        _yawFactor[CH_2] = cosf(radians(_servo2_pos + 180 - _swash1_phase_angle));
        _yawFactor[CH_3] = cosf(radians(_servo3_pos + 180 - _swash1_phase_angle));

        _yawFactor[CH_4] = cosf(radians(_servo4_pos - _swash2_phase_angle));
        _yawFactor[CH_5] = cosf(radians(_servo5_pos - _swash2_phase_angle));
        _yawFactor[CH_6] = cosf(radians(_servo6_pos - _swash2_phase_angle));
    } else { // AP_MOTORS_HELI_TANDEM_MODE_LONGITUDINAL
        // roll factors
        _rollFactor[CH_1] = cosf(radians(_servo1_pos + 90 - _swash1_phase_angle));
        _rollFactor[CH_2] = cosf(radians(_servo2_pos + 90 - _swash1_phase_angle));
        _rollFactor[CH_3] = cosf(radians(_servo3_pos + 90 - _swash1_phase_angle));

        _rollFactor[CH_4] = cosf(radians(_servo4_pos + 90 - _swash2_phase_angle));
        _rollFactor[CH_5] = cosf(radians(_servo5_pos + 90 - _swash2_phase_angle));
        _rollFactor[CH_6] = cosf(radians(_servo6_pos + 90 - _swash2_phase_angle));

        // pitch factors
        _pitchFactor[CH_1] = _dcp_scaler;
        _pitchFactor[CH_2] = _dcp_scaler;
        _pitchFactor[CH_3] = _dcp_scaler;

        _pitchFactor[CH_4] = -_dcp_scaler;
        _pitchFactor[CH_5] = -_dcp_scaler;
        _pitchFactor[CH_6] = -_dcp_scaler;

         // yaw factors
        _yawFactor[CH_1] = cosf(radians(_servo1_pos + 90 - _swash1_phase_angle));
        _yawFactor[CH_2] = cosf(radians(_servo2_pos + 90 - _swash1_phase_angle));
        _yawFactor[CH_3] = cosf(radians(_servo3_pos + 90 - _swash1_phase_angle));

        _yawFactor[CH_4] = cosf(radians(_servo4_pos + 270 - _swash2_phase_angle));
        _yawFactor[CH_5] = cosf(radians(_servo5_pos + 270 - _swash2_phase_angle));
        _yawFactor[CH_6] = cosf(radians(_servo6_pos + 270 - _swash2_phase_angle));
    }

    // collective factors
    _collectiveFactor[CH_1] = 1;
    _collectiveFactor[CH_2] = 1;
    _collectiveFactor[CH_3] = 1;

    _collectiveFactor[CH_4] = 1;
    _collectiveFactor[CH_5] = 1;
    _collectiveFactor[CH_6] = 1;
}

//
// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 0 ~ 1000
//                       yaw:   -4500 ~ 4500
//
void AP_MotorsHeliTandem::move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out)
{
    int16_t coll_out_scaled;

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
        // _collective_scalar should probably not be used or set to 1?
        coll_out_scaled = coll_in * _collective_scalar + _rc_throttle.radio_min - 1000;
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
        _collective_out = coll_in;
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
        coll_out_scaled = _collective_out * _collective_scalar + _collective_min - 1000;
    }

    // swashplate servos
    _servo_1.servo_out = (_rollFactor[CH_1] * roll_out + _pitchFactor[CH_1] * pitch_out + _yawFactor[CH_1] * yaw_out)/10 + _collectiveFactor[CH_1] * coll_out_scaled + (_servo_1.radio_trim-1500);
    _servo_2.servo_out = (_rollFactor[CH_2] * roll_out + _pitchFactor[CH_2] * pitch_out + _yawFactor[CH_2] * yaw_out)/10 + _collectiveFactor[CH_2] * coll_out_scaled + (_servo_2.radio_trim-1500);
    _servo_3.servo_out = (_rollFactor[CH_3] * roll_out + _pitchFactor[CH_3] * pitch_out + _yawFactor[CH_3] * yaw_out)/10 + _collectiveFactor[CH_3] * coll_out_scaled + (_servo_3.radio_trim-1500);
    _servo_4.servo_out = (_rollFactor[CH_4] * roll_out + _pitchFactor[CH_4] * pitch_out + _yawFactor[CH_4] * yaw_out)/10 + _collectiveFactor[CH_4] * coll_out_scaled + (_servo_4.radio_trim-1500);
    _servo_5.servo_out = (_rollFactor[CH_5] * roll_out + _pitchFactor[CH_5] * pitch_out + _yawFactor[CH_5] * yaw_out)/10 + _collectiveFactor[CH_5] * coll_out_scaled + (_servo_5.radio_trim-1500);
    _servo_6.servo_out = (_rollFactor[CH_6] * roll_out + _pitchFactor[CH_6] * pitch_out + _yawFactor[CH_6] * yaw_out)/10 + _collectiveFactor[CH_6] * coll_out_scaled + (_servo_6.radio_trim-1500);

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

// rsc_control - update value to send to tail and main rotor's ESC
// desired_rotor_speed is a desired speed from 0 to 1000
void AP_MotorsHeliTandem::rsc_control()
{
    // if disarmed output minimums
    if (!armed()) {
        // shut down main rotor
        if (_rsc_mode != AP_MOTORS_HELI_TANDEM_RSC_MODE_NONE) {
            _rotor_out = 0;
            _rotor_speed_estimate = 0;
            write_rsc(_rotor_out);
        }
        return;
    }

    // ramp up or down main rotor and tail
    if (_rotor_desired > 0) {
        rotor_ramp(_rotor_desired);
    }else{
        // shutting down main rotor
        rotor_ramp(0);
    }
}

// rotor_ramp - ramps rotor towards target
// result put in _rotor_out and sent to ESC
void AP_MotorsHeliTandem::rotor_ramp(int16_t rotor_target)
{
    // return immediately if not ramping required
    if (_rsc_mode == AP_MOTORS_HELI_TANDEM_RSC_MODE_NONE) {
        _rotor_out = rotor_target;
        return;
    }

    // range check rotor_target
    rotor_target = constrain_int16(rotor_target,0,1000);

    // ramp rotor esc output towards target
    if (_rotor_out < rotor_target) {
        // allow rotor out to jump to rotor's current speed
        if (_rotor_out < _rotor_speed_estimate) {
            _rotor_out = _rotor_speed_estimate;
        }
        // ramp up slowly to target
        _rotor_out += _rsc_ramp_increment;
        if (_rotor_out > rotor_target) {
            _rotor_out = rotor_target;
        }
    }else{
        // ramping down happens instantly
        _rotor_out = rotor_target;
    }

    // ramp rotor speed estimate towards rotor out
    if (_rotor_speed_estimate < _rotor_out) {
        _rotor_speed_estimate += _rsc_runup_increment;
        if (_rotor_speed_estimate > _rotor_out) {
            _rotor_speed_estimate = _rotor_out;
        }
    }else{
        _rotor_speed_estimate -= _rsc_runup_increment;
        if (_rotor_speed_estimate < _rotor_out) {
            _rotor_speed_estimate = _rotor_out;
        }
    }

    // set runup complete flag
    if (!_heliflags.motor_runup_complete && rotor_target > 0 && _rotor_speed_estimate >= rotor_target) {
        _heliflags.motor_runup_complete = true;
    }
    if (_heliflags.motor_runup_complete && rotor_target == 0 && _rotor_speed_estimate <= 0) {
        _heliflags.motor_runup_complete = false;
    }

    // output to rsc servo
    write_rsc(_rotor_out);
}

// write_rsc - outputs pwm onto output rsc channel (ch8)
// servo_out parameter is of the range 0 ~ 1000
void AP_MotorsHeliTandem::write_rsc(int16_t servo_out)
{
    _servo_rsc.servo_out = servo_out;
    _servo_rsc.calc_pwm();
    hal.rcout->write(AP_MOTORS_HELI_TANDEM_RSC, _servo_rsc.radio_out);
}