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
 *       AP_MotorsHeli.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <stdlib.h>
#include <AP_HAL.h>
#include "AP_MotorsHeli.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli::var_info[] PROGMEM = {


    // @Param: SV1_POS
    // @DisplayName: Servo 1 Position
    // @Description: Angular location of swash servo #1
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV1_POS", 1,      AP_MotorsHeli,  _servo1_pos, AP_MOTORS_HELI_SERVO1_POS),

    // @Param: SV2_POS
    // @DisplayName: Servo 2 Position
    // @Description: Angular location of swash servo #2
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV2_POS", 2,      AP_MotorsHeli,  _servo2_pos,  AP_MOTORS_HELI_SERVO2_POS),

    // @Param: SV3_POS
    // @DisplayName: Servo 3 Position
    // @Description: Angular location of swash servo #3
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV3_POS", 3,      AP_MotorsHeli,  _servo3_pos,  AP_MOTORS_HELI_SERVO3_POS),

    // @Param: ROL_MAX
    // @DisplayName: Swash Roll Angle Max
    // @Description: Maximum roll angle of the swash plate
    // @Range: 0 18000
    // @Units: Centi-Degrees
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("ROL_MAX", 4,      AP_MotorsHeli,  _roll_max,    AP_MOTORS_HELI_SWASH_ROLL_MAX),

    // @Param: PIT_MAX
    // @DisplayName: Swash Pitch Angle Max
    // @Description: Maximum pitch angle of the swash plate
    // @Range: 0 18000
    // @Units: Centi-Degrees
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("PIT_MAX", 5,      AP_MotorsHeli,  _pitch_max,   AP_MOTORS_HELI_SWASH_PITCH_MAX),

    // @Param: COL_MIN
    // @DisplayName: Collective Pitch Minimum
    // @Description: Lowest possible servo position for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MIN", 6,      AP_MotorsHeli,  _collective_min, AP_MOTORS_HELI_COLLECTIVE_MIN),

    // @Param: COL_MAX
    // @DisplayName: Collective Pitch Maximum
    // @Description: Highest possible servo position for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MAX", 7,      AP_MotorsHeli,  _collective_max, AP_MOTORS_HELI_COLLECTIVE_MAX),

    // @Param: COL_MID
    // @DisplayName: Collective Pitch Mid-Point
    // @Description: Swash servo position corresponding to zero collective pitch (or zero lift for Assymetrical blades)
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MID", 8,      AP_MotorsHeli,  _collective_mid, AP_MOTORS_HELI_COLLECTIVE_MID),

    // @Param: GYR_ENABLE
    // @DisplayName: External Gyro Enabled
    // @Description: Enabled/Disable an external rudder gyro connected to channel 7.  With no external gyro a more complex yaw controller is used
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GYR_ENABLE",9,    AP_MotorsHeli,  _ext_gyro_enabled, 0),

    // @Param: SWASH_TYPE
    // @DisplayName: Swash Type
    // @Description: Swash Type Setting - either 3-servo CCPM or H1 Mechanical Mixing
    // @Values: 0:3-Servo CCPM, 1:H1 Mechanical Mixing
    // @User: Standard
    AP_GROUPINFO("SWASH_TYPE",10,   AP_MotorsHeli,  _swash_type, AP_MOTORS_HELI_SWASH_CCPM),

    // @Param: GYR_GAIN
    // @DisplayName: External Gyro Gain
    // @Description: PWM sent to the external gyro on Ch7
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("GYR_GAIN",11,     AP_MotorsHeli,  _ext_gyro_gain, AP_MOTORS_HELI_EXT_GYRO_GAIN),

    // @Param: SV_MAN
    // @DisplayName: Manual Servo Mode
    // @Description: Pass radio inputs directly to servos for set-up. Do not set this manually!
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SV_MAN",  12,     AP_MotorsHeli,  _servo_manual,  0),

    // @Param: PHANG
    // @DisplayName: Swashplate Phase Angle Compensation
    // @Description: Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -90 90
    // @Units: Degrees
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG",   13,     AP_MotorsHeli,  _phase_angle,   0),

    // @Param: COLYAW
    // @DisplayName: Collective-Yaw Mixing
    // @Description: Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    // @Range: -10 10
    AP_GROUPINFO("COLYAW",  14,     AP_MotorsHeli,  _collective_yaw_effect, 0),

    // @Param: GOV_SETPOINT
    // @DisplayName: External Motor Governor Setpoint
    // @Description: PWM passed to the external motor governor when external governor is enabled
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("GOV_SETPOINT", 15, AP_MotorsHeli, _ext_gov_setpoint, AP_MOTORS_HELI_EXT_GOVERNOR_SETPOINT),

    // @Param: RSC_MODE
    // @DisplayName: Rotor Speed Control Mode
    // @Description: Which main rotor ESC control mode is active
    // @Values: 0:None, 1:Ch8 passthrough, 2:External Governor
    // @User: Standard
    AP_GROUPINFO("RSC_MODE", 16, AP_MotorsHeli,     _rsc_mode, AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH),

    // @Param: RSC_RATE
    // @DisplayName: RSC Ramp Rate
    // @Description: The time in 100th seconds the RSC takes to ramp up to speed
    // @Range: 0 6000
    // @Units: 100ths of Seconds
    // @User: Standard
    AP_GROUPINFO("RSC_RATE", 17, AP_MotorsHeli,     _rsc_ramp_up_rate, AP_MOTORS_HELI_RSC_RATE),

    // @Param: FLYBAR_MODE
    // @DisplayName: Flybar Mode Selector
    // @Description: Flybar present or not.  Affects attitude controller used during ACRO flight mode
    // @Range: 0:NoFlybar 1:Flybar
    // @User: Standard
    AP_GROUPINFO("FLYBAR_MODE", 18, AP_MotorsHeli,  _flybar_mode, AP_MOTORS_HELI_NOFLYBAR),

    // @Param: STAB_COL_MIN
    // @DisplayName: Stabilize Throttle Minimum
    // @Description: Minimum collective position while pilot directly controls collective
    // @Range: 0 50
    // @Units: Percent
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_MIN", 19, AP_MotorsHeli, _manual_collective_min, AP_MOTORS_HELI_MANUAL_COLLECTIVE_MIN),
	
    // @Param: STAB_COL_MAX
    // @DisplayName: Stabilize Throttle Maximum
    // @Description: Maximum collective position while pilot directly controls collective
    // @Range: 50 100
    // @Units: Percent
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_MAX", 20, AP_MotorsHeli, _manual_collective_max, AP_MOTORS_HELI_MANUAL_COLLECTIVE_MAX),

    // @Param: LAND_COL_MIN
    // @DisplayName: Landing Collective Minimum
    // @Description: Minimum collective position while landed or landing
    // @Range: 0 500
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LAND_COL_MIN", 21, AP_MotorsHeli, _land_collective_min, AP_MOTORS_HELI_LAND_COLLECTIVE_MIN),

    AP_GROUPEND
};

//
// public methods
//

// init
void AP_MotorsHeli::Init()
{
    // set update rate
    set_update_rate(_speed_hz);

    // ensure inputs are not passed through to servos
    _servo_manual = 0;

    // initialise swash plate
    init_swash();
}

// set update rate to motors - a value in hertz
void AP_MotorsHeli::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = 
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_1] |
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_2] |
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_3] |
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_4];
    hal.rcout->set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsHeli::enable()
{
    // enable output channels
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_1]);            // swash servo 1
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_2]);            // swash servo 2
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_3]);            // swash servo 3
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_4]);            // yaw
    hal.rcout->enable_ch(AP_MOTORS_HELI_EXT_GYRO);           // for external gyro
    hal.rcout->enable_ch(AP_MOTORS_HELI_EXT_RSC);            // for external RSC
}

// output_min - sends minimum values out to the motors
void AP_MotorsHeli::output_min()
{
    // move swash to mid
    move_swash(0,0,500,0);

    // override limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = false;
}


// output_test - wiggle servos in order to show connections are correct
void AP_MotorsHeli::output_test()
{
    int16_t i;
    // Send minimum values to all motors
    output_min();

    // servo 1
    for( i=0; i<5; i++ ) {
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo_1->radio_trim + 100);
        hal.scheduler->delay(300);
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo_1->radio_trim - 100);
        hal.scheduler->delay(300);
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo_1->radio_trim + 0);
        hal.scheduler->delay(300);
    }

    // servo 2
    for( i=0; i<5; i++ ) {
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo_2->radio_trim + 100);
        hal.scheduler->delay(300);
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo_2->radio_trim - 100);
        hal.scheduler->delay(300);
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo_2->radio_trim + 0);
        hal.scheduler->delay(300);
    }

    // servo 3
    for( i=0; i<5; i++ ) {
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo_3->radio_trim + 100);
        hal.scheduler->delay(300);
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo_3->radio_trim - 100);
        hal.scheduler->delay(300);
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo_3->radio_trim + 0);
        hal.scheduler->delay(300);
    }

    // external gyro
    if (_ext_gyro_enabled) {
        hal.rcout->write(AP_MOTORS_HELI_EXT_GYRO, _ext_gyro_gain);
    }

    // servo 4
    for( i=0; i<5; i++ ) {
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo_4->radio_trim + 100);
        hal.scheduler->delay(300);
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo_4->radio_trim - 100);
        hal.scheduler->delay(300);
        hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo_4->radio_trim + 0);
        hal.scheduler->delay(300);
    }

    // Send minimum values to all motors
    output_min();
}

// allow_arming - returns true if main rotor is spinning and it is ok to arm
bool AP_MotorsHeli::allow_arming()
{
    // ensure main rotor has started
    if (_rsc_mode != AP_MOTORS_HELI_RSC_MODE_NONE && _rc_8->control_in >= 10) {
        return false;
    }

    // all other cases it is ok to arm
    return true;
}

// get_pilot_desired_collective - converts pilot input (from 0 ~ 1000) to a value that can be fed into the move_swash function
int16_t AP_MotorsHeli::get_pilot_desired_collective(int16_t control_in)
{
    // return immediately if reduce collective range for manual flight has not been configured
    if (_manual_collective_min == 0 && _manual_collective_max == 100) {
        return control_in;
    }

    // scale 
    int16_t collective_out;
    collective_out = _manual_collective_min*10 + control_in * _collective_scalar_manual;
    collective_out = constrain_int16(collective_out, 0, 1000);
    return collective_out;
}

// return true if the main rotor is up to speed
bool AP_MotorsHeli::motor_runup_complete()
{
    // if we have no control of motors, assume pilot has spun them up
    if (_rsc_mode == AP_MOTORS_HELI_RSC_MODE_NONE) {
        return true;
    }

    return _heliflags.motor_runup_complete;
}

//
// protected methods
//

// output_armed - sends commands to the motors
void AP_MotorsHeli::output_armed()
{
    // if manual override (i.e. when setting up swash), pass pilot commands straight through to swash
    if (_servo_manual == 1) {
        _rc_roll->servo_out = _rc_roll->control_in;
        _rc_pitch->servo_out = _rc_pitch->control_in;
        _rc_throttle->servo_out = _rc_throttle->control_in;
        _rc_yaw->servo_out = _rc_yaw->control_in;
    }

    //static int counter = 0;
    _rc_roll->calc_pwm();
    _rc_pitch->calc_pwm();
    _rc_throttle->calc_pwm();
    _rc_yaw->calc_pwm();

    move_swash( _rc_roll->servo_out, _rc_pitch->servo_out, _rc_throttle->servo_out, _rc_yaw->servo_out );

    rsc_control();
}

// output_disarmed - sends commands to the motors
void AP_MotorsHeli::output_disarmed()
{
    // for helis - armed or disarmed we allow servos to move
    output_armed();
}

//
// private methods
//

// reset_swash - free up swash for maximum movements. Used for set-up
void AP_MotorsHeli::reset_swash()
{
    // free up servo ranges
    _servo_1->radio_min = 1000;
    _servo_1->radio_max = 2000;
    _servo_2->radio_min = 1000;
    _servo_2->radio_max = 2000;
    _servo_3->radio_min = 1000;
    _servo_3->radio_max = 2000;

    // calculate factors based on swash type and servo position
    calculate_roll_pitch_collective_factors();

    // set roll, pitch and throttle scaling
    _roll_scaler = 1.0f;
    _pitch_scaler = 1.0f;
    _collective_scalar = ((float)(_rc_throttle->radio_max - _rc_throttle->radio_min))/1000.0f;
	_collective_scalar_manual = 1.0f;

    // we must be in set-up mode so mark swash as uninitialised
    _heliflags.swash_initialised = false;
}

// init_swash - initialise the swash plate
void AP_MotorsHeli::init_swash()
{

    // swash servo initialisation
    _servo_1->set_range(0,1000);
    _servo_2->set_range(0,1000);
    _servo_3->set_range(0,1000);
    _servo_4->set_angle(4500);

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
    _collective_scalar = ((float)(_collective_max-_collective_min))/1000.0f;
    _collective_scalar_manual = ((float)(_manual_collective_max - _manual_collective_min))/100.0f;

    // calculate factors based on swash type and servo position
    calculate_roll_pitch_collective_factors();

    // servo min/max values
    _servo_1->radio_min = 1000;
    _servo_1->radio_max = 2000;
    _servo_2->radio_min = 1000;
    _servo_2->radio_max = 2000;
    _servo_3->radio_min = 1000;
    _servo_3->radio_max = 2000;

    // mark swash as initialised
    _heliflags.swash_initialised = true;
}

// calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
void AP_MotorsHeli::calculate_roll_pitch_collective_factors()
{
    if (_swash_type == AP_MOTORS_HELI_SWASH_CCPM) {                     //CCPM Swashplate, perform control mixing

        // roll factors
        _rollFactor[CH_1] = cosf(radians(_servo1_pos + 90 - _phase_angle));
        _rollFactor[CH_2] = cosf(radians(_servo2_pos + 90 - _phase_angle));
        _rollFactor[CH_3] = cosf(radians(_servo3_pos + 90 - _phase_angle));

        // pitch factors
        _pitchFactor[CH_1] = cosf(radians(_servo1_pos - _phase_angle));
        _pitchFactor[CH_2] = cosf(radians(_servo2_pos - _phase_angle));
        _pitchFactor[CH_3] = cosf(radians(_servo3_pos - _phase_angle));

        // collective factors
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;

    }else{              //H1 Swashplate, keep servo outputs seperated

        // roll factors
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = 0;
        _rollFactor[CH_3] = 0;

        // pitch factors
        _pitchFactor[CH_1] = 0;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = 0;

        // collective factors
        _collectiveFactor[CH_1] = 0;
        _collectiveFactor[CH_2] = 0;
        _collectiveFactor[CH_3] = 1;
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
void AP_MotorsHeli::move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out)
{
    int16_t yaw_offset = 0;
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
        coll_out_scaled = coll_in * _collective_scalar + _rc_throttle->radio_min - 1000;
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
	
        // rudder feed forward based on collective
        if (!_ext_gyro_enabled) {
            yaw_offset = _collective_yaw_effect * abs(coll_out_scaled - _collective_mid_pwm);
        }
    }

    // swashplate servos
    _servo_1->servo_out = (_rollFactor[CH_1] * roll_out + _pitchFactor[CH_1] * pitch_out)/10 + _collectiveFactor[CH_1] * coll_out_scaled + (_servo_1->radio_trim-1500);
    _servo_2->servo_out = (_rollFactor[CH_2] * roll_out + _pitchFactor[CH_2] * pitch_out)/10 + _collectiveFactor[CH_2] * coll_out_scaled + (_servo_2->radio_trim-1500);
    if (_swash_type == AP_MOTORS_HELI_SWASH_H1) {
        _servo_1->servo_out += 500;
        _servo_2->servo_out += 500;
    }
    _servo_3->servo_out = (_rollFactor[CH_3] * roll_out + _pitchFactor[CH_3] * pitch_out)/10 + _collectiveFactor[CH_3] * coll_out_scaled + (_servo_3->radio_trim-1500);
    _servo_4->servo_out = yaw_out + yaw_offset;

    // constrain yaw and update limits
    if (_servo_4->servo_out < -4500) {
        _servo_4->servo_out = -4500;
        limit.yaw = true;
    }
    if (_servo_4->servo_out > 4500) {
        _servo_4->servo_out = 4500;
        limit.yaw = true;
    }

    // use servo_out to calculate pwm_out and radio_out
    _servo_1->calc_pwm();
    _servo_2->calc_pwm();
    _servo_3->calc_pwm();
    _servo_4->calc_pwm();

    // actually move the servos
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo_1->radio_out);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo_2->radio_out);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo_3->radio_out);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo_4->radio_out);

    // to be compatible with other frame types
    motor_out[AP_MOTORS_MOT_1] = _servo_1->radio_out;
    motor_out[AP_MOTORS_MOT_2] = _servo_2->radio_out;
    motor_out[AP_MOTORS_MOT_3] = _servo_3->radio_out;
    motor_out[AP_MOTORS_MOT_4] = _servo_4->radio_out;

    // output gyro value
    if (_ext_gyro_enabled) {
        hal.rcout->write(AP_MOTORS_HELI_EXT_GYRO, _ext_gyro_gain);
    }
}

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// rsc_control - update value to send to main rotor's ESC
void AP_MotorsHeli::rsc_control()
{
    if (armed() && (_rsc_ramp >= _rsc_ramp_up_rate)){                   // rsc_ramp will never increase if rsc_mode = 0
        if (_motor_runup_timer < AP_MOTORS_HELI_MOTOR_RUNUP_TIME){      // therefore motor_runup_complete can never be true
            _motor_runup_timer++;
        } else {
            _heliflags.motor_runup_complete = true;
        }
    } else {
        _heliflags.motor_runup_complete = false;                        // motor_runup_complete will go to false if we
        _motor_runup_timer = 0;                                         // disarm or wind down the motor
    }

    switch (_rsc_mode) {

    case AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH:
        if( armed() && (_rc_8->radio_in > (_rc_8->radio_min + 10))) {
            if (_rsc_ramp < _rsc_ramp_up_rate) {
                _rsc_ramp++;
                _rsc_output = map(_rsc_ramp, 0, _rsc_ramp_up_rate, _rc_8->radio_min, _rc_8->radio_in);
            } else {
                _rsc_output = _rc_8->radio_in;
            }
        } else {
            _rsc_ramp--;                                            //Return RSC Ramp to 0 slowly, allowing for "warm restart"
            if (_rsc_ramp < 0) {
                _rsc_ramp = 0;
            }
            _rsc_output = _rc_8->radio_min;
        }
        hal.rcout->write(AP_MOTORS_HELI_EXT_RSC, _rsc_output);
        break;

    case AP_MOTORS_HELI_RSC_MODE_EXT_GOVERNOR:

        if (armed() && _rc_8->control_in > 100) {
            if (_rsc_ramp < _rsc_ramp_up_rate) {
                _rsc_ramp++;
                _rsc_output = map(_rsc_ramp, 0, _rsc_ramp_up_rate, 1000, _ext_gov_setpoint);
            } else {
                _rsc_output = _ext_gov_setpoint;
            }
        } else {
            _rsc_ramp--;                                          //Return RSC Ramp to 0 slowly, allowing for "warm restart"
            if (_rsc_ramp < 0) {
                _rsc_ramp = 0;
            }
            _rsc_output = 1000;                                  //Just to be sure RSC output is 0
        }
        hal.rcout->write(AP_MOTORS_HELI_EXT_RSC, _rsc_output);
        break;

    default:
        break;
    }
}
