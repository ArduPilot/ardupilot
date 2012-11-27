/*
 *       AP_MotorsHeli.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsHeli.h"

const AP_Param::GroupInfo AP_MotorsHeli::var_info[] PROGMEM = {


    // @Param: SV1_POS
    // @DisplayName: Servo 1 Position
    // @Description: This is the angular location of swash servo #1.
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV1_POS", 1,      AP_MotorsHeli,  servo1_pos, -60),

    // @Param: SV2_POS
    // @DisplayName: Servo 2 Position
    // @Description: This is the angular location of swash servo #2.
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV2_POS", 2,      AP_MotorsHeli,  servo2_pos,  60),

    // @Param: SV3_POS
    // @DisplayName: Servo 3 Position
    // @Description: This is the angular location of swash servo #3.
    // @Range: -180 180
    // @Units: Degrees
    // @User: Standard
    // @Increment: 1
    AP_GROUPINFO("SV3_POS", 3,      AP_MotorsHeli,  servo3_pos,  180),

    // @Param: ROL_MAX
    // @DisplayName: Maximum Roll Angle
    // @Description: This is the maximum allowable roll of the swash plate.
    // @Range: 0 18000
    // @Units: Degrees
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ROL_MAX", 4,      AP_MotorsHeli,  roll_max,    4500),

    // @Param: PIT_MAX
    // @DisplayName: Maximum Pitch Angle
    // @Description: This is the maximum allowable pitch of the swash plate.
    // @Range: 0 18000
    // @Units: Degrees
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PIT_MAX", 5,      AP_MotorsHeli,  pitch_max,   4500),

    // @Param: COL_MIN
    // @DisplayName: Collective Pitch Minimum
    // @Description: This controls the lowest possible servo position for the swashplate.
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MIN", 6,      AP_MotorsHeli,  collective_min, 1250),

    // @Param: COL_MAX
    // @DisplayName: Collective Pitch Maximum
    // @Description: This controls the highest possible servo position for the swashplate.
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MAX", 7,      AP_MotorsHeli,  collective_max, 1750),

    // @Param: COL_MID
    // @DisplayName: Collective Pitch Mid-Point
    // @Description: This is the swash servo position corresponding to zero collective pitch (or zero lift for Assymetrical blades).
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MID", 8,      AP_MotorsHeli,  collective_mid, 1500),

    // @Param: GYR_ENABLE
    // @DisplayName: External Gyro Enabled
    // @Description: Setting this to Enabled(1) will enable an external rudder gyro control which means outputting a gain on channel 7 and using a simpler heading control algorithm. Setting this to Disabled(0) will disable the external gyro gain on channel 7 and revert to a more complex yaw control algorithm.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GYR_ENABLE",      9,      AP_MotorsHeli,  ext_gyro_enabled, 0),

    // @Param: SWASH_TYPE
    // @DisplayName: Swash Plate Type
    // @Description: Setting this to 0 will configure for a 3-servo CCPM. Setting this to 1 will configure for mechanically mixed "H1".
    // @User: Standard
    AP_GROUPINFO("SWASH_TYPE",      10,     AP_MotorsHeli,  swash_type, AP_MOTORS_HELI_SWASH_CCPM),

    // @Param: GYR_GAIN
    // @DisplayName: External Gyro Gain
    // @Description: This is the PWM which is passed to the external gyro when external gyro is enabled.
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GYR_GAIN",        11,     AP_MotorsHeli,  ext_gyro_gain, 1350),

    // @Param: SV_MAN
    // @DisplayName: Manual Servo Mode
    // @Description: Setting this to Enabled(1) will pass radio inputs directly to servos. Setting this to Disabled(0) will enable Arducopter control of servos.  This is only meant to be used by the Mission Planner using swash plate set-up.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SV_MAN",          12,     AP_MotorsHeli,  servo_manual,  0),

    // @Param: PHANG
    // @DisplayName: Swashplate Phase Angle Compensation
    // @Description: This corrects for phase angle errors of the helicopter main rotor head.  For example if pitching the swash forward also induces a roll, that effect can be offset with this parameter.
    // @Range: -90 90
    // @Units: Degrees
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG",           13,     AP_MotorsHeli,  phase_angle,   0),

    // @Param: COLYAW
    // @DisplayName: Collective-Yaw Mixing
    // @Description: This is a feed-forward compensation to automatically add rudder input when collective pitch is increased.
    // @Range: 0 5
    AP_GROUPINFO("COLYAW",          14,     AP_MotorsHeli,  collective_yaw_effect, 0),

    // @Param: GOV_SETPOINT
    // @DisplayName: External Motor Governor Setpoint
    // @Description: This is the PWM which is passed to the external motor governor when external governor is enabled.
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("GOV_SETPOINT", 15, AP_MotorsHeli, ext_gov_setpoint, 1500),

    // @Param: RSC_MODE
    // @DisplayName: Rotor Speed Control Mode
    // @Description: This sets which ESC control mode is active.
    // @Range: 1 3
    // @User: Standard
    AP_GROUPINFO("RSC_MODE", 16, AP_MotorsHeli, rsc_mode, 1),

    // @Param: RSC_RATE
    // @DisplayName: RSC Ramp Rate
    // @Description: This sets the time the RSC takes to ramp up to full speed (Soft Start).
    // @Range: 0 6000
    // @Units: Seconds
    // @User: Standard
    AP_GROUPINFO("RSC_RATE", 17, AP_MotorsHeli, rsc_ramp_up_rate, 1000),

    // @Param: FLYBAR_MODE
    // @DisplayName: Flybar Mode Selector
    // @Description: This sets which acro mode is active. (0) is Flybarless (1) is Mechanical Flybar
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("FLYBAR_MODE", 18, AP_MotorsHeli, flybar_mode, 0),
	
	// @Param: STAB_COL_MIN
    // @DisplayName: Stabilize Throttle Minimum
    // @Description: This is the minimum collective setpoint in Stabilize Mode
    // @Range: 0 50
    // @Units: 1%
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_MIN", 19, AP_MotorsHeli, stab_col_min, 0),
	
	// @Param: STAB_COL_MAX
    // @DisplayName: Stabilize Throttle Maximum
    // @Description: This is the maximum collective setpoint in Stabilize Mode
    // @Range: 50 100
    // @Units: 1%
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_MAX", 20, AP_MotorsHeli, stab_col_max, 100),

    AP_GROUPEND
};

// init
void AP_MotorsHeli::Init()
{
    // set update rate
    set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsHeli::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    _rc->SetFastOutputChannels(_BV(_motor_to_channel_map[AP_MOTORS_MOT_1]) | _BV(_motor_to_channel_map[AP_MOTORS_MOT_2]) | _BV(_motor_to_channel_map[AP_MOTORS_MOT_3]) | _BV(_motor_to_channel_map[AP_MOTORS_MOT_4]), _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsHeli::enable()
{
    // enable output channels
    _rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_1]);            // swash servo 1
    _rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_2]);            // swash servo 2
    _rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_3]);            // swash servo 3
    _rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_4]);            // yaw
    _rc->enable_out(AP_MOTORS_HELI_EXT_GYRO);           // for external gyro
    _rc->enable_out(AP_MOTORS_HELI_EXT_RSC);            // for external RSC
}

// output_min - sends minimum values out to the motors
void AP_MotorsHeli::output_min()
{
    // move swash to mid
    move_swash(0,0,500,0);
}

// output_armed - sends commands to the motors
void AP_MotorsHeli::output_armed()
{
    // if manual override (i.e. when setting up swash), pass pilot commands straight through to swash
    if( servo_manual == 1 ) {
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
    if(_rc_throttle->control_in > 0) {
        // we have pushed up the throttle
        // remove safety
        _auto_armed = true;
    }

    // for helis - armed or disarmed we allow servos to move
    output_armed();
}

// output_disarmed - sends commands to the motors
void AP_MotorsHeli::output_test()
{
    int16_t i;
    // Send minimum values to all motors
    output_min();

    // servo 1
    for( i=0; i<5; i++ ) {
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo_1->radio_trim + 100);
        delay(300);
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo_1->radio_trim - 100);
        delay(300);
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo_1->radio_trim + 0);
        delay(300);
    }

    // servo 2
    for( i=0; i<5; i++ ) {
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo_2->radio_trim + 100);
        delay(300);
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo_2->radio_trim - 100);
        delay(300);
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo_2->radio_trim + 0);
        delay(300);
    }

    // servo 3
    for( i=0; i<5; i++ ) {
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo_3->radio_trim + 100);
        delay(300);
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo_3->radio_trim - 100);
        delay(300);
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo_3->radio_trim + 0);
        delay(300);
    }

    // external gyro
    if( ext_gyro_enabled ) {
        _rc->OutputCh(AP_MOTORS_HELI_EXT_GYRO, ext_gyro_gain);
    }

    // servo 4
    for( i=0; i<5; i++ ) {
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo_4->radio_trim + 100);
        delay(300);
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo_4->radio_trim - 100);
        delay(300);
        _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo_4->radio_trim + 0);
        delay(300);
    }

    // Send minimum values to all motors
    output_min();
}

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

    if( swash_type == AP_MOTORS_HELI_SWASH_CCPM ) {                     //CCPM Swashplate, perform servo control mixing

        // roll factors
        _rollFactor[CH_1] = cos(radians(servo1_pos + 90 - phase_angle));
        _rollFactor[CH_2] = cos(radians(servo2_pos + 90 - phase_angle));
        _rollFactor[CH_3] = cos(radians(servo3_pos + 90 - phase_angle));

        // pitch factors
        _pitchFactor[CH_1] = cos(radians(servo1_pos - phase_angle));
        _pitchFactor[CH_2] = cos(radians(servo2_pos - phase_angle));
        _pitchFactor[CH_3] = cos(radians(servo3_pos - phase_angle));

        // collective factors
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;

    }else{                                                                      //H1 Swashplate, keep servo outputs seperated

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

    // set roll, pitch and throttle scaling
    _roll_scaler = 1.0;
    _pitch_scaler = 1.0;
    _collective_scalar = ((float)(_rc_throttle->radio_max - _rc_throttle->radio_min))/1000.0;
	_stab_throttle_scalar = 1.0;

    // we must be in set-up mode so mark swash as uninitialised
    _swash_initialised = false;
}

// init_swash - initialise the swash plate
void AP_MotorsHeli::init_swash()
{

    // swash servo initialisation
    _servo_1->set_range(0,1000);
    _servo_2->set_range(0,1000);
    _servo_3->set_range(0,1000);
    _servo_4->set_angle(4500);

    // ensure _coll values are reasonable
    if( collective_min >= collective_max ) {
        collective_min = 1000;
        collective_max = 2000;
    }
    collective_mid = constrain(collective_mid, collective_min, collective_max);

    // calculate throttle mid point
    throttle_mid = ((float)(collective_mid-collective_min))/((float)(collective_max-collective_min))*1000.0;

    // determine roll, pitch and throttle scaling
    _roll_scaler = (float)roll_max/4500.0;
    _pitch_scaler = (float)pitch_max/4500.0;
    _collective_scalar = ((float)(collective_max-collective_min))/1000.0;
	_stab_throttle_scalar = ((float)(stab_col_max - stab_col_min))/100.0;

    if( swash_type == AP_MOTORS_HELI_SWASH_CCPM ) {                     //CCPM Swashplate, perform control mixing

        // roll factors
        _rollFactor[CH_1] = cos(radians(servo1_pos + 90 - phase_angle));
        _rollFactor[CH_2] = cos(radians(servo2_pos + 90 - phase_angle));
        _rollFactor[CH_3] = cos(radians(servo3_pos + 90 - phase_angle));

        // pitch factors
        _pitchFactor[CH_1] = cos(radians(servo1_pos - phase_angle));
        _pitchFactor[CH_2] = cos(radians(servo2_pos - phase_angle));
        _pitchFactor[CH_3] = cos(radians(servo3_pos - phase_angle));

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

    // servo min/max values
    _servo_1->radio_min = 1000;
    _servo_1->radio_max = 2000;
    _servo_2->radio_min = 1000;
    _servo_2->radio_max = 2000;
    _servo_3->radio_min = 1000;
    _servo_3->radio_max = 2000;

    // mark swash as initialised
    _swash_initialised = true;
}

//
// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 0 ~ 1000
//                       yaw:   -4500 ~ 4500
//
void AP_MotorsHeli::move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_out, int16_t yaw_out)
{
    int16_t yaw_offset = 0;
    int16_t coll_out_scaled;

    if( servo_manual == 1 ) {      // are we in manual servo mode? (i.e. swash set-up mode)?
        // check if we need to free up the swash
        if( _swash_initialised ) {
            reset_swash();
        }
        coll_out_scaled = coll_out * _collective_scalar + _rc_throttle->radio_min - 1000;
    }else{      // regular flight mode

        // check if we need to reinitialise the swash
        if( !_swash_initialised ) {
            init_swash();
        }

        // rescale roll_out and pitch-out into the min and max ranges to provide linear motion
        // across the input range instead of stopping when the input hits the constrain value
        // these calculations are based on an assumption of the user specified roll_max and pitch_max
        // coming into this equation at 4500 or less, and based on the original assumption of the
        // total _servo_x.servo_out range being -4500 to 4500.
        roll_out = roll_out * _roll_scaler;
        roll_out = constrain(roll_out, (int16_t)-roll_max, (int16_t)roll_max);

        pitch_out = pitch_out * _pitch_scaler;
        pitch_out = constrain(pitch_out, (int16_t)-pitch_max, (int16_t)pitch_max);

        // scale collective pitch
        coll_out = constrain(coll_out, 0, 1000);
		if (stab_throttle){
			coll_out = coll_out * _stab_throttle_scalar + stab_col_min*10;
		}
        coll_out_scaled = coll_out * _collective_scalar + collective_min - 1000;
		
        // rudder feed forward based on collective
        if( !ext_gyro_enabled ) {
            yaw_offset = collective_yaw_effect * abs(coll_out_scaled - throttle_mid);
        }
    }

    // swashplate servos
    _servo_1->servo_out = (_rollFactor[CH_1] * roll_out + _pitchFactor[CH_1] * pitch_out)/10 + _collectiveFactor[CH_1] * coll_out_scaled + (_servo_1->radio_trim-1500);
    _servo_2->servo_out = (_rollFactor[CH_2] * roll_out + _pitchFactor[CH_2] * pitch_out)/10 + _collectiveFactor[CH_2] * coll_out_scaled + (_servo_2->radio_trim-1500);
    if( swash_type == AP_MOTORS_HELI_SWASH_H1 ) {
        _servo_1->servo_out += 500;
        _servo_2->servo_out += 500;
    }
    _servo_3->servo_out = (_rollFactor[CH_3] * roll_out + _pitchFactor[CH_3] * pitch_out)/10 + _collectiveFactor[CH_3] * coll_out_scaled + (_servo_3->radio_trim-1500);
    _servo_4->servo_out = yaw_out + yaw_offset;

    // use servo_out to calculate pwm_out and radio_out
    _servo_1->calc_pwm();
    _servo_2->calc_pwm();
    _servo_3->calc_pwm();
    _servo_4->calc_pwm();

    // actually move the servos
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_1], _servo_1->radio_out);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_2], _servo_2->radio_out);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_3], _servo_3->radio_out);
    _rc->OutputCh(_motor_to_channel_map[AP_MOTORS_MOT_4], _servo_4->radio_out);

    // to be compatible with other frame types
    motor_out[AP_MOTORS_MOT_1] = _servo_1->radio_out;
    motor_out[AP_MOTORS_MOT_2] = _servo_2->radio_out;
    motor_out[AP_MOTORS_MOT_3] = _servo_3->radio_out;
    motor_out[AP_MOTORS_MOT_4] = _servo_4->radio_out;

    // output gyro value
    if( ext_gyro_enabled ) {
        _rc->OutputCh(AP_MOTORS_HELI_EXT_GYRO, ext_gyro_gain);
    }
}

void AP_MotorsHeli::rsc_control()

{
    switch ( rsc_mode ) {

    case AP_MOTORSHELI_RSC_MODE_CH8_PASSTHROUGH:
        if( armed() && _rc_8->control_in > 10 ) {
            if (rsc_ramp < rsc_ramp_up_rate) {
                rsc_ramp++;
                rsc_output = map(rsc_ramp, 0, rsc_ramp_up_rate, 1000, _rc_8->control_in);
            } else {
                rsc_output = _rc_8->control_in;
            }
        } else if( !armed() ) {
            _rc->OutputCh(AP_MOTORS_HELI_EXT_RSC, _rc_8->radio_min);
            rsc_ramp = 0;                       //Return RSC Ramp to 0
        }
        break;

    case AP_MOTORSHELI_RSC_MODE_EXT_GOV:

        if( armed() && _rc_throttle->control_in > 10) {
            if (rsc_ramp < rsc_ramp_up_rate) {
                rsc_ramp++;
                rsc_output = map(rsc_ramp, 0, rsc_ramp_up_rate, 1000, ext_gov_setpoint);
            } else {
                rsc_output = ext_gov_setpoint;
            }
        } else {
            rsc_ramp--;                                                 //Return RSC Ramp to 0 slowly, allowing for "warm restart"
            if (rsc_ramp < 0) {
                rsc_ramp = 0;
            }
            rsc_output = 1000;                                  //Just to be sure RSC output is 0
        }
        _rc->OutputCh(AP_MOTORS_HELI_EXT_RSC, rsc_output);
        break;

    //	case 3:																		// Open Loop ESC Control
    //
    //	coll_scaled = _motors->coll_out_scaled + 1000;
    //	if(coll_scaled <= _motors->collective_mid){
    //		esc_ol_output = map(coll_scaled, _motors->collective_min, _motors->collective_mid, esc_out_low, esc_out_mid);		// Bottom half of V-curve
    //	} else if (coll_scaled > _motors->collective_mid){
    //		esc_ol_output = map(coll_scaled, _motors->collective_mid, _motors->collective_max, esc_out_mid, esc_out_high);		// Top half of V-curve
    //	} else { esc_ol_output = 1000; }																									// Just in case.
    //
    //	if(_motors->armed() && _rc_throttle->control_in > 10){
    //			if (ext_esc_ramp < ext_esc_ramp_up){
    //				ext_esc_ramp++;
    //				ext_esc_output = map(ext_esc_ramp, 0, ext_esc_ramp_up, 1000, esc_ol_output);
    //			} else {
    //				ext_esc_output = esc_ol_output;
    //			}
    //		} else {
    //			ext_esc_ramp = 0;	//Return ESC Ramp to 0
    //			ext_esc_output = 1000; //Just to be sure ESC output is 0
    //}
    //		_rc->OutputCh(AP_MOTORS_HELI_EXT_ESC, ext_esc_output);
    //	break;


    default:
        break;
    }
};
