/*
	AP_MotorsHeli.cpp - ArduCopter motors library
	Code by RandyMackay. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
*/

#include "AP_MotorsHeli.h"

const AP_Param::GroupInfo AP_MotorsHeli::var_info[] PROGMEM = {
	AP_NESTEDGROUPINFO(AP_Motors, 0),
	AP_GROUPINFO("SV1_POS",	1,	AP_MotorsHeli,	servo1_pos),
	AP_GROUPINFO("SV2_POS",	2,	AP_MotorsHeli,	servo2_pos),
	AP_GROUPINFO("SV3_POS",	3,	AP_MotorsHeli,	servo3_pos),
	AP_GROUPINFO("ROL_MAX",	4,	AP_MotorsHeli,	roll_max),
	AP_GROUPINFO("PIT_MAX",	5,	AP_MotorsHeli,	pitch_max),
	AP_GROUPINFO("COL_MIN",	6,	AP_MotorsHeli,	collective_min),
	AP_GROUPINFO("COL_MAX",	7,	AP_MotorsHeli,	collective_max),
	AP_GROUPINFO("COL_MID",	8,	AP_MotorsHeli,	collective_mid),
	AP_GROUPINFO("GYR_ENABLE",	9,	AP_MotorsHeli,	ext_gyro_enabled),
	AP_GROUPINFO("SWASH_TYPE",	10,	AP_MotorsHeli,	swash_type),				// changed from trunk
	AP_GROUPINFO("GYR_GAIN",	11,	AP_MotorsHeli,	ext_gyro_gain),
	AP_GROUPINFO("SV_MAN",		12,	AP_MotorsHeli,	servo_manual),
	AP_GROUPINFO("PHANG",		13,	AP_MotorsHeli,	phase_angle),				// changed from trunk
	AP_GROUPINFO("COLYAW",		14,	AP_MotorsHeli,	collective_yaw_effect),	// changed from trunk
    AP_GROUPEND
};

// init
void AP_MotorsHeli::Init()
{
	// set update rate
	set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz or AP_MOTORS_SPEED_INSTANT_PWM for instant pwm
void AP_MotorsHeli::set_update_rate( uint16_t speed_hz )
{
	// record requested speed
	_speed_hz = speed_hz;

	// setup fast channels
	if( _speed_hz != AP_MOTORS_SPEED_INSTANT_PWM ) {
		_rc->SetFastOutputChannels(_BV(_motor_to_channel_map[AP_MOTORS_MOT_1]) | _BV(_motor_to_channel_map[AP_MOTORS_MOT_2]) | _BV(_motor_to_channel_map[AP_MOTORS_MOT_3]) | _BV(_motor_to_channel_map[AP_MOTORS_MOT_4]), _speed_hz);
	}
}

	// enable - starts allowing signals to be sent to motors
void AP_MotorsHeli::enable()
{
	// enable output channels
	_rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_1]);	// swash servo 1
	_rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_2]);	// swash servo 2
	_rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_3]);	// swash servo 3
	_rc->enable_out(_motor_to_channel_map[AP_MOTORS_MOT_4]);	// yaw
	_rc->enable_out(AP_MOTORS_HELI_EXT_GYRO);	// for external gyro
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
}

// output_disarmed - sends commands to the motors
void AP_MotorsHeli::output_disarmed()
{
	if(_rc_throttle->control_in > 0){
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

	if( swash_type == AP_MOTORS_HELI_SWASH_CCPM ) {			//CCPM Swashplate, perform servo control mixing
		
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
		
	}else{  								//H1 Swashplate, keep servo outputs seperated

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

	if( swash_type == AP_MOTORS_HELI_SWASH_CCPM ) {			//CCPM Swashplate, perform control mixing
		
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
		
	}else{  	//H1 Swashplate, keep servo outputs seperated

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

	if( servo_manual == 1 ) {  // are we in manual servo mode? (i.e. swash set-up mode)?
		// check if we need to free up the swash
		if( _swash_initialised ) {
			reset_swash();
		}
		coll_out_scaled = coll_out * _collective_scalar + _rc_throttle->radio_min - 1000;
	}else{  // regular flight mode

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
		coll_out_scaled = coll_out * _collective_scalar + collective_min - 1000;

		// rudder feed forward based on collective
		if( !ext_gyro_enabled ) {
			yaw_offset = collective_yaw_effect * abs(coll_out_scaled - collective_mid);
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

	// InstantPWM
	if( _speed_hz == AP_MOTORS_SPEED_INSTANT_PWM ) {
		_rc->Force_Out0_Out1();
		_rc->Force_Out2_Out3();
	}
}