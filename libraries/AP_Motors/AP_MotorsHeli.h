// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_MotorsHeli.h
/// @brief	Motor control class for Traditional Heli

#ifndef AP_MOTORSHELI
#define AP_MOTORSHELI

#include <inttypes.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_Motors.h>

#define AP_MOTORS_HELI_SPEED_DEFAULT 125	// default servo update rate for helicopters
#define AP_MOTORS_HELI_SPEED_DIGITAL_SERVOS 125	// update rate for digital servos
#define AP_MOTORS_HELI_SPEED_ANALOG_SERVOS 125	// update rate for analog servos

#define AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS 3

// tail servo uses channel 7
#define AP_MOTORS_HELI_EXT_GYRO	CH_7

// frame definitions
#define AP_MOTORS_HELI_SWASH_CCPM	0
#define AP_MOTORS_HELI_SWASH_H1		1

/// @class      AP_MotorsHeli 
class AP_MotorsHeli : public AP_Motors {
public: 

	/// Constructor 
	AP_MotorsHeli( uint8_t APM_version, 
					APM_RC_Class* rc_out,
					RC_Channel* rc_roll,
					RC_Channel* rc_pitch,
					RC_Channel* rc_throttle,
					RC_Channel* rc_yaw,
					RC_Channel* swash_servo_1,
					RC_Channel* swash_servo_2,
					RC_Channel* swash_servo_3,
					RC_Channel* yaw_servo,
					uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) : 
		AP_Motors(APM_version, rc_out, rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
		_servo_1(swash_servo_1),
		_servo_2(swash_servo_2),
		_servo_3(swash_servo_3),
		_servo_4(yaw_servo),
		swash_type(AP_MOTORS_HELI_SWASH_CCPM),
		servo1_pos				(-60),
		servo2_pos				(60),
		servo3_pos				(180),
		roll_max				(4500),
		pitch_max				(4500),
		collective_min			(1250),
		collective_max			(1750),
		collective_mid			(1500),
		ext_gyro_enabled		(0),
		ext_gyro_gain			(1350),
		phase_angle				(0),
		collective_yaw_effect	(0),
		servo_manual			(0),
		throttle_mid(0),
		_roll_scaler(1),
		_pitch_scaler(1),
		_collective_scalar(1),
		_swash_initialised(false)
		{};

	// init
	virtual void Init();

	// set update rate to motors - a value in hertz or AP_MOTORS_SPEED_INSTANT_PWM for instant pwm
	// you must have setup_motors before calling this
	virtual void set_update_rate( uint16_t speed_hz );

	// enable - starts allowing signals to be sent to motors
	virtual void enable();

	// get basic information about the platform
	virtual uint8_t get_num_motors() { return 4; };

	// motor test
	virtual void output_test();
 
 	// output_min - sends minimum values out to the motors
	virtual void output_min();

	// reset_swash - free up swash for maximum movements. Used for set-up
	virtual void reset_swash();

	// init_swash - initialise the swash plate
	virtual void init_swash();

	// heli_move_swash - moves swash plate to attitude of parameters passed in
	virtual void move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_out, int16_t yaw_out);

	// var_info for holding Parameter information
	static const struct AP_Param::GroupInfo var_info[];

//protected:
	// output - sends commands to the motors
	virtual void output_armed();
	virtual void output_disarmed();
	
	float _rollFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
	float _pitchFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
	float _collectiveFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];

	RC_Channel	*_servo_1;
	RC_Channel	*_servo_2;
	RC_Channel	*_servo_3;
	RC_Channel	*_servo_4;
	AP_Int8		swash_type;
	AP_Int16	servo1_pos;
	AP_Int16	servo2_pos;
	AP_Int16	servo3_pos;
	AP_Int16	roll_max;
	AP_Int16	pitch_max;
	AP_Int16	collective_min;
	AP_Int16	collective_max;
	AP_Int16	collective_mid;
	AP_Int16	ext_gyro_enabled;
	AP_Int16	ext_gyro_gain;
	AP_Int16	phase_angle;
	AP_Int16	collective_yaw_effect;
	AP_Int8		servo_manual;		// used to trigger swash reset from mission planner
	
	// internally used variables
	int16_t throttle_mid;			// throttle mid point in pwm form (i.e. 0 ~ 1000)
	float _roll_scaler;				// scaler to convert roll input from radio (i.e. -4500 ~ 4500) to max roll range
	float _pitch_scaler;			// scaler to convert pitch input from radio (i.e. -4500 ~ 4500) to max pitch range
	float _collective_scalar;		// throttle scalar to convert pwm form (i.e. 0 ~ 1000) passed in to actual servo range (i.e 1250~1750 would be 500)
	bool _swash_initialised;		// true if swash has been initialised

}; 

#endif  // AP_MOTORSHELI