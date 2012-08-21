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

// below is required to make "map" function available to this library
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define AP_MOTORS_HELI_SPEED_DEFAULT 125	// default servo update rate for helicopters
#define AP_MOTORS_HELI_SPEED_DIGITAL_SERVOS 125	// update rate for digital servos
#define AP_MOTORS_HELI_SPEED_ANALOG_SERVOS 125	// update rate for analog servos

#define AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS 3

// tail servo uses channel 7
#define AP_MOTORS_HELI_EXT_GYRO	CH_7

// frame definitions
#define AP_MOTORS_HELI_SWASH_CCPM	0
#define AP_MOTORS_HELI_SWASH_H1		1

// ext RSC definitions
#define AP_MOTORS_HELI_EXT_RSC CH_8
#define AP_MOTORSHELI_RSC_MODE_CH8_PASSTHROUGH  1
#define AP_MOTORSHELI_RSC_MODE_EXT_GOV			2


class AP_HeliControls;

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
					RC_Channel* rc_8,
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
		_rc_8(rc_8)
		{
			throttle_mid = 0;
			_roll_scaler = 1;
			_pitch_scaler = 1;
			_collective_scalar = 1;
			_swash_initialised = false;
		};

	RC_Channel	*_servo_1;
	RC_Channel	*_servo_2;
	RC_Channel	*_servo_3;
	RC_Channel  *_servo_4;
	RC_Channel  *_rc_8;	
	AP_Int8		swash_type;
	AP_Int16	servo1_pos;
	AP_Int16	servo2_pos;
	AP_Int16	servo3_pos;
	AP_Int16	collective_min;
	AP_Int16	collective_max;
	AP_Int16	collective_mid;
	AP_Int16    ext_gyro_enabled;
    AP_Int16    ext_gyro_gain;
	AP_Int16	roll_max;
	AP_Int16	pitch_max;
	AP_Int16	phase_angle;
	AP_Int16	collective_yaw_effect;
	AP_Int8		servo_manual;		// used to trigger swash reset from mission planner	
	AP_Int16	ext_gov_setpoint;	// maximum output to the motor governor
	AP_Int8		rsc_mode;			// sets the mode for rotor speed controller
	AP_Int16	rsc_ramp_up_rate;	// sets the time in 100th seconds the RSC takes to ramp up to speed
	AP_Int8		flybar_mode;		// selects FBL Acro Mode, or Flybarred Acro Mode
	int16_t 	throttle_mid;		// throttle mid point in pwm form (i.e. 0 ~ 1000)

	
	// init
	void Init();

	// set update rate to motors - a value in hertz or AP_MOTORS_SPEED_INSTANT_PWM for instant pwm
	// you must have setup_motors before calling this
	void set_update_rate( uint16_t speed_hz );

	// enable - starts allowing signals to be sent to motors
	void enable();

	// get basic information about the platform
	uint8_t get_num_motors() { return 5; };

	// motor test
	void output_test();
 
 	// output_min - sends minimum values out to the motors
	void output_min();

	// init_swash - initialise the swash plate
	void init_swash();

	// output - sends commands to the motors
	void output_armed();
	
	// var_info for holding Parameter information
	static const struct AP_Param::GroupInfo var_info[];

protected:
	
	// heli_move_swash - moves swash plate to attitude of parameters passed in
	void move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_out, int16_t yaw_out);
	
	// reset_swash - free up swash for maximum movements. Used for set-up
	void reset_swash();
	
	void output_disarmed();
	
	void rsc_control();
	
	float _rollFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
	float _pitchFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
	float _collectiveFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];

	
		
	// internally used variables
	
	float _roll_scaler;				// scaler to convert roll input from radio (i.e. -4500 ~ 4500) to max roll range
	float _pitch_scaler;			// scaler to convert pitch input from radio (i.e. -4500 ~ 4500) to max pitch range
	float _collective_scalar;		// throttle scalar to convert pwm form (i.e. 0 ~ 1000) passed in to actual servo range (i.e 1250~1750 would be 500)
	bool _swash_initialised;		// true if swash has been initialised
	int16_t 	rsc_output;			// final output to the external motor governor 1000-2000
	int16_t 	rsc_ramp;			// current state of ramping
	

}; 

#endif  // AP_MOTORSHELI
