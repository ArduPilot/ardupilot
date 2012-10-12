// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef __AP_YAW_CONTROLLER_H__
#define __AP_YAW_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <math.h> // for fabs()

class AP_YawController {
public:                      
	void set_ahrs(AP_AHRS *ahrs) { 
		_ahrs = ahrs; 
		_ins = _ahrs->get_ins();
	}

	int32_t get_servo_out(float scaler = 1.0, bool stick_movement = false);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	AP_Float _kp;
	AP_Float _ki;
	AP_Int16 _imax;
	uint32_t _last_t;
	float _last_error;

	float _integrator;
	bool _stick_movement;
	uint32_t _stick_movement_begin;
	uint32_t _freeze_start_time;

	AP_AHRS *_ahrs;
	AP_InertialSensor *_ins;

	// Low pass filter cut frequency for derivative calculation.
	// FCUT macro computes a frequency cut based on an acceptable delay.
	#define FCUT(d) (1 / ( 2 * 3.14 * (d) ) )
	static const float _fCut = FCUT(.5);
};

#endif // __AP_YAW_CONTROLLER_H__
