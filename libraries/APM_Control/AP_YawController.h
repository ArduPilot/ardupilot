// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef __AP_YAW_CONTROLLER_H__
#define __AP_YAW_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <math.h> // for fabs()

class AP_YawController {
public:                      
	AP_YawController()
	{
		AP_Param::setup_object_defaults(this, var_info);
	}

	void set_ahrs(AP_AHRS *ahrs) { 
		_ahrs = ahrs; 
		_ins = _ahrs->get_ins();
	}

	int32_t get_servo_out(float scaler = 1.0, bool stabilize = false, int16_t aspd_min = 0, int16_t aspd_max = 0);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	AP_Float _K_A;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Float _K_FF;
	uint32_t _last_t;
	float _last_error;
	float _last_out;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _K_D_last;

	float _integrator;

	AP_AHRS *_ahrs;
	AP_InertialSensor *_ins;

};

#endif // __AP_YAW_CONTROLLER_H__
