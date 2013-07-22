// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_YAW_CONTROLLER_H__
#define __AP_YAW_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <AP_SpdHgtControl.h>
#include <math.h>

class AP_YawController {
public:                      
	AP_YawController(const AP_SpdHgtControl::AircraftParameters &parms) :
		aparm(parms)
	{
		AP_Param::setup_object_defaults(this, var_info);
	}

	void set_ahrs(AP_AHRS *ahrs) { 
		_ahrs = ahrs; 
		_ins = _ahrs->get_ins();
	}

	int32_t get_servo_out(float scaler = 1.0, bool stabilize = false);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	const AP_SpdHgtControl::AircraftParameters &aparm;
	AP_Float _K_A;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Float _K_FF;
    AP_Int16 _imax;
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
