// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_YAW_CONTROLLER_H__
#define __AP_YAW_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <AP_Vehicle.h>
#include <math.h>

class AP_YawController {
public:                      
	AP_YawController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms) :
		aparm(parms),
        _ahrs(ahrs)
	{
		AP_Param::setup_object_defaults(this, var_info);
	}

	int32_t get_servo_out(float scaler, bool disable_integrator);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	const AP_Vehicle::FixedWing &aparm;
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

	AP_AHRS &_ahrs;
};

#endif // __AP_YAW_CONTROLLER_H__
