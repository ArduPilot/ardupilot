#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <DataFlash/DataFlash.h>
#include <cmath>

class AP_YawController {
public:                      
	AP_YawController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms) :
		aparm(parms),
        _ahrs(ahrs)
	{
		AP_Param::setup_object_defaults(this, var_info);
		_pid_info.desired = 0;
		_pid_info.FF = 0;
		_pid_info.P = 0;
	}

	int32_t get_servo_out(float scaler, bool disable_integrator);

	void reset_I();

	const DataFlash_Class::PID_Info& get_pid_info(void) const {return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

private:
	const AP_Vehicle::FixedWing &aparm;
	AP_Float _K_A;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Float _K_FF;
    AP_Int16 _imax;
	uint32_t _last_t;
	float _last_out;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _K_D_last;

	float _integrator;

	DataFlash_Class::PID_Info _pid_info;

	AP_AHRS &_ahrs;
};
