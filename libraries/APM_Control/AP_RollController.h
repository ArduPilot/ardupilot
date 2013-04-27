// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef __AP_ROLL_CONTROLLER_H__
#define __AP_ROLL_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <math.h> // for fabs()

class AP_RollController {
public:
	AP_RollController() { 
		AP_Param::setup_object_defaults(this, var_info);
	}

	void set_ahrs(AP_AHRS *ahrs) { _ahrs = ahrs; }

	int32_t get_servo_out(int32_t angle, float scaler=1.0, bool stabilize=false, int16_t aspd_min = 0);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	AP_Float _kp_angle;
	AP_Float _kp_ff;
	AP_Float _kp_rate;
	AP_Float _ki_rate;
	AP_Int16 _max_rate;
	uint32_t _last_t;
	float _last_out;

	float _integrator;

	AP_AHRS *_ahrs;

};

#endif // __AP_ROLL_CONTROLLER_H__
