// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_STEER_CONTROLLER_H__
#define __AP_STEER_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <AP_Vehicle.h>
#include <math.h>

class AP_SteerController {
public:
	AP_SteerController(AP_AHRS &ahrs) :
        _ahrs(ahrs)
    { 
		AP_Param::setup_object_defaults(this, var_info);
	}

    /*
      return a steering servo output from -4500 to 4500 given a
      desired lateral acceleration rate in m/s/s.
     */
	int32_t get_steering_out(float desired_accel);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	AP_Float _tau;
	AP_Float _K_P;
	AP_Float _K_I;
	AP_Float _K_D;
    AP_Int16  _imax;
	uint32_t _last_t;
	float _last_out;

	float _integrator;

	AP_AHRS &_ahrs;
};

#endif // __AP_STEER_CONTROLLER_H__
