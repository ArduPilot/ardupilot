// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//	Code by Jon Challinger
//  Modified by Paul Riseborough to implement a three loop autopilot
//  topology
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

#include <AP_Math.h>
#include <AP_HAL.h>
#include "AP_YawController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_YawController::var_info[] PROGMEM = {

	// @Param: K_A
	// @DisplayName: Sideslip control gain
	// @Description: This is the gain from measured lateral acceleration to demanded yaw rate. It should be set to zero unless active control of sideslip is desired. This will only work effectively if there is enough fuselage side area to generate a measureable lateral acceleration when the model sideslips. Flying wings and most gliders cannot use this term. This term should only be adjusted after the basic yaw damper gain K_D is tuned and the K_I integrator gain has been set. Set this gain to zero if only yaw damping is required.
	// @Range: 0 4
	// @Increment: 0.25
	AP_GROUPINFO("K_A",    0, AP_YawController, _K_A,    0),

	// @Param: K_I
	// @DisplayName: Sidelsip control integrator
	// @Description: This is the integral gain from lateral acceleration error. This gain should only be non-zero if active control over sideslip is desired. If active control over sideslip is required then this can be set to 1.0 as a first try.
	// @Range: 0 2
	// @Increment: 0.25
	AP_GROUPINFO("K_I",    1, AP_YawController, _K_I,    0),

	// @Param: K_D
	// @DisplayName: Yaw damping
	// @Description: This is the gain from yaw rate to rudder. It acts as a damper on yaw motion. If a basic yaw damper is required, this gain term can be incremented, whilst leaving the K_A and K_I gains at zero.
	// @Range: 0 2
	// @Increment: 0.25
	AP_GROUPINFO("K_D",    2, AP_YawController, _K_D,    0),

	// @Param: K_RLL
	// @DisplayName: Yaw coordination gain
	// @Description: This is the gain term that is applied to the yaw rate offset calculated as required to keep the yaw rate consistent with the turn rate for a coordinated turn. The default value is 1 which will work for all models. Advanced users can use it to correct for any tendency to yaw away from or into the turn once the turn is established. Increase to make the model yaw more initially and decrease to make the model yaw less initially. If values greater than 1.1 or less than 0.9 are required then it normally indicates a problem with the airspeed calibration.
	// @Range: 0.8 1.2
	// @Increment: 0.05
	AP_GROUPINFO("K_RLL",  3, AP_YawController, _K_FF,   1),

	AP_GROUPEND
};

int32_t AP_YawController::get_servo_out(float scaler, bool stabilize, int16_t aspd_min, int16_t aspd_max)
{
	uint32_t tnow = hal.scheduler->millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	if(_ins == NULL) { // can't control without a reference
		return 0; 
	}
	
	float delta_time = (float) dt / 1000.0f;
	
	// Calculate yaw rate required to keep up with a constant height coordinated turn
	float aspeed;
	float rate_offset;
	float bank_angle = _ahrs->roll;
	// limit bank angle between +- 80 deg if right way up
	if (fabsf(bank_angle) < 1.5707964f)	{
	    bank_angle = constrain_float(bank_angle,-1.3962634f,1.3962634f);
	}
	if (!_ahrs->airspeed_estimate(&aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aspd_min) + float(aspd_max));
	}
    rate_offset = (GRAVITY_MSS / max(aspeed , float(aspd_min))) * tanf(bank_angle) * cosf(bank_angle) * _K_FF;

    // Get body rate vector (radians/sec)
	float omega_z = _ahrs->get_gyro().z;
	
	// Get the accln vector (m/s^2)
	float accel_y = _ins->get_accel().y;

	// Subtract the steady turn component of rate from the measured rate
	// to calculate the rate relative to the turn requirement in degrees/sec
	float rate_hp_in = ToDeg(omega_z - rate_offset);
	
	// Apply a high-pass filter to the rate to washout any steady state error
	// due to bias errors in rate_offset
	// Use a cut-off frequency of omega = 0.2 rad/sec
	// Could make this adjustable by replacing 0.9960080 with (1 - omega * dt)
	float rate_hp_out = 0.9960080f * _last_rate_hp_out + rate_hp_in - _last_rate_hp_in;
	_last_rate_hp_out = rate_hp_out;
	_last_rate_hp_in = rate_hp_in;

	//Calculate input to integrator
	float integ_in = - _K_I * (_K_A * accel_y + rate_hp_out);
	
	// Apply integrator, but clamp input to prevent control saturation and freeze integrator below min FBW speed
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	// Don't integrate if _K_D is zero as integrator will keep winding up
	if (!stabilize && _K_D > 0) {
		//only integrate if airspeed above min value
		if (aspeed > float(aspd_min))
		{
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) _integrator += max(integ_in * delta_time , 0);
			// prevent the integrator from decreasing if surface defln demand  is below the lower limit
			else if (_last_out > 45) _integrator += min(integ_in * delta_time , 0);
			else _integrator += integ_in * delta_time;
		}
	} else {
		_integrator = 0;
	}
	
	// Protect against increases to _K_D during in-flight tuning from creating large control transients
	// due to stored integrator values
	if (_K_D > _K_D_last && _K_D > 0) {
	    _integrator = _K_D_last/_K_D * _integrator;
	}
	_K_D_last = _K_D;
	
	// Calculate demanded rudder deflection, +Ve deflection yaws nose right
	// Save to last value before application of limiter so that integrator limiting
	// can detect exceedance next frame
	// Scale using inverse dynamic pressure (1/V^2)
	_last_out =  _K_D * (_integrator - rate_hp_out) * scaler * scaler;

	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}

void AP_YawController::reset_I()
{
	_integrator = 0;
}
