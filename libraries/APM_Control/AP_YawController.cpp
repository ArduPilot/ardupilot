/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Code by Jon Challinger
//  Modified by Paul Riseborough to implement a three loop autopilot
//  topology
//
#include <AP_HAL/AP_HAL.h>
#include "AP_YawController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_YawController::var_info[] = {

	// @Param: SLIP
	// @DisplayName: Sideslip control gain
	// @Description: Gain from lateral acceleration to demanded yaw rate for aircraft with enough fuselage area to detect lateral acceleration and sideslips. Do not enable for flying wings and gliders. Actively coordinates flight more than just yaw damping. Set after YAW2SRV_DAMP and YAW2SRV_INT are tuned.
	// @Range: 0 4
	// @Increment: 0.25
    // @User: Advanced
	AP_GROUPINFO("SLIP",    0, AP_YawController, _K_A,    0),

	// @Param: INT
	// @DisplayName: Sideslip control integrator
	// @Description: Integral gain from lateral acceleration error. Effectively trims rudder to eliminate long-term sideslip.
	// @Range: 0 2
	// @Increment: 0.25
    // @User: Advanced
	AP_GROUPINFO("INT",    1, AP_YawController, _K_I,    0),

	// @Param: DAMP
	// @DisplayName: Yaw damping
	// @Description: Gain from yaw rate to rudder. Most effective at yaw damping and should be tuned after KFF_RDDRMIX. Also disables YAW2SRV_INT if set to 0.
	// @Range: 0 2
	// @Increment: 0.25
    // @User: Advanced
	AP_GROUPINFO("DAMP",   2, AP_YawController, _K_D,    0),

	// @Param: RLL
	// @DisplayName: Yaw coordination gain
	// @Description: Gain to the yaw rate required to keep it consistent with the turn rate in a coordinated turn. Corrects for yaw tendencies after the turn is established. Increase yaw into the turn by raising. Increase yaw out of the turn by decreasing. Values outside of 0.9-1.1 range indicate airspeed calibration problems.
	// @Range: 0.8 1.2
	// @Increment: 0.05
    // @User: Advanced
	AP_GROUPINFO("RLL",   3, AP_YawController, _K_FF,   1),

    /*
      Note: index 4 should not be used - it was used for an incorrect
      AP_Int8 version of the IMAX in the 2.74 release
     */


	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of yaw integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 1500 allows trim of up to 1/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",  5, AP_YawController, _imax,        1500),

	AP_GROUPEND
};

int32_t AP_YawController::get_servo_out(float scaler, bool disable_integrator)
{
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	

    int16_t aspd_min = aparm.airspeed_min;
    if (aspd_min < 1) {
        aspd_min = 1;
    }
	
	float delta_time = (float) dt / 1000.0f;
	
	// Calculate yaw rate required to keep up with a constant height coordinated turn
	float aspeed;
	float rate_offset;
	float bank_angle = _ahrs.roll;
	// limit bank angle between +- 80 deg if right way up
	if (fabsf(bank_angle) < 1.5707964f)	{
	    bank_angle = constrain_float(bank_angle,-1.3962634f,1.3962634f);
	}
	if (!_ahrs.airspeed_estimate(aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aspd_min) + float(aparm.airspeed_max));
	}
    rate_offset = (GRAVITY_MSS / MAX(aspeed , float(aspd_min))) * sinf(bank_angle) * _K_FF;

    // Get body rate vector (radians/sec)
	float omega_z = _ahrs.get_gyro().z;
	
	// Get the accln vector (m/s^2)
	float accel_y = AP::ins().get_accel().y;

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
	if (!disable_integrator && _K_D > 0) {
		//only integrate if airspeed above min value
		if (aspeed > float(aspd_min))
		{
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) {
                _integrator += MAX(integ_in * delta_time , 0);
            } else if (_last_out > 45) {
                // prevent the integrator from decreasing if surface defln demand  is below the lower limit
                _integrator += MIN(integ_in * delta_time , 0);
			} else {
                _integrator += integ_in * delta_time;
            }
		}
	} else {
		_integrator = 0;
	}

    if (_K_D < 0.0001f) {
        // yaw damping is disabled, and the integrator is scaled by damping, so return 0
        return 0;
    }
	
    // Scale the integration limit
    float intLimScaled = _imax * 0.01f / (_K_D * scaler * scaler);

    // Constrain the integrator state
    _integrator = constrain_float(_integrator, -intLimScaled, intLimScaled);
	
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
	_pid_info.I = _K_D * _integrator * scaler * scaler;
	_pid_info.D = _K_D * (-rate_hp_out) * scaler * scaler;
	_last_out =  _pid_info.I + _pid_info.D;

	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}

void AP_YawController::reset_I()
{
	_integrator = 0;
}