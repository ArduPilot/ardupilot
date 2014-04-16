// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

//	Code by Andrew Tridgell
//  Based upon the roll controller by Paul Riseborough and Jon Challinger
//

#include <AP_Math.h>
#include <AP_HAL.h>
#include "AP_SteerController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_SteerController::var_info[] PROGMEM = {
	// @Param: TCONST
	// @DisplayName: Steering Time Constant
	// @Description: This controls the time constant in seconds from demanded to achieved steering angle. A value of 0.75 is a good default and will work with nearly all rovers. Ground steering in aircraft needs a bit smaller time constant, and a value of 0.5 is recommended for best ground handling in fixed wing aircraft. A value of 0.75 means that the controller will try to correct any deviation between the desired and actual steering angle in 0.75 seconds. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the vehicle can achieve.
	// @Range: 0.4 1.0
	// @Units: seconds
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_SteerController, _tau,       0.75f),

    // TODO hide completely
	// @Param: DPRCTD
	// @DisplayName: Steering turning gain
	// @Description: WARNING: DEPRECATED. If this parameter is set, it will be used to recompute FF and reset to -1.
	// @Range: 0.1 10.0
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("DPRCTD",      1, AP_SteerController, _K_P,        -1.0f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: This is the gain from the integral of steering angle. Increasing this gain causes the controller to trim out steady offsets due to an out of trim vehicle.
	// @Range: 0 1.0
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("I",        3, AP_SteerController, _K_I,        0.2f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: This adjusts the damping of the steering control loop. This gain helps to reduce steering jitter with vibration. It should be increased in 0.01 increments as too high a value can lead to a high frequency steering oscillation that could overstress the vehicle.
	// @Range: 0 0.1
	// @Increment: 0.01
	// @User: User
	AP_GROUPINFO("D",        4, AP_SteerController, _K_D,        0.005f),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: This limits the number of degrees of steering in centi-degrees over which the integrator will operate. At the default setting of 1500 centi-degrees, the integrator will be limited to +- 15 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 1/3rd of the total control throw which is adequate unless the vehicle is severely out of trim.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",     5, AP_SteerController, _imax,        1500),

	// @Param: MINSPD
	// @DisplayName: Minimum speed
	// @Description: This is the minimum assumed ground speed in meters/second for steering. Having a minimum speed prevents oscillations when the vehicle first starts moving. The vehicle can still driver slower than this limit, but the steering calculations will be done based on this minimum speed.
	// @Range: 0 5
	// @Increment: 0.1
    // @Units: m/s
	// @User: User
	AP_GROUPINFO("MINSPD",   6, AP_SteerController, _minspeed,    1.0f),


    // @Param: FF
    // @DisplayName: Feedforward Gain
    // @Description: The feedforward gain from desired rate to servo output. This should be approximately equal to the diameter of the turning circle of the vehicle at low speed and maximum servo deflection.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: User
    AP_GROUPINFO("FF",      7, AP_SteerController, _K_FF,        1.8f),

    // @Param: ACCMAX
    // @DisplayName: Acceleration limit
    // @Description: Angular acceleration limit in degrees per second
    // @Range: 90 540
    // @Increment: 0.1
    // @User: User
	AP_GROUPINFO("ACCMAX", 8, AP_SteerController, _acc_max, 360),

	AP_GROUPEND
};


/*
  steering rate controller. Returns servo out -4500 to 4500 given
  desired yaw rate in degrees/sec. Positive yaw rate means clockwise yaw.
*/
int32_t AP_SteerController::get_steering_out_rate(float desired_rate)
{
	uint32_t tnow = hal.scheduler->millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;

    float speed = _ahrs.groundspeed();
    if (speed < _minspeed) {
        // assume a minimum speed. This stops osciallations when first starting to move
        speed = _minspeed;
    }
    float delta_time    = (float)dt * 0.001f;
    //limit acceleration:
    float max_rate_dem_change_degs = _acc_max * delta_time;
    static float last_desired_rate = NAN;
    if(!isnan(last_desired_rate)) {
        desired_rate = constrain_float(desired_rate, last_desired_rate-max_rate_dem_change_degs,last_desired_rate+max_rate_dem_change_degs);
    }
    last_desired_rate = desired_rate;

    // this is a linear approximation of the inverse steering
    // equation for a ground vehicle. It returns steering as an angle from -45 to 45
    float scaler = 1.0f / speed;

	// Calculate the steering rate error (deg/sec) and apply gain scaler
	float rate_error = (desired_rate - ToDeg(_ahrs.get_gyro().z)) * scaler;

	// Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    // No conversion is required for K_D
	float ki_rate = _K_I * _tau * 45.0f;

    if(_K_P != -1.0f) {
        // compute _K_FF from _K_P
        _K_FF.set_and_save(max((_K_P - _K_I * _tau) * _tau - _K_D , 0.0f));
        // reset _K_P to -1.0f
        _K_P.set_and_save(-1.0f);
    }
	
	// Multiply roll rate error by _ki_rate and integrate
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	if (ki_rate > 0 && speed >= _minspeed) {
		// only integrate if gain and time step are positive.
		if (dt > 0) {
		    float integrator_delta = rate_error * ki_rate * delta_time;
			// prevent the integrator from increasing if steering defln demand is above the upper limit
			if (_last_out < -45) {
                integrator_delta = max(integrator_delta , 0);
            } else if (_last_out > 45) {
                // prevent the integrator from decreasing if steering defln demand is below the lower limit
                integrator_delta = min(integrator_delta, 0);
            }
			_integrator += integrator_delta;
		}
	} else if (ki_rate == 0) {
        _integrator = 0;
    }
	
    // Scale the integration limit
    float intLimScaled = _imax * 0.01f;

    // Constrain the integrator state
    _integrator = constrain_float(_integrator, -intLimScaled, intLimScaled);

	// Calculate the demanded control surface deflection
	_last_out = (rate_error * _K_D * 45.0f) + (ToRad(desired_rate) * _K_FF * 45.0f) * scaler + _integrator;
	

	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}


/*
  lateral acceleration controller. Returns servo value -4500 to 4500
  given a desired lateral acceleration
*/
int32_t AP_SteerController::get_steering_out_lat_accel(float desired_accel)
{
    float speed = _ahrs.groundspeed();
    if (speed < _minspeed) {
        // assume a minimum speed. This reduces osciallations when first starting to move
        speed = _minspeed;
    }

	// Calculate the desired steering rate given desired_accel and speed
    float desired_rate = ToDeg(desired_accel / speed);
    return get_steering_out_rate(desired_rate);
}

/*
  return a steering servo value from -4500 to 4500 given an angular
  steering error in centidegrees.
*/
int32_t AP_SteerController::get_steering_out_angle_error(int32_t angle_err_cd)
{
    if (_tau < 0.1) {
        _tau = 0.1;
    }

    float angle_err = angle_err_cd *0.01;
    float desired_rate;
    float omega = 1.0f/_tau;

    if(_acc_max == 0) {
        desired_rate = angle_err * omega;
    } else {
        float linear_angle = _acc_max/(omega*omega);
        if(angle_err > linear_angle) {
            desired_rate = safe_sqrt(2.0f*_acc_max*(fabs(angle_err)-(linear_angle/2.0f)));
        } else if (angle_err < -linear_angle) {
            desired_rate = -safe_sqrt(2.0f*_acc_max*(fabs(angle_err)-(linear_angle/2.0f)));
        } else {
            desired_rate = angle_err/_tau;
        }
    }

    return get_steering_out_rate(desired_rate);
}

//return the stopping distance of the controller in radians
float AP_SteerController::get_stopping_angle() {
    float ang_vel = _ahrs.get_gyro().z;
    float acc_max_rad_ss = ToRad((float)_acc_max);
    if(fabs(ang_vel) < acc_max_rad_ss*_tau) {
        return _tau*ang_vel;
    } else {
        float stopping_angle = acc_max_rad_ss*_tau*0.5f + 0.5f*ang_vel*ang_vel/acc_max_rad_ss;
        if(ang_vel < 0) {
            return -stopping_angle;
        } else {
            return stopping_angle;
        }
    }
}

void AP_SteerController::reset_I()
{
	_integrator = 0;
}

