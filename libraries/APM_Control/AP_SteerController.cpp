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

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_SteerController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_SteerController::var_info[] = {
	// @Param: TCONST
	// @DisplayName: Steering Time Constant
	// @Description: This controls the time constant in seconds from demanded to achieved steering angle. A value of 0.75 is a good default and will work with nearly all rovers. Ground steering in aircraft needs a bit smaller time constant, and a value of 0.5 is recommended for best ground handling in fixed wing aircraft. A value of 0.75 means that the controller will try to correct any deviation between the desired and actual steering angle in 0.75 seconds. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the vehicle can achieve.
	// @Range: 0.4 1.0
	// @Units: s
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_SteerController, _tau,       0.75f),

	// @Param: P
	// @DisplayName: Steering turning gain
	// @Description: The proportional gain for steering. This should be approximately equal to the diameter of the turning circle of the vehicle at low speed and maximum steering angle
	// @Range: 0.1 10.0
	// @Increment: 0.1
	// @User: Standard
	AP_GROUPINFO("P",      1, AP_SteerController, _K_P,        1.8f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: This is the gain from the integral of steering angle. Increasing this gain causes the controller to trim out steady offsets due to an out of trim vehicle.
	// @Range: 0 1.0
	// @Increment: 0.05
	// @User: Standard
	AP_GROUPINFO("I",        3, AP_SteerController, _K_I,        0.2f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: This adjusts the damping of the steering control loop. This gain helps to reduce steering jitter with vibration. It should be increased in 0.01 increments as too high a value can lead to a high frequency steering oscillation that could overstress the vehicle.
	// @Range: 0 0.1
	// @Increment: 0.01
	// @User: Standard
	AP_GROUPINFO("D",        4, AP_SteerController, _K_D,        0.005f),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: This limits the number of degrees of steering in centi-degrees over which the integrator will operate. At the default setting of 1500 centi-degrees, the integrator will be limited to +- 15 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 1/3rd of the total control throw which is adequate unless the vehicle is severely out of trim.
	// @Range: 0 4500
	// @Increment: 10
	// @Units: cdeg
	// @User: Advanced
	AP_GROUPINFO("IMAX",     5, AP_SteerController, _imax,        1500),

	// @Param: MINSPD
	// @DisplayName: Minimum speed
	// @Description: This is the minimum assumed ground speed in meters/second for steering. Having a minimum speed prevents oscillations when the vehicle first starts moving. The vehicle can still drive slower than this limit, but the steering calculations will be done based on this minimum speed.
	// @Range: 0 5
	// @Increment: 0.1
	// @Units: m/s
	// @User: Standard
	AP_GROUPINFO("MINSPD",   6, AP_SteerController, _minspeed,    1.0f),


	// @Param: FF
	// @DisplayName: Steering feed forward
	// @Description: The feed forward gain for steering this is the ratio of the achieved turn rate to applied steering. A value of 1 means that the vehicle would yaw at a rate of 45 degrees per second with full steering deflection at 1m/s ground speed.
	// @Range: 0.0 10.0
	// @Increment: 0.1
	// @User: Standard
	AP_GROUPINFO("FF",      7, AP_SteerController, _K_FF,        0),


    // @Param: DRTSPD
    // @DisplayName: Derating speed
    // @Description: Speed after that the maximum degree of steering will start to derate. Set this speed to a maximum speed that a plane can do controlled turn at maximum angle of steering wheel without rolling to wing. If 0 then no derating is used.
    // @Range: 0.0 30.0
    // @Increment: 0.1
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("DRTSPD",  8, AP_SteerController, _deratespeed,        0),

    // @Param: DRTFCT
    // @DisplayName: Derating factor
    // @Description: Degrees of steering wheel to derate at each additional m/s of speed above "Derating speed". Should be set so that at higher speeds the plane does not roll to the wing in turns.
    // @Range: 0.0 50.0
    // @Increment: 0.1
    // @Units: deg/m/s
    // @User: Advanced
    AP_GROUPINFO("DRTFCT", 9, AP_SteerController, _deratefactor,        10),

    // @Param: DRTMIN
    // @DisplayName: Minimum angle of wheel
    // @Description: The angle that limits smallest angle of steering wheel at maximum speed. Even if it should derate below, it will stop derating at this angle.
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced
    AP_GROUPINFO("DRTMIN", 10, AP_SteerController, _mindegree,        4500),

	AP_GROUPEND
};


/*
  steering rate controller. Returns servo out -4500 to 4500 given
  desired yaw rate in degrees/sec. Positive yaw rate means clockwise yaw.
*/
int32_t AP_SteerController::get_steering_out_rate(float desired_rate)
{
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;

    float speed = _ahrs.groundspeed();
    if (speed < _minspeed) {
        // assume a minimum speed. This stops oscillations when first starting to move
        speed = _minspeed;
    }

    // this is a linear approximation of the inverse steering
    // equation for a ground vehicle. It returns steering as an angle from -45 to 45
    float scaler = 1.0f / speed;

    _pid_info.target = desired_rate;

	// Calculate the steering rate error (deg/sec) and apply gain scaler
    // We do this in earth frame to allow for rover leaning over in hard corners
    float yaw_rate_earth = ToDeg(_ahrs.get_yaw_rate_earth());
    if (_reverse) {
        yaw_rate_earth *= -1.0f;
    }
    _pid_info.actual = yaw_rate_earth;

    float rate_error = (desired_rate - yaw_rate_earth) * scaler;
	
	// Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    // No conversion is required for K_D
	float ki_rate = _K_I * _tau * 45.0f;
	float kp_ff = MAX((_K_P - _K_I * _tau) * _tau  - _K_D , 0) * 45.0f;
	float k_ff = _K_FF * 45.0f;
	float delta_time    = (float)dt * 0.001f;
	
	// Multiply yaw rate error by _ki_rate and integrate
	// Don't integrate if in stabilize mode as the integrator will wind up against the pilots inputs
	if (ki_rate > 0 && speed >= _minspeed) {
		// only integrate if gain and time step are positive.
		if (dt > 0) {
		    float integrator_delta = rate_error * ki_rate * delta_time * scaler;
			// prevent the integrator from increasing if steering defln demand is above the upper limit
			if (_last_out < -45) {
                integrator_delta = MAX(integrator_delta , 0);
            } else if (_last_out > 45) {
                // prevent the integrator from decreasing if steering defln demand is below the lower limit
                integrator_delta = MIN(integrator_delta, 0);
            }
			_pid_info.I += integrator_delta;
		}
	} else {
		_pid_info.I = 0;
	}
	
    // Scale the integration limit
    float intLimScaled = _imax * 0.01f;

    // Constrain the integrator state
    _pid_info.I = constrain_float(_pid_info.I, -intLimScaled, intLimScaled);

    _pid_info.D = rate_error * _K_D * 4.0f; 
    _pid_info.P = (ToRad(desired_rate) * kp_ff) * scaler;
    _pid_info.FF = (ToRad(desired_rate) * k_ff) * scaler;
	
    // Calculate the demanded control surface deflection
    _last_out = _pid_info.D + _pid_info.FF + _pid_info.P + _pid_info.I;
	
    float derate_constraint = 4500;

    // Calculate required constrain based on speed
    if (!is_zero(_deratespeed) && speed > _deratespeed) {
        derate_constraint = 4500 - (speed - _deratespeed) * _deratefactor * 100;
        if (derate_constraint < _mindegree) {
            derate_constraint = _mindegree;
        }
    }

    // Convert to centi-degrees and constrain
    return constrain_float(_last_out * 100, -derate_constraint, derate_constraint);
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
    if (_reverse) {
        desired_rate *= -1;
    }
    return get_steering_out_rate(desired_rate);
}

/*
  return a steering servo value from -4500 to 4500 given an angular
  steering error in centidegrees.
*/
int32_t AP_SteerController::get_steering_out_angle_error(int32_t angle_err)
{
    if (_tau < 0.1f) {
        _tau = 0.1f;
    }
	
	// Calculate the desired steering rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f / _tau;

    return get_steering_out_rate(desired_rate);
}

void AP_SteerController::reset_I()
{
	_pid_info.I = 0;
}

