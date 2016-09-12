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

//	Initial Code by Jon Challinger
//  Modified by Paul Riseborough

#include <AP_HAL/AP_HAL.h>
#include "AP_PitchController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PitchController::var_info[] = {

	// @Param: TCONST
	// @DisplayName: Pitch Time Constant
	// @Description: This controls the time constant in seconds from demanded to achieved pitch angle. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.
	// @Range: 0.4 1.0
	// @Units: seconds
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_PitchController, gains.tau,       0.5f),

	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: This is the gain from pitch angle to elevator. This gain works the same way as PTCH2SRV_P in the old PID controller and can be set to the same value.
	// @Range: 0.1 3.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("P",        1, AP_PitchController, gains.P,          0.6f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: This is the gain from pitch rate to elevator. This adjusts the damping of the pitch control loop. It has the same effect as PTCH2SRV_D in the old PID controller and can be set to the same value, but without the spikes in servo demands. This gain helps to reduce pitching in turbulence. Some airframes such as flying wings that have poor pitch damping can benefit from increasing this gain term. This should be increased in 0.01 increments as too high a value can lead to a high frequency pitch oscillation that could overstress the airframe.
	// @Range: 0 0.1
	// @Increment: 0.01
	// @User: User
	AP_GROUPINFO("D",        2, AP_PitchController, gains.D,        0.02f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: This is the gain applied to the integral of pitch angle. It has the same effect as PTCH2SRV_I in the old PID controller and can be set to the same value. Increasing this gain causes the controller to trim out constant offsets between demanded and measured pitch angle.
	// @Range: 0 0.5
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("I",        3, AP_PitchController, gains.I,        0.15f),

	// @Param: RMAX_UP
	// @DisplayName: Pitch up max rate
	// @Description: This sets the maximum nose up pitch rate that the controller will demand (degrees/sec). Setting it to zero disables the limit.
	// @Range: 0 100
	// @Units: degrees/second
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX_UP",     4, AP_PitchController, gains.rmax,   0.0f),

	// @Param: RMAX_DN
	// @DisplayName: Pitch down max rate
	// @Description: This sets the maximum nose down pitch rate that the controller will demand (degrees/sec). Setting it to zero disables the limit.
	// @Range: 0 100
	// @Units: degrees/second
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX_DN",     5, AP_PitchController, _max_rate_neg,   0.0f),

	// @Param: RLL
	// @DisplayName: Roll compensation
	// @Description: This is the gain term that is applied to the pitch rate offset calculated as required to keep the nose level during turns. The default value is 1 which will work for all models. Advanced users can use it to correct for height variation in turns. If height is lost initially in turns this can be increased in small increments of 0.05 to compensate. If height is gained initially in turns then it can be decreased.
	// @Range: 0.7 1.5
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("RLL",      6, AP_PitchController, _roll_ff,        1.0f),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: This limits the number of centi-degrees of elevator over which the integrator will operate. At the default setting of 3000 centi-degrees, the integrator will be limited to +- 30 degrees of servo travel. The maximum servo deflection is +- 45 degrees, so the default value represents a 2/3rd of the total control throw which is adequate for most aircraft unless they are severely out of trim or have very limited elevator control effectiveness.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      7, AP_PitchController, gains.imax,     3000),

	// @Param: FF
	// @DisplayName: Feed forward Gain
	// @Description: This is the gain from demanded rate to elevator output. 
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("FF",        8, AP_PitchController, gains.FF,       0.0f),

	AP_GROUPEND
};

/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded pitch rate in degrees/second
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
 5) maximum FBW airspeed (metres/sec)
*/
int32_t AP_PitchController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed)
{
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	float delta_time    = (float)dt * 0.001f;
	
	// Get body rate vector (radians/sec)
	float omega_y = _ahrs.get_gyro().y;
	
	// Calculate the pitch rate error (deg/sec) and scale
    float achieved_rate = ToDeg(omega_y);
	float rate_error = (desired_rate - achieved_rate) * scaler;
	
	// Multiply pitch rate error by _ki_rate and integrate
	// Scaler is applied before integrator so that integrator state relates directly to elevator deflection
	// This means elevator trim offset doesn't change as the value of scaler changes with airspeed
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	if (!disable_integrator && gains.I > 0) {
        float k_I = gains.I;
        if (is_zero(gains.FF)) {
            /*
              if the user hasn't set a direct FF then assume they are
              not doing sophisticated tuning. Set a minimum I value of
              0.15 to ensure that the time constant for trimming in
              pitch is not too long. We have had a lot of user issues
              with very small I value leading to very slow pitch
              trimming, which causes a lot of problems for TECS. A
              value of 0.15 is still quite small, but a lot better
              than what many users are running.
             */
            k_I = MAX(k_I, 0.15f);
        }
        float ki_rate = k_I * gains.tau;
		//only integrate if gain and time step are positive and airspeed above min value.
		if (dt > 0 && aspeed > 0.5f*float(aparm.airspeed_min)) {
		    float integrator_delta = rate_error * ki_rate * delta_time * scaler;
			if (_last_out < -45) {
				// prevent the integrator from increasing if surface defln demand is above the upper limit
				integrator_delta = MAX(integrator_delta , 0);
			} else if (_last_out > 45) {
				// prevent the integrator from decreasing if surface defln demand  is below the lower limit
				integrator_delta = MIN(integrator_delta , 0);
			}
			_pid_info.I += integrator_delta;
		}
	} else {
		_pid_info.I = 0;
	}

    // Scale the integration limit
    float intLimScaled = gains.imax * 0.01f;

    // Constrain the integrator state
    _pid_info.I = constrain_float(_pid_info.I, -intLimScaled, intLimScaled);

	// Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    // No conversion is required for K_D
    float eas2tas = _ahrs.get_EAS2TAS();
	float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau  - gains.D , 0) / eas2tas;
    float k_ff = gains.FF / eas2tas;
	
	// Calculate the demanded control surface deflection
	// Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
	// path, but want a 1/speed^2 scaler applied to the rate error path. 
	// This is because acceleration scales with speed^2, but rate scales with speed.
    _pid_info.P = desired_rate * kp_ff * scaler;
    _pid_info.FF = desired_rate * k_ff * scaler;
    _pid_info.D = rate_error * gains.D * scaler;
	_last_out = _pid_info.D + _pid_info.FF + _pid_info.P;
    _pid_info.desired = desired_rate;

    if (autotune.running && aspeed > aparm.airspeed_min) {
        // let autotune have a go at the values 
        // Note that we don't pass the integrator component so we get
        // a better idea of how much the base PD controller
        // contributed
        autotune.update(desired_rate, achieved_rate, _last_out);
        
        // set down rate to rate up when auto-tuning
        _max_rate_neg.set_and_save_ifchanged(gains.rmax);
    }

	_last_out += _pid_info.I;

    /*
      when we are past the users defined roll limit for the
      aircraft our priority should be to bring the aircraft back
      within the roll limit. Using elevator for pitch control at
      large roll angles is ineffective, and can be counter
      productive as it induces earth-frame yaw which can reduce
      the ability to roll. We linearly reduce elevator input when
      beyond the configured roll limit, reducing to zero at 90
      degrees
    */
    float roll_wrapped = fabsf(_ahrs.roll_sensor);
    if (roll_wrapped > 9000) {
        roll_wrapped = 18000 - roll_wrapped;
    }
    if (roll_wrapped > aparm.roll_limit_cd + 500 && aparm.roll_limit_cd < 8500 &&
        labs(_ahrs.pitch_sensor) < 7000) {
        float roll_prop = (roll_wrapped - (aparm.roll_limit_cd+500)) / (float)(9000 - aparm.roll_limit_cd);
        _last_out *= (1 - roll_prop);
    }
    
	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}

/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded pitch rate in degrees/second
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
 5) maximum FBW airspeed (metres/sec)
*/
int32_t AP_PitchController::get_rate_out(float desired_rate, float scaler)
{
    float aspeed;
	if (!_ahrs.airspeed_estimate(&aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
	}
    return _get_rate_out(desired_rate, scaler, false, aspeed);
}

/*
  get the rate offset in degrees/second needed for pitch in body frame
  to maintain height in a coordinated turn.

  Also returns the inverted flag and the estimated airspeed in m/s for
  use by the rest of the pitch controller
 */
float AP_PitchController::_get_coordination_rate_offset(float &aspeed, bool &inverted) const
{
	float rate_offset;
	float bank_angle = _ahrs.roll;

	// limit bank angle between +- 80 deg if right way up
	if (fabsf(bank_angle) < radians(90))	{
	    bank_angle = constrain_float(bank_angle,-radians(80),radians(80));
        inverted = false;
	} else {
		inverted = true;
		if (bank_angle > 0.0f) {
			bank_angle = constrain_float(bank_angle,radians(100),radians(180));
		} else {
			bank_angle = constrain_float(bank_angle,-radians(180),-radians(100));
		}
	}
	if (!_ahrs.airspeed_estimate(&aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
	}
    if (abs(_ahrs.pitch_sensor) > 7000) {
        // don't do turn coordination handling when at very high pitch angles
        rate_offset = 0;
    } else {
        rate_offset = cosf(_ahrs.pitch)*fabsf(ToDeg((GRAVITY_MSS / MAX((aspeed * _ahrs.get_EAS2TAS()) , float(aparm.airspeed_min))) * tanf(bank_angle) * sinf(bank_angle))) * _roll_ff;
    }
	if (inverted) {
		rate_offset = -rate_offset;
	}
    return rate_offset;
}

// Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
// A positive demand is up
// Inputs are: 
// 1) demanded pitch angle in centi-degrees
// 2) control gain scaler = scaling_speed / aspeed
// 3) boolean which is true when stabilise mode is active
// 4) minimum FBW airspeed (metres/sec)
// 5) maximum FBW airspeed (metres/sec)
//
int32_t AP_PitchController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
	// Calculate offset to pitch rate demand required to maintain pitch angle whilst banking
	// Calculate ideal turn rate from bank angle and airspeed assuming a level coordinated turn
	// Pitch rate offset is the component of turn rate about the pitch axis
	float aspeed;
	float rate_offset;
	bool inverted;

    if (gains.tau < 0.1f) {
        gains.tau.set(0.1f);
    }

    rate_offset = _get_coordination_rate_offset(aspeed, inverted);
	
	// Calculate the desired pitch rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f / gains.tau;
	
	// limit the maximum pitch rate demand. Don't apply when inverted
	// as the rates will be tuned when upright, and it is common that
	// much higher rates are needed inverted	
	if (!inverted) {
		if (_max_rate_neg && desired_rate < -_max_rate_neg) {
			desired_rate = -_max_rate_neg;
		} else if (gains.rmax && desired_rate > gains.rmax) {
			desired_rate = gains.rmax;
		}
	}
	
	if (inverted) {
		desired_rate = -desired_rate;
	}

	// Apply the turn correction offset
	desired_rate = desired_rate + rate_offset;

    return _get_rate_out(desired_rate, scaler, disable_integrator, aspeed);
}

void AP_PitchController::reset_I()
{
	_pid_info.I = 0;
}
