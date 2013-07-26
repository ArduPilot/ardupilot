// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AP_TECS.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 #include <stdio.h>
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif
//Debug("%.2f %.2f %.2f %.2f \n", var1, var2, var3, var4);

// table of user settable parameters
const AP_Param::GroupInfo AP_TECS::var_info[] PROGMEM = {

    // @Param: CLMB_MAX
    // @DisplayName: Maximum Climb Rate (metres/sec)
    // @Description: This is the best climb rate that the aircraft can achieve with the throttle set to THR_MAX and the airspeed set to the default value. For electric aircraft make sure this number can be achieved towards the end of flight when the battery voltage has reduced. The setting of this parameter can be checked by commanding a positive altitude change of 100m in loiter, RTL or guided mode. If the throttle required to climb is close to THR_MAX and the aircraft is maintaining airspeed, then this parameter is set correctly. If the airspeed starts to reduce, then the parameter is set to high, and if the throttle demand require to climb and maintain speed is noticeably less than THR_MAX, then either CLMB_MAX should be increased or THR_MAX reduced.
	// @Increment: 0.1
	// @User: User
    AP_GROUPINFO("CLMB_MAX",    0, AP_TECS, _maxClimbRate, 5.0f),

    // @Param: SINK_MIN
    // @DisplayName: Minimum Sink Rate (metres/sec)
    // @Description: This is the sink rate of the aircraft with the throttle set to THR_MIN and the same airspeed as used to measure CLMB_MAX.
	// @Increment: 0.1
	// @User: User
    AP_GROUPINFO("SINK_MIN",    1, AP_TECS, _minSinkRate, 2.0f),

    // @Param: TIME_CONST
    // @DisplayName: Controller time constant (sec)
    // @Description: This is the time constant of the TECS control algorithm. Smaller values make it faster to respond, large values make it slower to respond.
	// @Range: 3.0-10.0
	// @Increment: 0.2
	// @User: Advanced
    AP_GROUPINFO("TIME_CONST",  2, AP_TECS, _timeConst, 5.0f),

    // @Param: THR_DAMP
    // @DisplayName: Controller throttle damping
    // @Description: This is the damping gain for the throttle demand loop. Increase to add damping  to correct for oscillations in speed and height.
	// @Range: 0.1-1.0
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("THR_DAMP",    3, AP_TECS, _thrDamp, 0.5f),

    // @Param: INTEG_GAIN
    // @DisplayName: Controller integrator
    // @Description: This is the integrator gain on the control loop. Increase to increase the rate at which speed and height offsets are trimmed out
	// @Range: 0.0-0.5
	// @Increment: 0.02
	// @User: Advanced
    AP_GROUPINFO("INTEG_GAIN", 4, AP_TECS, _integGain, 0.1f),

    // @Param: VERT_ACC
    // @DisplayName: Vertical Acceleration Limit (metres/sec^2)
    // @Description: This is the maximum vertical acceleration either up or down that the  controller will use to correct speed or height errors.
	// @Range: 1.0-10.0
	// @Increment: 0.5
	// @User: Advanced
    AP_GROUPINFO("VERT_ACC",  5, AP_TECS, _vertAccLim, 7.0f),

    // @Param: HGT_OMEGA
    // @DisplayName: Height complementary filter frequency (radians/sec)
    // @Description: This is the cross-over frequency of the complementary filter used to fuse vertical acceleration and baro alt to obtain an estimate of height rate and height.
	// @Range: 1.0-5.0
	// @Increment: 0.05
	// @User: Advanced
    AP_GROUPINFO("HGT_OMEGA", 6, AP_TECS, _hgtCompFiltOmega, 3.0f),

    // @Param: SPD_OMEGA
    // @DisplayName: Speed complementary filter frequency (radians/sec)
    // @Description: This is the cross-over frequency of the complementary filter used to fuse longitudinal acceleration and airspeed to obtain a lower noise and lag estimate of airspeed.
	// @Range: 0.5-2.0
	// @Increment: 0.05
	// @User: Advanced
    AP_GROUPINFO("SPD_OMEGA", 7, AP_TECS, _spdCompFiltOmega, 2.0f),

    // @Param: RLL2THR
    // @DisplayName: Bank angle compensation gain
    // @Description: Increasing this gain turn increases the amount of throttle that will be used to compensate for the additional drag created by turning. Ideally this should be set to approximately 10 x the extra sink rate in m/s created by a 45 degree bank turn. Increase this gain if the aircraft initially loses energy in turns and reduce if the aircraft initially gains energy in turns. Efficient high aspect-ratio aircraft (eg powered sailplanes) can use a lower value, whereas inefficient low aspect-ratio models (eg delta wings) can use a higher value.
	// @Range: 5.0 to 30.0
	// @Increment: 1.0
	// @User: Advanced
    AP_GROUPINFO("RLL2THR",  8, AP_TECS, _rollComp, 10.0f),
 
    // @Param: SPDWEIGHT
    // @DisplayName: Weighting applied to speed control
    // @Description: This parameter adjusts the amount of weighting that the pitch control applies to speed vs height errors. Setting it to 0.0 will cause the pitch control to control height and ignore speed errors. This will normally improve height accuracy but give larger airspeed errors. Setting it to 2.0 will cause the pitch control loop to control speed and ignore height errors. This will normally reduce airsped errors, but give larger height errors.	A value of 1.0 gives a balanced response and is the default.
	// @Range: 0.0 to 2.0
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("SPDWEIGHT", 9, AP_TECS, _spdWeight, 1.0f),
 
    // @Param: PTCH_DAMP
    // @DisplayName: Controller pitch damping
    // @Description: This is the damping gain for the pitch demand loop. Increase to add damping  to correct for oscillations in speed and height.
	// @Range: 0.1-1.0
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("PTCH_DAMP", 10, AP_TECS, _ptchDamp, 0.0f),

    // @Param: SINK_MAX
    // @DisplayName: Maximum Descent Rate (metres/sec)
    // @Description: This sets the maximum descent rate that the controller will use.  If this value is too large, the aircraft will reach the pitch angle limit first and be enable to achieve the descent rate. This should be set to a value that can be achieved at the lower pitch angle limit.
	// @Increment: 0.1
	// @User: User
    AP_GROUPINFO("SINK_MAX",  11, AP_TECS, _maxSinkRate, 5.0f),

    AP_GROUPEND
};

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to 
 *    height priority
 *  - Underspeed protection that demands maximum throttle and switches pitch angle control 
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, integrator and damping gains and the use
 *    of easy to measure aircraft performance data
 *
 */

void AP_TECS::update_50hz(float hgt_afe)
{ 
	// Implement third order complementary filter for height and height rate
	// estimted height rate = _integ2_state
	// estimated height above field elevation  = _integ3_state
    // Reference Paper : 
	// Optimising the Gains of the Baro-Inertial Vertical Channel
	// Widnall W.S, Sinha P.K, 
	// AIAA Journal of Guidance and Control, 78-1307R

    // Calculate time in seconds since last update
    uint32_t now = hal.scheduler->micros();
	float DT = max((now - _update_50hz_last_usec),0)*1.0e-6f;
	if (DT > 1.0) {
	    _integ3_state = hgt_afe;
		_integ2_state = 0.0f;
		_integ1_state = 0.0f;
		DT            = 0.02f; // when first starting TECS, use a
							   // small time constant
	}
	_update_50hz_last_usec = now;	

	// Get height acceleration
	float hgt_ddot_mea = -(_ahrs->get_accel_ef().z + GRAVITY_MSS);
	// Perform filter calculation using backwards Euler integration
    // Coefficients selected to place all three filter poles at omega
	float omega2 = _hgtCompFiltOmega*_hgtCompFiltOmega;
	float hgt_err = hgt_afe - _integ3_state;
	float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;
	_integ1_state = _integ1_state + integ1_input * DT;
	float integ2_input = _integ1_state + hgt_ddot_mea + hgt_err * omega2 * 3.0f;
	_integ2_state = _integ2_state + integ2_input * DT;
	float integ3_input = _integ2_state + hgt_err * _hgtCompFiltOmega * 3.0f;
    // If more than 1 second has elapsed since last update then reset the integrator state
    // to the measured height
    if (DT > 1.0) {
        _integ3_state = hgt_afe;
    } else {
	    _integ3_state = _integ3_state + integ3_input*DT;
    }

	// Update and average speed rate of change
    // Only required if airspeed is being measured and controlled
    float temp = 0;
	if (_ahrs->airspeed_sensor_enabled() && _ahrs->airspeed_estimate_true(&_EAS)) {
        // Get DCM
        const Matrix3f &rotMat = _ahrs->get_dcm_matrix();
	    // Calculate speed rate of change
	    temp = rotMat.c.x * GRAVITY_MSS + _ahrs->get_ins()->get_accel().x;
	    // take 5 point moving average
        _vel_dot = _vdot_filter.apply(temp);
    } else {
       _vel_dot = 0.0f;
    }

}

void AP_TECS::_update_speed(void)
{
    // Calculate time in seconds since last update
    uint32_t now = hal.scheduler->micros();
	float DT = max((now - _update_speed_last_usec),0)*1.0e-6f;
	_update_speed_last_usec = now;	

    // Convert equivalent airspeeds to true airspeeds

    float EAS2TAS = _ahrs->get_EAS2TAS();
    _TAS_dem  = _EAS_dem * EAS2TAS;
    _TASmax   = aparm.airspeed_max * EAS2TAS;
    _TASmin   = aparm.airspeed_min * EAS2TAS;

    // Reset states of time since last update is too large
    if (DT > 1.0) {
        _integ5_state = (_EAS * EAS2TAS);
        _integ4_state = 0.0f;
		DT            = 0.1f; // when first starting TECS, use a
							  // small time constant
    }

    // Get airspeed or default to halfway between min and max if
    // airspeed is not being used and set speed rate to zero
    if (!_ahrs->airspeed_sensor_enabled() || !_ahrs->airspeed_estimate(&_EAS)) {
        // If no airspeed available use average of min and max
        _EAS = 0.5f * (aparm.airspeed_min + aparm.airspeed_max);
    }

    // Implement a second order complementary filter to obtain a
    // smoothed airspeed estimate
    // airspeed estimate is held in _integ5_state
    float aspdErr = (_EAS * EAS2TAS) - _integ5_state;
    float integ4_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;
    // Prevent state from winding up
    if (_integ5_state < 3.1){
        integ4_input = max(integ4_input , 0.0);
    }
    _integ4_state = _integ4_state + integ4_input * DT;
    float integ5_input = _integ4_state + _vel_dot + aspdErr * _spdCompFiltOmega * 1.4142f;
    _integ5_state = _integ5_state + integ5_input * DT;
    // limit the airspeed to a minimum of 3 m/s
    _integ5_state = max(_integ5_state, 3.0f);

}

void AP_TECS::_update_speed_demand(void)
{
	// Set the airspeed demand to the minimum value if an underspeed condition exists
	// or a bad descent condition exists
	// This will minimise the rate of descent resulting from an engine failure,
	// enable the maximum climb rate to be achieved and prevent continued full power descent
	// into the ground due to an unachievable airspeed value
	if ((_badDescent) || (_underspeed))
	{
		_TAS_dem     = _TASmin;
	}
	
    // Constrain speed demand
    _TAS_dem = constrain_float(_TAS_dem, _TASmin, _TASmax);

    // calculate velocity rate limits based on physical performance limits
	// provision to use a different rate limit if bad descent or underspeed condition exists
	// Use 50% of maximum energy rate to allow margin for total energy contgroller
    float velRateMax;
    float velRateMin;
	if ((_badDescent) || (_underspeed))
	{
    	velRateMax = 0.5f * _STEdot_max / _integ5_state;
    	velRateMin = 0.5f * _STEdot_min / _integ5_state;
	}
	else
	{
    	velRateMax = 0.5f * _STEdot_max / _integ5_state;
    	velRateMin = 0.5f * _STEdot_min / _integ5_state;
	}
	
    // Apply rate limit
    if ((_TAS_dem - _TAS_dem_adj) > (velRateMax * 0.1f))
    {
        _TAS_dem_adj = _TAS_dem_adj + velRateMax * 0.1f;
        _TAS_rate_dem = velRateMax;
    }
    else if ((_TAS_dem - _TAS_dem_adj) < (velRateMin * 0.1f))
    {
        _TAS_dem_adj = _TAS_dem_adj + velRateMin * 0.1f;
        _TAS_rate_dem = velRateMin;
    }
    else
    {
        _TAS_dem_adj = _TAS_dem;
        _TAS_rate_dem = (_TAS_dem - _TAS_dem_last) / 0.1f;
    }
    // Constrain speed demand again to protect against bad values on initialisation.
    _TAS_dem_adj = constrain_float(_TAS_dem_adj, _TASmin, _TASmax);
    _TAS_dem_last = _TAS_dem;
}

void AP_TECS::_update_height_demand(void)
{
	// Apply 2 point moving average to demanded height
	// This is required because height demand is only updated at 5Hz
	_hgt_dem = 0.5f * (_hgt_dem + _hgt_dem_in_old);
	_hgt_dem_in_old = _hgt_dem;

    // Limit height rate of change
	if ((_hgt_dem - _hgt_dem_prev) > (_maxClimbRate * 0.1f))
    {
        _hgt_dem = _hgt_dem_prev + _maxClimbRate * 0.1f;
    }
    else if ((_hgt_dem - _hgt_dem_prev) < (-_maxSinkRate * 0.1f))
    {
        _hgt_dem = _hgt_dem_prev - _maxSinkRate * 0.1f;
    }
    _hgt_dem_prev = _hgt_dem;

	// Apply first order lag to height demand
	_hgt_dem_adj = 0.05f * _hgt_dem + 0.95f * _hgt_dem_adj_last;
    _hgt_rate_dem = (_hgt_dem_adj - _hgt_dem_adj_last) / 0.1f;
	_hgt_dem_adj_last = _hgt_dem_adj;
}

void AP_TECS::_detect_underspeed(void) 
{
    if (((_integ5_state < _TASmin * 0.9f) && (_throttle_dem >= _THRmaxf * 0.95f)) || ((_integ3_state < _hgt_dem_adj) && _underspeed))
    {
        _underspeed = true;
    }
    else
    {
        _underspeed = false;
    }
}

void AP_TECS::_update_energies(void) 
{
    // Calculate specific energy demands
    _SPE_dem = _hgt_dem_adj * GRAVITY_MSS;
    _SKE_dem = 0.5f * _TAS_dem_adj * _TAS_dem_adj;

    // Calculate specific energy rate demands
    _SPEdot_dem = _hgt_rate_dem * GRAVITY_MSS;
    _SKEdot_dem = _integ5_state * _TAS_rate_dem;

    // Calculate specific energy
    _SPE_est = _integ3_state * GRAVITY_MSS;
    _SKE_est = 0.5f * _integ5_state * _integ5_state;

    // Calculate specific energy rate
    _SPEdot = _integ2_state * GRAVITY_MSS;
    _SKEdot = _integ5_state * _vel_dot;
}

void AP_TECS::_update_throttle(void)
{
    // Calculate total energy values
    _STE_error = _SPE_dem - _SPE_est + _SKE_dem - _SKE_est;
    float STEdot_dem = constrain_float((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);
    float STEdot_error = STEdot_dem - _SPEdot - _SKEdot;

	// Apply 0.5 second first order filter to STEdot_error
	// This is required to remove accelerometer noise from the  measurement
	STEdot_error = 0.2f*STEdot_error + 0.8f*_STEdotErrLast;
	_STEdotErrLast = STEdot_error;

    // Calculate throttle demand
    // If underspeed condition is set, then demand full throttle
    if (_underspeed)
    {
        _throttle_dem_unc = 1.0f;
    }
    else
    {
		// Calculate gain scaler from specific energy error to throttle
		float K_STE2Thr = 1 / (_timeConst * (_STEdot_max - _STEdot_min));

        // Calculate feed-forward throttle
        float ff_throttle = 0;
		float nomThr = aparm.throttle_cruise * 0.01f;
		const Matrix3f &rotMat = _ahrs->get_dcm_matrix();
		// Use the demanded rate of change of total energy as the feed-forward demand, but add
		// additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
		// drag increase during turns.
		float cosPhi = sqrt((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
		STEdot_dem = STEdot_dem + _rollComp * (1.0f/constrain_float(cosPhi * cosPhi , 0.1f, 1.0f) - 1.0f);
		if (STEdot_dem >= 0)
		{
			ff_throttle = nomThr + STEdot_dem / _STEdot_max * (1.0f - nomThr);
		}
		else
		{
			ff_throttle = nomThr - STEdot_dem / _STEdot_min * nomThr;
		}

		// Calculate PD + FF throttle
		_throttle_dem = (_STE_error + STEdot_error * _thrDamp) * K_STE2Thr + ff_throttle;

		// Rate limit PD + FF throttle
	    // Calculate the throttle increment from the specified slew time
		if (aparm.throttle_slewrate != 0) {
			float thrRateIncr = _DT * (_THRmaxf - _THRminf) * aparm.throttle_slewrate * 0.01f;

			_throttle_dem = constrain_float(_throttle_dem, 
											_last_throttle_dem - thrRateIncr, 
											_last_throttle_dem + thrRateIncr);
			_last_throttle_dem = _throttle_dem;
		}


		// Calculate integrator state upper and lower limits
		// Set to a value thqat will allow 0.1 (10%) throttle saturation to allow for noise on the demand
        float integ_max = (_THRmaxf - _throttle_dem + 0.1f);
		float integ_min = (_THRminf - _throttle_dem - 0.1f);

  		// Calculate integrator state, constraining state
		// Set integrator to a max throttle value dduring climbout
        _integ6_state = _integ6_state + (_STE_error * _integGain) * _DT * K_STE2Thr;
		if (_climbOutDem)
		{
			_integ6_state = integ_max;
		}
		else
		{
			_integ6_state = constrain_float(_integ6_state, integ_min, integ_max);
		}

		// Sum the components. 
		// Only use feed-forward component if airspeed is not being used
	    if (_ahrs->airspeed_sensor_enabled()) {
	        _throttle_dem = _throttle_dem + _integ6_state;
	    } else {
 	       _throttle_dem = ff_throttle;
	    }
    }

    // Constrain throttle demand
    _throttle_dem = constrain_float(_throttle_dem, _THRminf, _THRmaxf);
}

void AP_TECS::_update_throttle_option(int16_t throttle_nudge)
{
	// Calculate throttle demand by interpolating between pitch and throttle limits
    float nomThr = (aparm.throttle_cruise + throttle_nudge)* 0.01f;	
	if (_climbOutDem)
	{
		_throttle_dem = _THRmaxf;
	}
	else if (_pitch_dem > 0.0 && _PITCHmaxf > 0.0)
	{
		_throttle_dem = nomThr + (_THRmaxf - nomThr) * _pitch_dem / _PITCHmaxf;
	}
	else if (_pitch_dem < 0.0 && _PITCHminf < 0.0)
	{
		_throttle_dem = nomThr + (_THRminf - nomThr) * _pitch_dem / _PITCHminf;
	}
	else
	{
		_throttle_dem = nomThr;
	}
}

void AP_TECS::_detect_bad_descent(void) 
{
	// Detect a demanded airspeed too high for the aircraft to achieve. This will be
	// evident by the the following conditions:
	// 1) Underspeed protection not active
	// 2) Specific total energy error > 200 (greater than ~20m height error)
	// 3) Specific total energy reducing 
	// 4) throttle demand > 90%
	// If these four conditions exist simultaneously, then the protection
	// mode will be activated. 
	// Once active, the following condition are required to stay in the mode
	// 1) Underspeed protection not active
	// 2) Specific total energy error > 0
	// This mode will produce an undulating speed and height response as it cuts in and out but will prevent the aircraft from descending into the ground if an unachievable speed demand is set
	float STEdot = _SPEdot + _SKEdot;
	if ((!_underspeed && (_STE_error > 200.0f) && (STEdot < 0.0f) && (_throttle_dem >= _THRmaxf * 0.9f)) || (_badDescent && !_underspeed && (_STE_error > 0.0f)))
	{
		_badDescent = true;
	}
	else
	{
		_badDescent = false;
	}
}

void AP_TECS::_update_pitch(void) 
{
	// Calculate Speed/Height Control Weighting
    // This is used to determine how the pitch control prioritises speed and height control
    // A weighting of 1 provides equal priority (this is the normal mode of operation)
    // A SKE_weighting of 0 provides 100% priority to height control. This is used when no airspeed measurement is available
	// A SKE_weighting of 2 provides 100% priority to speed control. This is used when an underspeed condition is detected
	// or during takeoff/climbout where a minimum pitch angle is set to ensure height is gained. In this instance, if airspeed
	// rises above the demanded value, the pitch angle will be increased by the TECS controller.
	float SKE_weighting = constrain_float(_spdWeight, 0.0f, 2.0f);
	if ( ( _underspeed || _climbOutDem ) && _ahrs->airspeed_sensor_enabled() ) {
		SKE_weighting = 2.0f;
	} else if (!_ahrs->airspeed_sensor_enabled()) {
		SKE_weighting = 0.0f;
	}
	float SPE_weighting = 2.0f - SKE_weighting;

    // Calculate Specific Energy Balance demand, and error
	float SEB_dem      = _SPE_dem * SPE_weighting - _SKE_dem * SKE_weighting;
	float SEBdot_dem   = _SPEdot_dem * SPE_weighting - _SKEdot_dem * SKE_weighting;
	float SEB_error    = SEB_dem - (_SPE_est * SPE_weighting - _SKE_est * SKE_weighting);
	float SEBdot_error = SEBdot_dem - (_SPEdot * SPE_weighting - _SKEdot * SKE_weighting);

    // Calculate integrator state, constraining input if pitch limits are exceeded
    float integ7_input = SEB_error * _integGain;
    if (_pitch_dem_unc > _PITCHmaxf) 
    {
        integ7_input = min(integ7_input, _PITCHmaxf - _pitch_dem_unc);
    }
    else if (_pitch_dem_unc < _PITCHminf)
    {
        integ7_input = max(integ7_input, _PITCHminf - _pitch_dem_unc);
    }
    _integ7_state = _integ7_state + integ7_input * _DT;

    // Apply max and min values for integrator state that will allow for no more than 
	// 5deg of saturation. This allows for some pitch variation due to gusts before the 
	// integrator is clipped. Otherwise the effectiveness of the integrator will be reduced in turbulence
	float gainInv = (_integ5_state * _timeConst * GRAVITY_MSS);
    float temp = SEB_error + SEBdot_error * _ptchDamp + SEBdot_dem * _timeConst;
	_integ7_state = constrain_float(_integ7_state, (gainInv * (_PITCHminf - 0.0783f)) - temp, (gainInv * (_PITCHmaxf + 0.0783f)) - temp);

    // Calculate pitch demand from specific energy balance signals
    _pitch_dem_unc = (temp + _integ7_state) / gainInv;

    // Constrain pitch demand
    _pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);

    // Rate limit the pitch demand to comply with specified vertical
    // acceleration limit
    float ptchRateIncr = _DT * _vertAccLim / _integ5_state;

    if ((_pitch_dem - _last_pitch_dem) > ptchRateIncr)
    {
		_pitch_dem = _last_pitch_dem + ptchRateIncr;
	}
		else if ((_pitch_dem - _last_pitch_dem) < -ptchRateIncr)
	{
		_pitch_dem = _last_pitch_dem - ptchRateIncr;
	}
	_last_pitch_dem = _pitch_dem;
}

void AP_TECS::_initialise_states(int32_t ptchMinCO_cd, float hgt_afe) 
{
	// Initialise states and variables if DT > 1 second or in climbout
	if (_DT > 1.0)
	{
		_integ6_state      = 0.0f;
		_integ7_state      = 0.0f;
		_last_throttle_dem = aparm.throttle_cruise * 0.01f;
		_last_pitch_dem    = _ahrs->pitch;
		_hgt_dem_adj_last  = hgt_afe;
		_hgt_dem_adj       = _hgt_dem_adj_last;
		_hgt_dem_prev      = _hgt_dem_adj_last;
        _hgt_dem_in_old    = _hgt_dem_adj_last;
        _TAS_dem_last      = _TAS_dem;
        _TAS_dem_adj       = _TAS_dem;
		_underspeed        = false;
		_badDescent        = false;
		_DT                = 0.1f; // when first starting TECS, use a
								   // small time constant
	}
	else if (_climbOutDem)
	{
		_PITCHminf          = 0.000174533f * ptchMinCO_cd;
		_THRminf            = _THRmaxf - 0.01f;
		_hgt_dem_adj_last  = hgt_afe;
		_hgt_dem_adj       = _hgt_dem_adj_last;
		_hgt_dem_prev      = _hgt_dem_adj_last;
        _TAS_dem_last      = _TAS_dem;
        _TAS_dem_adj       = _TAS_dem;
		_underspeed        = false;
		_badDescent 	   = false;
	}
}

void AP_TECS::_update_STE_rate_lim(void) 
{
    // Calculate Specific Total Energy Rate Limits
	// This is a tivial calculation at the moment but will get bigger once we start adding altitude effects
    _STEdot_max = _maxClimbRate * GRAVITY_MSS;
    _STEdot_min = - _minSinkRate * GRAVITY_MSS;
}

void AP_TECS::update_pitch_throttle(int32_t hgt_dem_cm,
									int32_t EAS_dem_cm, 
									bool climbOutDem, 
									int32_t ptchMinCO_cd, 
									int16_t throttle_nudge,
									float hgt_afe) 
{
    // Calculate time in seconds since last update
    uint32_t now = hal.scheduler->micros();
	_DT = max((now - _update_pitch_throttle_last_usec),0)*1.0e-6f;
	_update_pitch_throttle_last_usec = now;	

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed();

	// Convert inputs
    _hgt_dem = hgt_dem_cm * 0.01f;
	_EAS_dem = EAS_dem_cm * 0.01f;
    _THRmaxf  = aparm.throttle_max * 0.01f;
    _THRminf  = aparm.throttle_min * 0.01f;
	_PITCHmaxf = 0.000174533f * aparm.pitch_limit_max_cd;
	_PITCHminf = 0.000174533f * aparm.pitch_limit_min_cd;
	_climbOutDem = climbOutDem;

	// initialise selected states and variables if DT > 1 second or in climbout
	_initialise_states(ptchMinCO_cd, hgt_afe);

    // Calculate Specific Total Energy Rate Limits
	_update_STE_rate_lim();

    // Calculate the speed demand
    _update_speed_demand();

	// Calculate the height demand
	_update_height_demand();

    // Detect underspeed condition
    _detect_underspeed();

    // Calculate specific energy quantitiues
    _update_energies();

    // Calculate throttle demand - use simple pitch to throttle if no airspeed sensor
	if (_ahrs->airspeed_sensor_enabled()) {
        _update_throttle();
	} else {
        _update_throttle_option(throttle_nudge);
	}

    // Detect bad descent due to demanded airspeed being too high
	_detect_bad_descent();

	// Calculate pitch demand
	_update_pitch();

    // Write internal variables to the log_tuning structure. This
    // structure will be logged in dataflash at 10Hz
	log_tuning.hgt_dem  = _hgt_dem_adj;
	log_tuning.hgt      = _integ3_state;
	log_tuning.dhgt_dem = _hgt_rate_dem;
	log_tuning.dhgt     = _integ2_state;
	log_tuning.spd_dem  = _TAS_dem_adj;
	log_tuning.spd      = _integ5_state;
	log_tuning.dspd     = _vel_dot;
	log_tuning.ithr     = _integ6_state;
	log_tuning.iptch    = _integ7_state;
	log_tuning.thr      = _throttle_dem;
	log_tuning.ptch     = _pitch_dem;
	log_tuning.dspd_dem = _TAS_rate_dem;
}

// log the contents of the log_tuning structure to dataflash
void AP_TECS::log_data(DataFlash_Class &dataflash, uint8_t msgid)
{
    log_tuning.head1 = HEAD_BYTE1;
    log_tuning.head2 = HEAD_BYTE2;
    log_tuning.msgid = msgid;
	dataflash.WriteBlock(&log_tuning, sizeof(log_tuning));
}
