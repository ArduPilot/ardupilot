// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
    // @DisplayName: 最大爬升速度（m/s）
    // @Description: 这是飞行器在油门达到THR_MAX并且空速为default时能达到的最大爬升速度。对于电动飞行器请确认这个数值在飞行最后电池电压减小的情况下仍然能够被达到。这个数字的设置能够通过控制正向100m的高度变化在Loiter，RTL或者guided模式下来检查。如果需要的油门接近THR_MAX并且飞行器保持空速，那么这个参数就设置正确了。如果空速开始减小，那么参数就设置的太高，如果空速油门达到所需的爬升速度并且维持且显著地低于THR_MAX，那么CLMB_MAX应该被增加或者THR_MAX应该被减小。
	// @Increment: 0.1
	// @Range: 0.1 20.0
	// @User: User
    AP_GROUPINFO("CLMB_MAX",    0, AP_TECS, _maxClimbRate, 5.0f),

    // @Param: SINK_MIN
    // @DisplayName: 最小下降速度（m/s）
    // @Description: 这是飞行器在油门位于THR_MIN并且空速和用来测量CLMB_MAX的一样时的最小下降速度。
	// @Increment: 0.1
	// @Range: 0.1 10.0
	// @User: User
    AP_GROUPINFO("SINK_MIN",    1, AP_TECS, _minSinkRate, 2.0f),

    // @Param: TIME_CONST
    // @DisplayName: 控制器时间常数（s）
    // @Description: 这是TECS控制算法的时间常数。更小的值让它更快的相应，更大的值让更慢的相应。
	// @Range: 3.0 10.0
	// @Increment: 0.2
	// @User: Advanced
    AP_GROUPINFO("TIME_CONST",  2, AP_TECS, _timeConst, 5.0f),

    // @Param: THR_DAMP
    // @DisplayName: 控制器油门抑制
    // @Description: 这是用于油门需求循环的抑制增益。增加会提高增益来校正速度和高度上的抖动。
	// @Range: 0.1 1.0
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("THR_DAMP",    3, AP_TECS, _thrDamp, 0.5f),

    // @Param: INTEG_GAIN
    // @DisplayName: 控制器积分器
    // @Description: 这是控制循环的积分增益。增加会增加速度和高度位移的剪切速度。
	// @Range: 0.0 0.5
	// @Increment: 0.02
	// @User: Advanced
    AP_GROUPINFO("INTEG_GAIN", 4, AP_TECS, _integGain, 0.1f),

    // @Param: VERT_ACC
    // @DisplayName: 垂直加速度限制（m/s^2)
    // @Description: 这是上升或下降的最大垂直加速度，用于修正速度或高度误差。
	// @Range: 1.0 10.0
	// @Increment: 0.5
	// @User: Advanced
    AP_GROUPINFO("VERT_ACC",  5, AP_TECS, _vertAccLim, 7.0f),

    // @Param: HGT_OMEGA
    // @DisplayName: 高度辅助过滤器频率（rad/s)
    // @Description: 这是辅助过滤器用来结合垂直加速度和气压计高度估算高度和垂直速度的交叉频率
	// @Range: 1.0 5.0
	// @Increment: 0.05
	// @User: Advanced
    AP_GROUPINFO("HGT_OMEGA", 6, AP_TECS, _hgtCompFiltOmega, 3.0f),

    // @Param: SPD_OMEGA
    // @DisplayName: 速度辅助过滤器频率（rad/s）
    // @Description: 这是辅助过滤器用来结合纵向加速度和空速来获得更低的噪声和空速延迟的交叉频率
	// @Range: 0.5 2.0
	// @Increment: 0.05
	// @User: Advanced
    AP_GROUPINFO("SPD_OMEGA", 7, AP_TECS, _spdCompFiltOmega, 2.0f),

    // @Param: RLL2THR
    // @DisplayName: 倾斜角度补偿增益
    // @Description: 增大这个增益值会增大油门量以抵消转弯时的额外阻力。理想情况下这个应该设为45度角倾斜转弯时下降速度（m/s）的10倍。如果飞机一开始在转弯时缺乏能量，增加这个数值，如果能量过高，降低这个数值。高效的高长宽比的飞机（例如有动力的轻型滑翔机）可以使用更低的数值，而低效率低长宽比的飞机（例如三角翼）可以使用更高的数值。
	// @Range: 5.0 30.0
	// @Increment: 1.0
	// @User: Advanced
    AP_GROUPINFO("RLL2THR",  8, AP_TECS, _rollComp, 10.0f),
 
    // @Param: SPDWEIGHT
    // @DisplayName: 速度控制比重
    // @Description: 这个参数调节俯仰控制在速度和高度误差上的比重。设为0.0会导致俯仰控制控制高度而忽略速度误差。这会通常提高高度精度但是会造成更大的空速误差。设为2.0会导致俯仰控制控制速度而忽略高度误差。这通常会减少速度误差，但是会造成更大的高度误差。1.0附近的取值会给出平衡的响应并且这是默认值。
	// @Range: 0.0 2.0
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("SPDWEIGHT", 9, AP_TECS, _spdWeight, 1.0f),
 
    // @Param: PTCH_DAMP
    // @DisplayName: 控制器俯仰抑制
    // @Description: 这是俯仰需求循环的抑制增益。增大会提高抑制来校正速度和高度上的抖动。
	// @Range: 0.1 1.0
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("PTCH_DAMP", 10, AP_TECS, _ptchDamp, 0.0f),

    // @Param: SINK_MAX
    // @DisplayName: 最大下降速度（m/s）
    // @Description: 这个设置控制器使用的最大的下降速度。如果这个值过大，飞机会先达到俯仰角度限制而无法达到最大下降速度。这个值应该设为在最低俯仰角下飞机能够达到的最快下降速度。
	// @Increment: 0.1
	// @Range: 0.0 20.0
	// @User: User
    AP_GROUPINFO("SINK_MAX",  11, AP_TECS, _maxSinkRate, 5.0f),

    // @Param: LAND_ARSPD
    // @DisplayName: 下降接近时空速（m/s）
    // @Description: 当自动降落时，这个数值会被座位目标空速用于降落。注意如果你的平台没有空速传感器，那么这个参数无效（请换用TECS_LAND_THR）。如果数值是负值，降落期间这个数值将不会被使用。
    // @Range: -1 127
    // @Increment: 1
    // @User: User
    AP_GROUPINFO("LAND_ARSPD", 12, AP_TECS, _landAirspeed, -1),

    // @Param: LAND_THR
    // @DisplayName: 接近着陆时巡航油门（%）
    // @Description: 使用这个参数而不是LAND_ASPD如果你的平台没有空速传感器。这是接近着陆时的巡航油门。如果这个值是负值并且TECS_LAND_ASPD在使用，那么这个值在接近着陆时不会被使用。
    // @Range: -1 to 100
    // @Increment: 0.1
    // @User: User
    AP_GROUPINFO("LAND_THR", 13, AP_TECS, _landThrottle, -1),

    // @Param: LAND_SPDWGT
    // @DisplayName: Weighting applied to speed control during landing.
    // @Description: Same as SPDWEIGHT parameter, with the exception that this parameter is applied during landing flight stages.  A value closer to 2 will result in the plane ignoring height error during landing and our experience has been that the plane will therefore keep the nose up -- sometimes good for a glider landing (with the side effect that you will likely glide a ways past the landing point).  A value closer to 0 results in the plane ignoring speed error -- use caution when lowering the value below 1 -- ignoring speed could result in a stall.
	// @Range: 0.0 2.0
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("LAND_SPDWGT", 14, AP_TECS, _spdWeightLand, 1.0f),

    // @Param: PITCH_MAX
    // @DisplayName: Maximum pitch in auto flight
    // @Description: This controls maximum pitch up in automatic throttle modes. If this is set to zero then LIM_PITCH_MAX is used instead. The purpose of this parameter is to allow the use of a smaller pitch range when in automatic flight than what is used in FBWA mode.
	// @Range: 0 45
	// @Increment: 1
	// @User: Advanced
    AP_GROUPINFO("PITCH_MAX", 15, AP_TECS, _pitch_max, 0),

    // @Param: PITCH_MIN
    // @DisplayName: Minimum pitch in auto flight
    // @Description: This controls minimum pitch in automatic throttle modes. If this is set to zero then LIM_PITCH_MIN is used instead. The purpose of this parameter is to allow the use of a smaller pitch range when in automatic flight than what is used in FBWA mode. Note that TECS_PITCH_MIN should be a negative number.
	// @Range: -45 0
	// @Increment: 1
	// @User: Advanced
    AP_GROUPINFO("PITCH_MIN", 16, AP_TECS, _pitch_min, 0),

    // @Param: LAND_SINK
    // @DisplayName: Sink rate for final landing stage
    // @Description: The sink rate in meters/second for the final stage of landing.
	// @Range: 0.0 2.0
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("LAND_SINK", 17, AP_TECS, _land_sink, 0.25f),

    // @Param: LAND_TCONST
    // @DisplayName: Land controller time constant (sec)
    // @Description: This is the time constant of the TECS control algorithm when in final landing stage of flight. It should be smaller than TECS_TIME_CONST to allow for faster flare
	// @Range: 1.0 5.0
	// @Increment: 0.2
	// @User: Advanced
    AP_GROUPINFO("LAND_TCONST", 18, AP_TECS, _landTimeConst, 2.0f),

    // @Param: LAND_DAMP
    // @DisplayName: Controller sink rate to pitch gain during flare
    // @Description: This is the sink rate gain for the pitch demand loop when in final landing stage of flight. It should be larger than TECS_PTCH_DAMP to allow for better sink rate control during flare.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_DAMP", 19, AP_TECS, _landDamp, 0.5f),

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
	// estimted height rate = _climb_rate
	// estimated height above field elevation  = _integ3_state
    // Reference Paper : 
	// Optimising the Gains of the Baro-Inertial Vertical Channel
	// Widnall W.S, Sinha P.K, 
	// AIAA Journal of Guidance and Control, 78-1307R

    // Calculate time in seconds since last update
    uint32_t now = hal.scheduler->micros();
	float DT = max((now - _update_50hz_last_usec),0)*1.0e-6f;
	if (DT > 1.0f) {
	    _integ3_state = hgt_afe;
		_climb_rate = 0.0f;
		_integ1_state = 0.0f;
		DT            = 0.02f; // when first starting TECS, use a
							   // small time constant
	}
	_update_50hz_last_usec = now;	

	// USe inertial nav verical velocity and height if available
	Vector3f posned, velned;
	if (_ahrs.get_velocity_NED(velned) && _ahrs.get_relative_position_NED(posned)) {
		_climb_rate   = - velned.z;
		_integ3_state   = - posned.z;
	} else {
		// Get height acceleration
		float hgt_ddot_mea = -(_ahrs.get_accel_ef().z + GRAVITY_MSS);
		// Perform filter calculation using backwards Euler integration
		// Coefficients selected to place all three filter poles at omega
		float omega2 = _hgtCompFiltOmega*_hgtCompFiltOmega;
		float hgt_err = hgt_afe - _integ3_state;
		float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;
		_integ1_state = _integ1_state + integ1_input * DT;
		float integ2_input = _integ1_state + hgt_ddot_mea + hgt_err * omega2 * 3.0f;
		_climb_rate = _climb_rate + integ2_input * DT;
		float integ3_input = _climb_rate + hgt_err * _hgtCompFiltOmega * 3.0f;
		// If more than 1 second has elapsed since last update then reset the integrator state
		// to the measured height
		if (DT > 1.0f) {
		    _integ3_state = hgt_afe;
		} else {
			_integ3_state = _integ3_state + integ3_input*DT;
		}
	}

	// Update and average speed rate of change
    // Get DCM
    const Matrix3f &rotMat = _ahrs.get_dcm_matrix();
	// Calculate speed rate of change
	float temp = rotMat.c.x * GRAVITY_MSS + _ahrs.get_ins().get_accel().x;
	// take 5 point moving average
    _vel_dot = _vdot_filter.apply(temp);

}

void AP_TECS::_update_speed(float load_factor)
{
    // Calculate time in seconds since last update
    uint32_t now = hal.scheduler->micros();
	float DT = max((now - _update_speed_last_usec),0)*1.0e-6f;
	_update_speed_last_usec = now;	

    // Convert equivalent airspeeds to true airspeeds

    float EAS2TAS = _ahrs.get_EAS2TAS();
    _TAS_dem  = _EAS_dem * EAS2TAS;
	_TASmax   = aparm.airspeed_max * EAS2TAS;
	_TASmin   = aparm.airspeed_min * EAS2TAS;

    if (aparm.stall_prevention) {
        // when stall prevention is active we raise the mimimum
        // airspeed based on aerodynamic load factor
        _TASmin *= load_factor;
    }

    if (_TASmax < _TASmin) {
        _TASmax = _TASmin;
    }
    if (_landAirspeed >= 0 && _ahrs.airspeed_sensor_enabled() &&
           (_flight_stage == FLIGHT_LAND_APPROACH || _flight_stage== FLIGHT_LAND_FINAL)) {
		_TAS_dem = _landAirspeed * EAS2TAS;
		if (_TASmin > _TAS_dem) {
			_TASmin = _TAS_dem;
		}
    }

    // Reset states of time since last update is too large
    if (DT > 1.0f) {
        _integ5_state = (_EAS * EAS2TAS);
        _integ4_state = 0.0f;
		DT            = 0.1f; // when first starting TECS, use a
							  // small time constant
    }

    // Get airspeed or default to halfway between min and max if
    // airspeed is not being used and set speed rate to zero
    if (!_ahrs.airspeed_sensor_enabled() || !_ahrs.airspeed_estimate(&_EAS)) {
        // If no airspeed available use average of min and max
        _EAS = 0.5f * (aparm.airspeed_min.get() + (float)aparm.airspeed_max.get());
    }

    // Implement a second order complementary filter to obtain a
    // smoothed airspeed estimate
    // airspeed estimate is held in _integ5_state
    float aspdErr = (_EAS * EAS2TAS) - _integ5_state;
    float integ4_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;
    // Prevent state from winding up
    if (_integ5_state < 3.1f){
        integ4_input = max(integ4_input , 0.0f);
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
	
    // Constrain speed demand, taking into account the load factor
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

    // in final landing stage force height rate demand to the
    // configured sink rate and adjust the demanded height to
    // be kinematically consistent with the height rate.
	if (_flight_stage == FLIGHT_LAND_FINAL) {
        if (_flare_counter == 0) {
            _hgt_rate_dem = _climb_rate;
            _land_hgt_dem = _hgt_dem_adj;
        }
        // bring it in over 1s to prevent overshoot
        if (_flare_counter < 10) {
            _hgt_rate_dem = _hgt_rate_dem * 0.8f - 0.2f * _land_sink;
            _flare_counter++;
        } else {
            _hgt_rate_dem = - _land_sink;
        }
        _land_hgt_dem += 0.1f * _hgt_rate_dem;
        _hgt_dem_adj = _land_hgt_dem;
    } else {
        _hgt_rate_dem = (_hgt_dem_adj - _hgt_dem_adj_last) / 0.1f;
        _flare_counter = 0;
    }
    _hgt_dem_adj_last = _hgt_dem_adj;
}

void AP_TECS::_detect_underspeed(void) 
{
    if (((_integ5_state < _TASmin * 0.9f) && 
		 (_throttle_dem >= _THRmaxf * 0.95f) &&
		 _flight_stage != AP_TECS::FLIGHT_LAND_FINAL) || 
		((_integ3_state < _hgt_dem_adj) && _underspeed))
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
    _SPEdot = _climb_rate * GRAVITY_MSS;
    _SKEdot = _integ5_state * _vel_dot;

}

/*
  current time constant. It is lower in landing to try to give a precise approach
 */
float AP_TECS::timeConstant(void)
{
    if (_flight_stage==FLIGHT_LAND_FINAL ||
        _flight_stage==FLIGHT_LAND_APPROACH) {
        return _landTimeConst;
    }
    return _timeConst;
}

void AP_TECS::_update_throttle(void)
{
    // Calculate limits to be applied to potential energy error to prevent over or underspeed occurring due to large height errors
    float SPE_err_max = 0.5f * _TASmax * _TASmax - _SKE_dem;
    float SPE_err_min = 0.5f * _TASmin * _TASmin - _SKE_dem;

    // Calculate total energy error
    _STE_error = constrain_float((_SPE_dem - _SPE_est), SPE_err_min, SPE_err_max) + _SKE_dem - _SKE_est;
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
        _throttle_dem = 1.0f;
    }
    else
    {
		// Calculate gain scaler from specific energy error to throttle
		float K_STE2Thr = 1 / (timeConstant() * (_STEdot_max - _STEdot_min));

        // Calculate feed-forward throttle
        float ff_throttle = 0;
        float nomThr = aparm.throttle_cruise * 0.01f;
		const Matrix3f &rotMat = _ahrs.get_dcm_matrix();
		// Use the demanded rate of change of total energy as the feed-forward demand, but add
		// additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
		// drag increase during turns.
		float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
		STEdot_dem = STEdot_dem + _rollComp * (1.0f/constrain_float(cosPhi * cosPhi , 0.1f, 1.0f) - 1.0f);
		ff_throttle = nomThr + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);

		// Calculate PD + FF throttle
		_throttle_dem = (_STE_error + STEdot_error * _thrDamp) * K_STE2Thr + ff_throttle;

        // Constrain throttle demand
        _throttle_dem = constrain_float(_throttle_dem, _THRminf, _THRmaxf);

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
        // Set to a value that will allow 0.1 (10%) throttle saturation to allow for noise on the demand
        // Additionally constrain the integrator state amplitude so that the integrator comes off limits faster.
        float maxAmp = 0.5f*(_THRmaxf - _THRminf);
        float integ_max = constrain_float((_THRmaxf - _throttle_dem + 0.1f),-maxAmp,maxAmp);
        float integ_min = constrain_float((_THRminf - _throttle_dem - 0.1f),-maxAmp,maxAmp);

  		// Calculate integrator state, constraining state
		// Set integrator to a max throttle value during climbout
        _integ6_state = _integ6_state + (_STE_error * _integGain) * _DT * K_STE2Thr;
		if (_flight_stage == AP_TECS::FLIGHT_TAKEOFF)
		{
			_integ6_state = integ_max;
		}
		else
		{
			_integ6_state = constrain_float(_integ6_state, integ_min, integ_max);
		}

		// Sum the components. 
		// Only use feed-forward component if airspeed is not being used
	    if (_ahrs.airspeed_sensor_enabled()) {
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
    float nomThr;
    //If landing and we don't have an airspeed sensor and we have a non-zero
    //TECS_LAND_THR param then use it
    if ((_flight_stage == FLIGHT_LAND_APPROACH || _flight_stage== FLIGHT_LAND_FINAL) &&
           _landThrottle >= 0) {            
        nomThr = (_landThrottle + throttle_nudge) * 0.01f;
    } else { //not landing or not using TECS_LAND_THR parameter
		nomThr = (aparm.throttle_cruise + throttle_nudge)* 0.01f;
    }
    
	if (_flight_stage == AP_TECS::FLIGHT_TAKEOFF)
	{
		_throttle_dem = _THRmaxf;
	}
	else if (_pitch_dem > 0.0f && _PITCHmaxf > 0.0f)
	{
		_throttle_dem = nomThr + (_THRmaxf - nomThr) * _pitch_dem / _PITCHmaxf;
	}
	else if (_pitch_dem < 0.0f && _PITCHminf < 0.0f)
	{
		_throttle_dem = nomThr + (_THRminf - nomThr) * _pitch_dem / _PITCHminf;
	}
	else
	{
		_throttle_dem = nomThr;
	}

    // Calculate additional throttle for turn drag compensation including throttle nudging
	const Matrix3f &rotMat = _ahrs.get_dcm_matrix();
	// Use the demanded rate of change of total energy as the feed-forward demand, but add
	// additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
	// drag increase during turns.
	float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
	float STEdot_dem = _rollComp * (1.0f/constrain_float(cosPhi * cosPhi , 0.1f, 1.0f) - 1.0f);
	_throttle_dem = _throttle_dem + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);
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
	// A SKE_weighting of 2 provides 100% priority to speed control. This is used when an underspeed condition is detected. In this instance, if airspeed
	// rises above the demanded value, the pitch angle will be increased by the TECS controller.
	float SKE_weighting = constrain_float(_spdWeight, 0.0f, 2.0f);
    if (!_ahrs.airspeed_sensor_enabled()) {
		SKE_weighting = 0.0f;
    } else if ( _underspeed || _flight_stage == AP_TECS::FLIGHT_TAKEOFF) {
		SKE_weighting = 2.0f;
    } else if (_flight_stage == AP_TECS::FLIGHT_LAND_APPROACH || _flight_stage == AP_TECS::FLIGHT_LAND_FINAL) {
        SKE_weighting = constrain_float(_spdWeightLand, 0.0f, 2.0f);
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
    // During climbout/takeoff, bias the demanded pitch angle so that zero speed error produces a pitch angle 
    // demand equal to the minimum value (which is )set by the mission plan during this mode). Otherwise the
    // integrator has to catch up before the nose can be raised to reduce speed during climbout.
    // During flare a different damping gain is used
	float gainInv = (_integ5_state * timeConstant() * GRAVITY_MSS);
    float temp = SEB_error + SEBdot_dem * timeConstant();
    if (_flight_stage == AP_TECS::FLIGHT_LAND_FINAL) {
        temp += SEBdot_error * _landDamp;
    } else {
        temp += SEBdot_error * _ptchDamp;
    }
    if (_flight_stage == AP_TECS::FLIGHT_TAKEOFF) {
        temp += _PITCHminf * gainInv;
    }
    _integ7_state = constrain_float(_integ7_state, (gainInv * (_PITCHminf - 0.0783f)) - temp, (gainInv * (_PITCHmaxf + 0.0783f)) - temp);

    // Calculate pitch demand from specific energy balance signals
    _pitch_dem_unc = (temp + _integ7_state) / gainInv;

    // Constrain pitch demand
    _pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);

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
	if (_DT > 1.0f)
	{
		_integ6_state      = 0.0f;
		_integ7_state      = 0.0f;
		_last_throttle_dem = aparm.throttle_cruise * 0.01f;
		_last_pitch_dem    = _ahrs.pitch;
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
	else if (_flight_stage == AP_TECS::FLIGHT_TAKEOFF)
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
	// This is a trivial calculation at the moment but will get bigger once we start adding altitude effects
    _STEdot_max = _maxClimbRate * GRAVITY_MSS;
    _STEdot_min = - _minSinkRate * GRAVITY_MSS;
}

void AP_TECS::update_pitch_throttle(int32_t hgt_dem_cm,
									int32_t EAS_dem_cm, 
									enum FlightStage flight_stage,
									int32_t ptchMinCO_cd, 
									int16_t throttle_nudge,
									float hgt_afe,
                                    float load_factor) 
{
    // Calculate time in seconds since last update
    uint32_t now = hal.scheduler->micros();
	_DT = max((now - _update_pitch_throttle_last_usec),0)*1.0e-6f;
	_update_pitch_throttle_last_usec = now;	

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed(load_factor);

	// Convert inputs
    _hgt_dem = hgt_dem_cm * 0.01f;
	_EAS_dem = EAS_dem_cm * 0.01f;
    _THRmaxf  = aparm.throttle_max * 0.01f;
    _THRminf  = aparm.throttle_min * 0.01f;

	// work out the maximum and minimum pitch
	// if TECS_PITCH_{MAX,MIN} isn't set then use
	// LIM_PITCH_{MAX,MIN}. Don't allow TECS_PITCH_{MAX,MIN} to be
	// larger than LIM_PITCH_{MAX,MIN}
	if (_pitch_max <= 0) {
		_PITCHmaxf = aparm.pitch_limit_max_cd * 0.01f;
	} else {
		_PITCHmaxf = min(_pitch_max, aparm.pitch_limit_max_cd * 0.01f);
	}
	if (_pitch_min >= 0) {
		_PITCHminf = aparm.pitch_limit_min_cd * 0.01f;
	} else {
		_PITCHminf = max(_pitch_min, aparm.pitch_limit_min_cd * 0.01f);
	}
    if (flight_stage == FLIGHT_LAND_FINAL) {
        // in flare use min pitch from LAND_PITCH_CD
        _PITCHminf = max(_PITCHminf, aparm.land_pitch_cd * 0.01f);

        // and allow zero throttle
        _THRminf = 0;
    }
	// convert to radians
	_PITCHmaxf = radians(_PITCHmaxf);
	_PITCHminf = radians(_PITCHminf);
	_flight_stage = flight_stage;

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
	if (_ahrs.airspeed_sensor_enabled()) {
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
	log_tuning.dhgt     = _climb_rate;
	log_tuning.spd_dem  = _TAS_dem_adj;
	log_tuning.spd      = _integ5_state;
	log_tuning.dspd     = _vel_dot;
	log_tuning.ithr     = _integ6_state;
	log_tuning.iptch    = _integ7_state;
	log_tuning.thr      = _throttle_dem;
	log_tuning.ptch     = _pitch_dem;
	log_tuning.dspd_dem = _TAS_rate_dem;
	log_tuning.time_ms  = hal.scheduler->millis();
}

// log the contents of the log_tuning structure to dataflash
void AP_TECS::log_data(DataFlash_Class &dataflash, uint8_t msgid)
{
    log_tuning.head1 = HEAD_BYTE1;
    log_tuning.head2 = HEAD_BYTE2;
    log_tuning.msgid = msgid;
	dataflash.WriteBlock(&log_tuning, sizeof(log_tuning));
}
