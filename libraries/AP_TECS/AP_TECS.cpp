// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_TECS.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
# define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
# define Debug(fmt, args ...)
#endif
//Debug("%.2f %.2f %.2f %.2f \n", var1, var2, var3, var4);

// table of user settable parameters
const AP_Param::GroupInfo AP_TECS::var_info[] = {

    // @Param: CLMB_MAX
    // @DisplayName: Maximum Climb Rate (metres/sec)
    // @Description: This is the best climb rate that the aircraft can achieve with the throttle set to THR_MAX and the airspeed set to the default value. For electric aircraft make sure this number can be achieved towards the end of flight when the battery voltage has reduced. The setting of this parameter can be checked by commanding a positive altitude change of 100m in loiter, RTL or guided mode. If the throttle required to climb is close to THR_MAX and the aircraft is maintaining airspeed, then this parameter is set correctly. If the airspeed starts to reduce, then the parameter is set to high, and if the throttle demand require to climb and maintain speed is noticeably less than THR_MAX, then either CLMB_MAX should be increased or THR_MAX reduced.
    // @Increment: 0.1
    // @Range: 0.1 20.0
    // @User: User
    AP_GROUPINFO("CLMB_MAX",    0, AP_TECS, _maxClimbRate, 5.0f),

    // @Param: SINK_MIN
    // @DisplayName: Minimum Sink Rate (metres/sec)
    // @Description: This is the sink rate of the aircraft with the throttle set to THR_MIN and the same airspeed as used to measure CLMB_MAX.
    // @Increment: 0.1
    // @Range: 0.1 10.0
    // @User: User
    AP_GROUPINFO("SINK_MIN",    1, AP_TECS, _minSinkRate, 2.0f),

    // @Param: TIME_CONST
    // @DisplayName: Controller time constant (sec)
    // @Description: This is the time constant of the TECS control algorithm. Smaller values make it faster to respond, large values make it slower to respond.
    // @Range: 3.0 10.0
    // @Increment: 0.2
    // @User: Advanced
    AP_GROUPINFO("TIME_CONST",  2, AP_TECS, _timeConst, 5.0f),

    // @Param: THR_DAMP
    // @DisplayName: Controller throttle damping
    // @Description: This is the damping gain for the throttle demand loop. Increase to add damping  to correct for oscillations in speed and height.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("THR_DAMP",    3, AP_TECS, _thrDamp, 0.5f),

    // @Param: INTEG_GAIN
    // @DisplayName: Controller integrator
    // @Description: This is the integrator gain on the control loop. Increase to increase the rate at which speed and height offsets are trimmed out
    // @Range: 0.0 0.5
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("INTEG_GAIN", 4, AP_TECS, _integGain, 0.1f),

    // @Param: VERT_ACC
    // @DisplayName: Vertical Acceleration Limit (metres/sec^2)
    // @Description: This is the maximum vertical acceleration either up or down that the  controller will use to correct speed or height errors.
    // @Range: 1.0 10.0
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("VERT_ACC",  5, AP_TECS, _vertAccLim, 7.0f),

    // @Param: HGT_OMEGA
    // @DisplayName: Height complementary filter frequency (radians/sec)
    // @Description: This is the cross-over frequency of the complementary filter used to fuse vertical acceleration and baro alt to obtain an estimate of height rate and height.
    // @Range: 1.0 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("HGT_OMEGA", 6, AP_TECS, _hgtCompFiltOmega, 3.0f),

    // @Param: SPD_OMEGA
    // @DisplayName: Speed complementary filter frequency (radians/sec)
    // @Description: This is the cross-over frequency of the complementary filter used to fuse longitudinal acceleration and airspeed to obtain a lower noise and lag estimate of airspeed.
    // @Range: 0.5 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("SPD_OMEGA", 7, AP_TECS, _spdCompFiltOmega, 2.0f),

    // @Param: RLL2THR
    // @DisplayName: Bank angle compensation gain
    // @Description: Increasing this gain turn increases the amount of throttle that will be used to compensate for the additional drag created by turning. Ideally this should be set to approximately 10 x the extra sink rate in m/s created by a 45 degree bank turn. Increase this gain if the aircraft initially loses energy in turns and reduce if the aircraft initially gains energy in turns. Efficient high aspect-ratio aircraft (eg powered sailplanes) can use a lower value, whereas inefficient low aspect-ratio models (eg delta wings) can use a higher value.
    // @Range: 5.0 30.0
    // @Increment: 1.0
    // @User: Advanced
    AP_GROUPINFO("RLL2THR",  8, AP_TECS, _rollComp, 10.0f),

    // @Param: SPDWEIGHT
    // @DisplayName: Weighting applied to speed control
    // @Description: This parameter adjusts the amount of weighting that the pitch control applies to speed vs height errors. Setting it to 0.0 will cause the pitch control to control height and ignore speed errors. This will normally improve height accuracy but give larger airspeed errors. Setting it to 2.0 will cause the pitch control loop to control speed and ignore height errors. This will normally reduce airsped errors, but give larger height errors.	A value of 1.0 gives a balanced response and is the default.
    // @Range: 0.0 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPDWEIGHT", 9, AP_TECS, _spdWeight, 1.0f),

    // @Param: PTCH_DAMP
    // @DisplayName: Controller pitch damping
    // @Description: This is the damping gain for the pitch demand loop. Increase to add damping  to correct for oscillations in speed and height.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PTCH_DAMP", 10, AP_TECS, _ptchDamp, 0.0f),

    // @Param: SINK_MAX
    // @DisplayName: Maximum Descent Rate (metres/sec)
    // @Description: This sets the maximum descent rate that the controller will use.  If this value is too large, the aircraft will reach the pitch angle limit first and be unable to achieve the descent rate. This should be set to a value that can be achieved at the lower pitch angle limit.
    // @Increment: 0.1
    // @Range: 0.0 20.0
    // @User: User
    AP_GROUPINFO("SINK_MAX",  11, AP_TECS, _maxSinkRate, 5.0f),

    // @Param: LAND_ARSPD
    // @DisplayName: Airspeed during landing approach (m/s)
    // @Description: When performing an autonomus landing, this value is used as the goal airspeed during approach.  Note that this parameter is not useful if your platform does not have an airspeed sensor (use TECS_LAND_THR instead).  If negative then this value is not used during landing.
    // @Range: -1 127
    // @Increment: 1
    // @User: User
    AP_GROUPINFO("LAND_ARSPD", 12, AP_TECS, _landAirspeed, -1),

    // @Param: LAND_THR
    // @DisplayName: Cruise throttle during landing approach (percentage)
    // @Description: Use this parameter instead of LAND_ARSPD if your platform does not have an airspeed sensor.  It is the cruise throttle during landing approach.  If this value is negative then it is disabled and TECS_LAND_ARSPD is used instead.
    // @Range: -1 100
    // @Increment: 0.1
    // @User: User
    AP_GROUPINFO("LAND_THR", 13, AP_TECS, _landThrottle, -1),

    // @Param: LAND_SPDWGT
    // @DisplayName: Weighting applied to speed control during landing.
    // @Description: Same as SPDWEIGHT parameter, with the exception that this parameter is applied during landing flight stages.  A value closer to 2 will result in the plane ignoring height error during landing and our experience has been that the plane will therefore keep the nose up -- sometimes good for a glider landing (with the side effect that you will likely glide a ways past the landing point).  A value closer to 0 results in the plane ignoring speed error -- use caution when lowering the value below 1 -- ignoring speed could result in a stall. Values between 0 and 2 are valid values for a fixed landing weight. When using -1 the weight will be scaled during the landing. At the start of the landing approach it starts with TECS_SPDWEIGHT and scales down to 0 by the time you reach the land point. Example: Halfway down the landing approach you'll effectively have a weight of TECS_SPDWEIGHT/2.
    // @Range: -1.0 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_SPDWGT", 14, AP_TECS, _spdWeightLand, -1.0f),

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

    // @Param: LAND_PMAX
    // @DisplayName: Maximum pitch during final stage of landing
    // @Description: This limits the pitch used during the final stage of automatic landing. During the final landing stage most planes need to keep their pitch small to avoid stalling. A maximum of 10 degrees is usually good. A value of zero means to use the normal pitch limits.
    // @Range: -5 40
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LAND_PMAX", 20, AP_TECS, _land_pitch_max, 10),

    // @Param: APPR_SMAX
    // @DisplayName: Sink rate max for landing approach stage
    // @Description: The sink rate max for the landing approach stage of landing. This will need to be large for steep landing approaches especially when using reverse thrust. If 0, then use TECS_SINK_MAX.
    // @Range: 0.0 20.0
    // @Units: m/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("APPR_SMAX", 21, AP_TECS, _maxSinkRate_approach, 0),

    // @Param: LAND_SRC
    // @DisplayName: Land sink rate change
    // @Description: When zero, the flare sink rate (TECS_LAND_SINK) is a fixed sink demand. With this enabled the flare sinkrate will increase/decrease the flare sink demand as you get further beyond the LAND waypoint. Has no effect before the waypoint. This value is added to TECS_LAND_SINK proportional to distance traveled after wp. With an increasing sink rate you can still land in a given distance if you're traveling too fast and cruise passed the land point. A positive value will force the plane to land sooner proportional to distance passed land point. A negative number will tell the plane to slowly climb allowing for a pitched-up stall landing. Recommend 0.2 as initial value.
    // @Range: -2.0 2.0
    // @Units: m/s/m
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_SRC", 22, AP_TECS, _land_sink_rate_change, 0),

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
    // estimated height above field elevation  = _height
    // Reference Paper :
    // Optimising the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    /*
      if we have a vertical position estimate from the EKF then use
      it, otherwise use barometric altitude
     */
    Vector3f posned;
    if (_ahrs.get_relative_position_NED(posned)) {
        _height = - posned.z;
    } else {
        _height = _ahrs.get_baro().get_altitude();
    }

    // Calculate time in seconds since last update
    uint32_t now = AP_HAL::micros();
    float DT = MAX((now - _update_50hz_last_usec), 0U) * 1.0e-6f;
    if (DT > 1.0f) {
        _climb_rate = 0.0f;
        _height_filter.dd_height = 0.0f;
        DT            = 0.02f; // when first starting TECS, use a
        // small time constant
    }
    _update_50hz_last_usec = now;

    // Use inertial nav verical velocity and height if available
    Vector3f velned;
    if (_ahrs.get_velocity_NED(velned)) {
        // if possible use the EKF vertical velocity
        _climb_rate = -velned.z;
    } else {
        /*
          use a complimentary filter to calculate climb_rate. This is
          designed to minimise lag
         */
        float baro_alt = _ahrs.get_baro().get_altitude();
        // Get height acceleration
        float hgt_ddot_mea = -(_ahrs.get_accel_ef().z + GRAVITY_MSS);
        // Perform filter calculation using backwards Euler integration
        // Coefficients selected to place all three filter poles at omega
        float omega2 = _hgtCompFiltOmega*_hgtCompFiltOmega;
        float hgt_err = baro_alt - _height_filter.height;
        float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;

        _height_filter.dd_height += integ1_input * DT;

        float integ2_input = _height_filter.dd_height + hgt_ddot_mea + hgt_err * omega2 * 3.0f;

        _climb_rate += integ2_input * DT;

        float integ3_input = _climb_rate + hgt_err * _hgtCompFiltOmega * 3.0f;
        // If more than 1 second has elapsed since last update then reset the integrator state
        // to the measured height
        if (DT > 1.0f) {
            _height_filter.height = _height;
        } else {
            _height_filter.height += integ3_input*DT;
        }
    }

    // Update and average speed rate of change
    // Get DCM
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Calculate speed rate of change
    float temp = rotMat.c.x * GRAVITY_MSS + _ahrs.get_ins().get_accel().x;
    // take 5 point moving average
    _vel_dot = _vdot_filter.apply(temp);

}

void AP_TECS::_update_speed(float load_factor)
{
    // Calculate time in seconds since last update
    uint32_t now = AP_HAL::micros();
    float DT = MAX((now - _update_speed_last_usec), 0U) * 1.0e-6f;
    _update_speed_last_usec = now;

    // Convert equivalent airspeeds to true airspeeds

    float EAS2TAS = _ahrs.get_EAS2TAS();
    _TASmax   = aparm.airspeed_max * EAS2TAS;
    _TASmin   = aparm.airspeed_min * EAS2TAS;

    if (aparm.stall_prevention) {
        // when stall prevention is active we raise the mimimum
        // airspeed based on aerodynamic load factor
        _TASmin *= load_factor;
    }

    float demanded_airspeed = _EAS_dem;
    if (_ahrs.airspeed_sensor_enabled()) {
        if ((_flight_stage == FLIGHT_LAND_APPROACH || _flight_stage == FLIGHT_LAND_FINAL) && _landAirspeed >= 0) {
            demanded_airspeed = _landAirspeed;
        } else if (_flight_stage == FLIGHT_LAND_PREFLARE) {
            if (aparm.land_pre_flare_airspeed > 0) {
                demanded_airspeed = aparm.land_pre_flare_airspeed;
            } else if (_landAirspeed >= 0) {
                demanded_airspeed = _landAirspeed;
            }
        }
    }
    _TAS_dem = demanded_airspeed * EAS2TAS;
    if (_TASmax < _TASmin) {
        _TASmax = _TASmin;
    }
    if (_TASmin > _TAS_dem) {
        _TASmin = _TAS_dem;
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
    if (_integ5_state < 3.1f) {
        integ4_input = MAX(integ4_input , 0.0f);
    }
    _integ4_state = _integ4_state + integ4_input * DT;
    float integ5_input = _integ4_state + _vel_dot + aspdErr * _spdCompFiltOmega * 1.4142f;
    _integ5_state = _integ5_state + integ5_input * DT;
    // limit the airspeed to a minimum of 3 m/s
    _integ5_state = MAX(_integ5_state, 3.0f);

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
    _hgt_dem = 0.5f * (_hgt_dem + _hgt_dem_in_old);
    _hgt_dem_in_old = _hgt_dem;

    float max_sink_rate = _maxSinkRate;
    if (_maxSinkRate_approach > 0 && is_on_land_approach(true)) {
        // special sink rate for approach to accommodate steep slopes and reverse thrust.
        // A special check must be done to see if we're LANDing on approach but also if
        // we're in that tiny window just starting NAV_LAND but still in NORMAL mode. If
        // we have a steep slope with a short approach we'll want to allow acquiring the
        // glide slope right away.
        max_sink_rate = _maxSinkRate_approach;
    }

    // Limit height rate of change
    if ((_hgt_dem - _hgt_dem_prev) > (_maxClimbRate * 0.1f))
    {
        _hgt_dem = _hgt_dem_prev + _maxClimbRate * 0.1f;
    }
    else if ((_hgt_dem - _hgt_dem_prev) < (-max_sink_rate * 0.1f))
    {
        _hgt_dem = _hgt_dem_prev - max_sink_rate * 0.1f;
    }
    _hgt_dem_prev = _hgt_dem;

    // Apply first order lag to height demand
    _hgt_dem_adj = 0.05f * _hgt_dem + 0.95f * _hgt_dem_adj_last;

    // in final landing stage force height rate demand to the
    // configured sink rate and adjust the demanded height to
    // be kinematically consistent with the height rate.
    if (_flight_stage == FLIGHT_LAND_FINAL) {
        _integ7_state = 0;
        if (_flare_counter == 0) {
            _hgt_rate_dem = _climb_rate;
            _land_hgt_dem = _hgt_dem_adj;
        }

        // adjust the flare sink rate to increase/decrease as your travel further beyond the land wp
        float land_sink_rate_adj = _land_sink + _land_sink_rate_change*_distance_beyond_land_wp;

        // bring it in over 1s to prevent overshoot
        if (_flare_counter < 10) {
            _hgt_rate_dem = _hgt_rate_dem * 0.8f - 0.2f * land_sink_rate_adj;
            _flare_counter++;
        } else {
            _hgt_rate_dem = - land_sink_rate_adj;
        }
        _land_hgt_dem += 0.1f * _hgt_rate_dem;
        _hgt_dem_adj = _land_hgt_dem;
    } else {
        _hgt_rate_dem = (_hgt_dem_adj - _hgt_dem_adj_last) / 0.1f;
        _flare_counter = 0;
    }

    // for landing approach we will predict ahead by the time constant
    // plus the lag produced by the first order filter. This avoids a
    // lagged height demand while constantly descending which causes
    // us to consistently be above the desired glide slope. This will
    // be replaced with a better zero-lag filter in the future.
    float new_hgt_dem = _hgt_dem_adj;
    if (_flight_stage == FLIGHT_LAND_APPROACH || _flight_stage == FLIGHT_LAND_FINAL || _flight_stage == FLIGHT_LAND_PREFLARE) {
        new_hgt_dem += (_hgt_dem_adj - _hgt_dem_adj_last)*10.0f*(timeConstant()+1);
    }
    _hgt_dem_adj_last = _hgt_dem_adj;
    _hgt_dem_adj = new_hgt_dem;
}

void AP_TECS::_detect_underspeed(void)
{
    if (_flight_stage == AP_TECS::FLIGHT_VTOL) {
        _underspeed = false;
    } else if (((_integ5_state < _TASmin * 0.9f) &&
            (_throttle_dem >= _THRmaxf * 0.95f) &&
            _flight_stage != AP_TECS::FLIGHT_LAND_FINAL) ||
            ((_height < _hgt_dem_adj) && _underspeed))
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
    _SPE_est = _height * GRAVITY_MSS;
    _SKE_est = 0.5f * _integ5_state * _integ5_state;

    // Calculate specific energy rate
    _SPEdot = _climb_rate * GRAVITY_MSS;
    _SKEdot = _integ5_state * _vel_dot;

}

/*
  current time constant. It is lower in landing to try to give a precise approach
 */
float AP_TECS::timeConstant(void) const
{
    if (_flight_stage==FLIGHT_LAND_FINAL ||
            _flight_stage==FLIGHT_LAND_PREFLARE ||
            _flight_stage==FLIGHT_LAND_APPROACH) {
        if (_landTimeConst < 0.1f) {
            return 0.1f;
        }
        return _landTimeConst;
    }
    if (_timeConst < 0.1f) {
        return 0.1f;
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
        const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
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
        if (_flight_stage == AP_TECS::FLIGHT_TAKEOFF || _flight_stage == AP_TECS::FLIGHT_LAND_ABORT)
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
    if ((_flight_stage == FLIGHT_LAND_APPROACH || _flight_stage == FLIGHT_LAND_FINAL || _flight_stage == FLIGHT_LAND_PREFLARE) &&
            _landThrottle >= 0) {
        nomThr = (_landThrottle + throttle_nudge) * 0.01f;
    } else { //not landing or not using TECS_LAND_THR parameter
        nomThr = (aparm.throttle_cruise + throttle_nudge)* 0.01f;
    }

    if (_pitch_dem > 0.0f && _PITCHmaxf > 0.0f)
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
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
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
    } else if ( _underspeed || _flight_stage == AP_TECS::FLIGHT_TAKEOFF || _flight_stage == AP_TECS::FLIGHT_LAND_ABORT) {
        SKE_weighting = 2.0f;
    } else if (_flight_stage == AP_TECS::FLIGHT_LAND_APPROACH ||
            _flight_stage == AP_TECS::FLIGHT_LAND_PREFLARE ||
            _flight_stage == AP_TECS::FLIGHT_LAND_FINAL) {
        if (_spdWeightLand < 0) {
            // use sliding scale from normal weight down to zero at landing
            float scaled_weight = _spdWeight * (1.0f - _path_proportion);
            SKE_weighting = constrain_float(scaled_weight, 0.0f, 2.0f);
        } else {
            SKE_weighting = constrain_float(_spdWeightLand, 0.0f, 2.0f);
        }
    }

    float SPE_weighting = 2.0f - SKE_weighting;

    // Calculate Specific Energy Balance demand, and error
    float SEB_dem      = _SPE_dem * SPE_weighting - _SKE_dem * SKE_weighting;
    float SEBdot_dem   = _SPEdot_dem * SPE_weighting - _SKEdot_dem * SKE_weighting;
    float SEB_error    = SEB_dem - (_SPE_est * SPE_weighting - _SKE_est * SKE_weighting);
    float SEBdot_error = SEBdot_dem - (_SPEdot * SPE_weighting - _SKEdot * SKE_weighting);

    // Calculate integrator state, constraining input if pitch limits are exceeded
    float integ7_input = SEB_error * _integGain;
    if (_pitch_dem > _PITCHmaxf)
    {
        integ7_input = MIN(integ7_input, _PITCHmaxf - _pitch_dem);
    }
    else if (_pitch_dem < _PITCHminf)
    {
        integ7_input = MAX(integ7_input, _PITCHminf - _pitch_dem);
    }
    _integ7_state = _integ7_state + integ7_input * _DT;

#if 0
    if (_flight_stage == FLIGHT_LAND_FINAL && fabsf(_climb_rate) > 0.2f) {
        ::printf("_hgt_rate_dem=%.1f _hgt_dem_adj=%.1f climb=%.1f _flare_counter=%u _pitch_dem=%.1f SEB_dem=%.2f SEBdot_dem=%.2f SEB_error=%.2f SEBdot_error=%.2f\n",
                 _hgt_rate_dem, _hgt_dem_adj, _climb_rate, _flare_counter, degrees(_pitch_dem),
                 SEB_dem, SEBdot_dem, SEB_error, SEBdot_error);
    }
#endif


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
    if (_flight_stage == AP_TECS::FLIGHT_TAKEOFF || _flight_stage == AP_TECS::FLIGHT_LAND_ABORT) {
        temp += _PITCHminf * gainInv;
    }
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

    // re-constrain pitch demand
    _pitch_dem = constrain_float(_pitch_dem, _PITCHminf, _PITCHmaxf);

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
    else if (_flight_stage == AP_TECS::FLIGHT_TAKEOFF || _flight_stage == AP_TECS::FLIGHT_LAND_ABORT)
    {
        _PITCHminf          = 0.000174533f * ptchMinCO_cd;
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

// return true if on landing approach or pre-flare approach flight stages. Optional
// argument allows to be true while in normal stage just before switching to approach
bool AP_TECS::is_on_land_approach(bool include_segment_between_NORMAL_and_APPROACH)
{
    if (!_is_doing_auto_land) {
        return false;
    }

    bool on_land_approach = false;

    on_land_approach |= (_flight_stage == AP_TECS::FLIGHT_LAND_APPROACH);
    on_land_approach |= (_flight_stage == AP_TECS::FLIGHT_LAND_PREFLARE);

    if (include_segment_between_NORMAL_and_APPROACH) {
        // include the brief time where we are performing NAV_LAND but still
        // on NORMAL stage until we line up with the approach slope
        on_land_approach |= (_flight_stage == AP_TECS::FLIGHT_NORMAL);
    }

    return on_land_approach;
}

void AP_TECS::update_pitch_throttle(int32_t hgt_dem_cm,
                                    int32_t EAS_dem_cm,
                                    enum FlightStage flight_stage,
                                    bool is_doing_auto_land,
                                    float distance_beyond_land_wp,
                                    int32_t ptchMinCO_cd,
                                    int16_t throttle_nudge,
                                    float hgt_afe,
                                    float load_factor)
{
    // Calculate time in seconds since last update
    uint32_t now = AP_HAL::micros();
    _DT = MAX((now - _update_pitch_throttle_last_usec), 0U) * 1.0e-6f;
    _update_pitch_throttle_last_usec = now;

    _is_doing_auto_land = is_doing_auto_land;
    _distance_beyond_land_wp = distance_beyond_land_wp;

    // Convert inputs
    _hgt_dem = hgt_dem_cm * 0.01f;
    _EAS_dem = EAS_dem_cm * 0.01f;

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed(load_factor);

    if (aparm.takeoff_throttle_max != 0 &&
            (_flight_stage == AP_TECS::FLIGHT_TAKEOFF || _flight_stage == AP_TECS::FLIGHT_LAND_ABORT)) {
        _THRmaxf  = aparm.takeoff_throttle_max * 0.01f;
    } else {
        _THRmaxf  = aparm.throttle_max * 0.01f;
    }
    _THRminf  = aparm.throttle_min * 0.01f;

    // work out the maximum and minimum pitch
    // if TECS_PITCH_{MAX,MIN} isn't set then use
    // LIM_PITCH_{MAX,MIN}. Don't allow TECS_PITCH_{MAX,MIN} to be
    // larger than LIM_PITCH_{MAX,MIN}
    if (_pitch_max <= 0) {
        _PITCHmaxf = aparm.pitch_limit_max_cd * 0.01f;
    } else {
        _PITCHmaxf = MIN(_pitch_max, aparm.pitch_limit_max_cd * 0.01f);
    }

    // apply temporary pitch limit and clear
    _PITCHmaxf = constrain_float(_PITCHmaxf, -90, _pitch_max_limit);
    _pitch_max_limit = 90;
    
    if (_pitch_min >= 0) {
        _PITCHminf = aparm.pitch_limit_min_cd * 0.01f;
    } else {
        _PITCHminf = MAX(_pitch_min, aparm.pitch_limit_min_cd * 0.01f);
    }
    if (flight_stage == FLIGHT_LAND_FINAL) {
        // in flare use min pitch from LAND_PITCH_CD
        _PITCHminf = MAX(_PITCHminf, aparm.land_pitch_cd * 0.01f);

        // and use max pitch from TECS_LAND_PMAX
        if (_land_pitch_max != 0) {
            _PITCHmaxf = MIN(_PITCHmaxf, _land_pitch_max);
        }

        // and allow zero throttle
        _THRminf = 0;
    } else if ((flight_stage == FLIGHT_LAND_APPROACH || flight_stage == FLIGHT_LAND_PREFLARE) && (-_climb_rate) > _land_sink) {
        // constrain the pitch in landing as we get close to the flare
        // point. Use a simple linear limit from 15 meters after the
        // landing point
        float time_to_flare = (- hgt_afe / _climb_rate) - aparm.land_flare_sec;
        if (time_to_flare < 0) {
            // we should be flaring already
            _PITCHminf = MAX(_PITCHminf, aparm.land_pitch_cd * 0.01f);
        } else if (time_to_flare < timeConstant()*2) {
            // smoothly move the min pitch to the flare min pitch over
            // twice the time constant
            float p = time_to_flare/(2*timeConstant());
            float pitch_limit_cd = p*aparm.pitch_limit_min_cd + (1-p)*aparm.land_pitch_cd;
#if 0
            ::printf("ttf=%.1f hgt_afe=%.1f _PITCHminf=%.1f pitch_limit=%.1f climb=%.1f\n",
                     time_to_flare, hgt_afe, _PITCHminf, pitch_limit_cd*0.01f, _climb_rate);
#endif
            _PITCHminf = MAX(_PITCHminf, pitch_limit_cd*0.01f);
        }
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
    log_tuning.hgt      = _height;
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
    log_tuning.time_us  = AP_HAL::micros64();
}

// log the contents of the log_tuning structure to dataflash
void AP_TECS::log_data(DataFlash_Class &dataflash, uint8_t msgid)
{
    log_tuning.head1 = HEAD_BYTE1;
    log_tuning.head2 = HEAD_BYTE2;
    log_tuning.msgid = msgid;
    dataflash.WriteBlock(&log_tuning, sizeof(log_tuning));
}
