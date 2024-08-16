#include "AP_TECS.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Landing/AP_Landing.h>

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
    // @Description: Maximum demanded climb rate. Do not set higher than the climb speed at THR_MAX at AIRSPEED_CRUISE when the battery is at low voltage. Reduce value if airspeed cannot be maintained on ascent. Increase value if throttle does not increase significantly to ascend.
    // @Increment: 0.1
    // @Range: 0.1 20.0
    // @User: Standard
    AP_GROUPINFO("CLMB_MAX",    0, AP_TECS, _maxClimbRate, 5.0f),

    // @Param: SINK_MIN
    // @DisplayName: Minimum Sink Rate (metres/sec)
    // @Description: Minimum sink rate when at THR_MIN and AIRSPEED_CRUISE.
    // @Increment: 0.1
    // @Range: 0.1 10.0
    // @User: Standard
    AP_GROUPINFO("SINK_MIN",    1, AP_TECS, _minSinkRate, 2.0f),

    // @Param: TIME_CONST
    // @DisplayName: Controller time constant (sec)
    // @Description: Time constant of the TECS control algorithm. Small values make faster altitude corrections but can cause overshoot and aggressive behavior.
    // @Range: 3.0 10.0
    // @Increment: 0.2
    // @User: Advanced
    AP_GROUPINFO("TIME_CONST",  2, AP_TECS, _timeConst, 5.0f),

    // @Param: THR_DAMP
    // @DisplayName: Controller throttle damping
    // @Description: Damping gain for throttle demand loop. Increase to add throttle activity to dampen oscillations in speed and height.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("THR_DAMP",    3, AP_TECS, _thrDamp, 0.5f),

    // @Param: INTEG_GAIN
    // @DisplayName: Controller integrator
    // @Description: Integrator gain to trim out long-term speed and height errors.
    // @Range: 0.0 0.5
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("INTEG_GAIN", 4, AP_TECS, _integGain, 0.3f),

    // @Param: VERT_ACC
    // @DisplayName: Vertical Acceleration Limit (metres/sec^2)
    // @Description: Maximum vertical acceleration used to correct speed or height errors.
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
    // @Description: Gain from bank angle to throttle to compensate for loss of airspeed from drag in turns. Set to approximately 10x the sink rate in m/s caused by a 45-degree turn. High efficiency models may need less while less efficient aircraft may need more. Should be tuned in an automatic mission with waypoints and turns greater than 90 degrees. Tune with PTCH2SRV_RLL and KFF_RDDRMIX to achieve constant airspeed, constant altitude turns.
    // @Range: 5.0 30.0
    // @Increment: 1.0
    // @User: Advanced
    AP_GROUPINFO("RLL2THR",  8, AP_TECS, _rollComp, 10.0f),

    // @Param: SPDWEIGHT
    // @DisplayName: Weighting applied to speed control
    // @Description: Mixing of pitch and throttle correction for height and airspeed errors. Pitch controls altitude and throttle controls airspeed if set to 0. Pitch controls airspeed and throttle controls altitude if set to 2 (good for gliders). Blended if set to 1.
    // @Range: 0.0 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPDWEIGHT", 9, AP_TECS, _spdWeight, 1.0f),

    // @Param: PTCH_DAMP
    // @DisplayName: Controller pitch damping
    // @Description: Damping gain for pitch control from TECS control.  Increasing may correct for oscillations in speed and height, but too much may cause additional oscillation and degraded control.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PTCH_DAMP", 10, AP_TECS, _ptchDamp, 0.3f),

    // @Param: SINK_MAX
    // @DisplayName: Maximum Descent Rate (metres/sec)
    // @Description: Maximum demanded descent rate. Do not set higher than the vertical speed the aircraft can maintain at THR_MIN, TECS_PITCH_MIN, and AIRSPEED_MAX.
    // @Increment: 0.1
    // @Range: 0.0 20.0
    // @User: Standard
    AP_GROUPINFO("SINK_MAX",  11, AP_TECS, _maxSinkRate, 5.0f),

    // @Param: LAND_ARSPD
    // @DisplayName: Airspeed during landing approach (m/s)
    // @Description: When performing an autonomus landing, this value is used as the goal airspeed during approach.  Max airspeed allowed is Trim Airspeed or AIRSPEED_MAX as defined by LAND_OPTIONS bitmask.  Note that this parameter is not useful if your platform does not have an airspeed sensor (use TECS_LAND_THR instead).  If negative then this value is halfway between AIRSPEED_MIN and TRIM_CRUISE_CM speed for fixed wing autolandings.
    // @Range: -1 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LAND_ARSPD", 12, AP_TECS, _landAirspeed, -1),

    // @Param: LAND_THR
    // @DisplayName: Cruise throttle during landing approach (percentage)
    // @Description: Use this parameter instead of LAND_ARSPD if your platform does not have an airspeed sensor.  It is the cruise throttle during landing approach.  If this value is negative then it is disabled and TECS_LAND_ARSPD is used instead.
    // @Range: -1 100
    // @Increment: 0.1
    // @User: Standard
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
    // @Description: Overrides PTCH_LIM_MAX_DEG in automatic throttle modes to reduce climb rates. Uses PTCH_LIM_MAX_DEG if set to 0. For proper TECS tuning, set to the angle that the aircraft can climb at AIRSPEED_CRUISE and THR_MAX.
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PITCH_MAX", 15, AP_TECS, _pitch_max, 15),

    // @Param: PITCH_MIN
    // @DisplayName: Minimum pitch in auto flight
    // @Description: Overrides PTCH_LIM_MIN_DEG in automatic throttle modes to reduce descent rates. Uses PTCH_LIM_MIN_DEG if set to 0. For proper TECS tuning, set to the angle that the aircraft can descend at without overspeeding.
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

    // @Param: LAND_TDAMP
    // @DisplayName: Controller throttle damping when landing
    // @Description: Damping gain for the throttle demand loop during an auto-landing. Same as TECS_THR_DAMP but only in effect during an auto-land. Increase to add throttle activity to dampen oscillations in speed and height. When set to 0 landing throttle damping is controlled by TECS_THR_DAMP.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_TDAMP", 23, AP_TECS, _land_throttle_damp, 0),

    // @Param: LAND_IGAIN
    // @DisplayName: Controller integrator during landing
    // @Description: This is the integrator gain on the control loop during landing. When set to 0 then TECS_INTEG_GAIN is used. Increase to increase the rate at which speed and height offsets are trimmed out. Typically values lower than TECS_INTEG_GAIN work best
    // @Range: 0.0 0.5
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("LAND_IGAIN", 24, AP_TECS, _integGain_land, 0),

    // @Param: TKOFF_IGAIN
    // @DisplayName: Controller integrator during takeoff
    // @Description: This is the integrator gain on the control loop during takeoff. Increase to increase the rate at which speed and height offsets are trimmed out.
    // @Range: 0.0 0.5
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("TKOFF_IGAIN", 25, AP_TECS, _integGain_takeoff, 0),

    // @Param: LAND_PDAMP
    // @DisplayName: Pitch damping gain when landing
    // @Description: This is the damping gain for the pitch demand loop during landing. Increase to add damping  to correct for oscillations in speed and height. If set to 0 then TECS_PTCH_DAMP will be used instead.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LAND_PDAMP", 26, AP_TECS, _land_pitch_damp, 0),

    // @Param: SYNAIRSPEED
    // @DisplayName: Enable the use of synthetic airspeed
    // @Description: This enables the use of synthetic airspeed in TECS for aircraft that don't have a real airspeed sensor. This is useful for development testing where the user is aware of the considerable limitations of the synthetic airspeed system, such as very poor estimates when a wind estimate is not accurate. Do not enable this option unless you fully understand the limitations of a synthetic airspeed estimate. This option has no effect if a healthy airspeed sensor is being used for airspeed measurements.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("SYNAIRSPEED", 27, AP_TECS, _use_synthetic_airspeed, 0),

    // @Param: OPTIONS
    // @DisplayName: Extra TECS options
    // @Description: This allows the enabling of special features in the speed/height controller.
    // @Bitmask: 0:GliderOnly,1:AllowDescentSpeedup
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 28, AP_TECS, _options, 0),

    // @Param: PTCH_FF_V0
    // @DisplayName: Baseline airspeed for pitch feed-forward.
    // @Description: This parameter sets the airspeed at which no feed-forward is applied between demanded airspeed and pitch. It should correspond to the airspeed in metres per second at which the plane glides at neutral pitch including STAB_PITCH_DOWN.
    // @Range: 5.0 50.0
    // @User: Advanced
    AP_GROUPINFO("PTCH_FF_V0", 29, AP_TECS, _pitch_ff_v0, 12.0),

    // @Param: PTCH_FF_K
    // @DisplayName: Gain for pitch feed-forward.
    // @Description: This parameter sets the gain between demanded airspeed and pitch. It has units of radians per metre per second and should generally be negative. A good starting value is -0.04 for gliders and -0.08 for draggy airframes. The default (0.0) disables this feed-forward.
    // @Range: -5.0 0.0
    // @User: Advanced
    AP_GROUPINFO("PTCH_FF_K", 30, AP_TECS, _pitch_ff_k, 0.0),

    // 31 previously used by TECS_LAND_PTRIM

    // @Param: FLARE_HGT
    // @DisplayName: Flare holdoff height
    // @Description: When height above ground is below this, the sink rate will be held at TECS_LAND_SINK. Use this to perform a hold-off manoeuvre when combined with small values for TECS_LAND_SINK.
    // @Range: 0 15
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("FLARE_HGT", 32, AP_TECS, _flare_holdoff_hgt, 1.0f),

    // @Param: HDEM_TCONST
    // @DisplayName: Height Demand Time Constant
    // @Description: This sets the time constant of the low pass filter that is applied to the height demand input when bit 1 of TECS_OPTIONS is not selected.
    // @Range: 1.0 5.0
    // @Units: s
    // @Increment: 0.2
    // @User: Advanced
    AP_GROUPINFO("HDEM_TCONST", 33, AP_TECS, _hgt_dem_tconst, 3.0f),

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

void AP_TECS::update_50hz(void)
{
    // Implement third order complementary filter for height and height rate
    // estimated height rate = _climb_rate
    // estimated height above field elevation  = _height
    // Reference Paper :
    // Optimizing the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    /*
      if we have a vertical position estimate from the EKF then use
      it, otherwise use barometric altitude
     */
    _ahrs.get_relative_position_D_home(_height);
    _height *= -1.0f;

    // Calculate time in seconds since last update
    uint64_t now = AP_HAL::micros64();
    float DT = (now - _update_50hz_last_usec) * 1.0e-6f;
    _flags.reset = DT > 1.0f;
    if (_flags.reset) {
        _climb_rate = 0.0f;
        _height_filter.dd_height = 0.0f;
        DT = 0.02f; // when first starting TECS, use most likely time constant
        _vdot_filter.reset();
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
        const float baro_alt = AP::baro().get_altitude();
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
        if (_flags.reset) {
            _height_filter.height = _height;
        } else {
            _height_filter.height += integ3_input*DT;
        }
    }

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed(DT);
}

void AP_TECS::_update_speed(float DT)
{
    // Update and average speed rate of change

    // calculate a low pass filtered _vel_dot
    if (_flags.reset) {
        _vdot_filter.reset();
        _vel_dot_lpf = _vel_dot;
    } else {
        // Get DCM
        const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
        // Calculate speed rate of change
        float temp = rotMat.c.x * GRAVITY_MSS + AP::ins().get_accel().x;
        // take 5 point moving average
        _vel_dot = _vdot_filter.apply(temp);
        const float alpha = DT / (DT + timeConstant());
        _vel_dot_lpf = _vel_dot_lpf * (1.0f - alpha) + _vel_dot * alpha;
    }

    bool use_airspeed = _use_synthetic_airspeed_once || _use_synthetic_airspeed.get() || _ahrs.using_airspeed_sensor();

    // Convert equivalent airspeeds to true airspeeds and harmonise limits

    float EAS2TAS = _ahrs.get_EAS2TAS();
    _TAS_dem = _EAS_dem * EAS2TAS;
    if (_flags.reset || !use_airspeed) {
        _TASmax = aparm.airspeed_max * EAS2TAS;
    } else if (_thr_clip_status == clipStatus::MAX) {
        // wind down airspeed upper limit  to prevent a situation where the aircraft can't climb
        // at the maximum speed
        const float velRateMin = 0.5f * _STEdot_min / MAX(_TAS_state, aparm.airspeed_min * EAS2TAS);
        _TASmax += _DT * velRateMin;
        _TASmax = MAX(_TASmax, aparm.airspeed_cruise * EAS2TAS);
    } else {
        // wind airspeed upper limit back to parameter defined value
        const float velRateMax = 0.5f * _STEdot_max / MAX(_TAS_state, aparm.airspeed_min * EAS2TAS);
        _TASmax += _DT * velRateMax;
    }
    _TASmax   = MIN(_TASmax, aparm.airspeed_max * EAS2TAS);
    _TASmin   = aparm.airspeed_min * EAS2TAS;

    if (aparm.stall_prevention) {
        // when stall prevention is active we raise the minimum
        // airspeed based on aerodynamic load factor
        if (is_positive(aparm.airspeed_stall)) {
            _TASmin = MAX(_TASmin, aparm.airspeed_stall*EAS2TAS*_load_factor);
        } else {
            _TASmin *= _load_factor;
        }
    }

    if (_TASmax < _TASmin) {
        _TASmax = _TASmin;
    }

    // Get measured airspeed or default to trim speed and constrain to range between min and max if
    // airspeed sensor data cannot be used
    if (!use_airspeed || !_ahrs.airspeed_estimate(_EAS)) {
        // If no airspeed available use average of min and max
        _EAS = constrain_float(aparm.airspeed_cruise.get(), (float)aparm.airspeed_min.get(), (float)aparm.airspeed_max.get());
    }

    // limit the airspeed to a minimum of 3 m/s
    const float min_airspeed = 3.0;

    // Reset states of time since last update is too large
    if (_flags.reset) {
        _TAS_state = (_EAS * EAS2TAS);
        _TAS_state = MAX(_TAS_state, min_airspeed);
        _integDTAS_state = 0.0f;
        return;
    }

    // Implement a second order complementary filter to obtain a
    // smoothed airspeed estimate
    // airspeed estimate is held in _TAS_state
    float aspdErr = (_EAS * EAS2TAS) - _TAS_state;
    float integDTAS_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;
    // Prevent state from winding up
    if (_TAS_state < 3.1f) {
        integDTAS_input = MAX(integDTAS_input, 0.0f);
    }
    _integDTAS_state = _integDTAS_state + integDTAS_input * DT;
    float TAS_input = _integDTAS_state + _vel_dot + aspdErr * _spdCompFiltOmega * 1.4142f;
    _TAS_state = _TAS_state + TAS_input * DT;
    _TAS_state = MAX(_TAS_state, min_airspeed);

}

void AP_TECS::_update_speed_demand(void)
{
    if (_options & OPTION_DESCENT_SPEEDUP) {
        // Allow demanded speed to  go to maximum when descending at maximum descent rate
        _TAS_dem = _TAS_dem + (_TASmax - _TAS_dem) * _sink_fraction;
    }

    // Set the airspeed demand to the minimum value if an underspeed condition exists
    // or a bad descent condition exists
    // This will minimise the rate of descent resulting from an engine failure,
    // enable the maximum climb rate to be achieved and prevent continued full power descent
    // into the ground due to an unachievable airspeed value
    if ((_flags.badDescent) || (_flags.underspeed)) {
        _TAS_dem     = _TASmin;
    }

    // Constrain speed demand, taking into account the load factor
    _TAS_dem = constrain_float(_TAS_dem, _TASmin, _TASmax);

    // Determine the true cruising airspeed (m/s)
    const float TAScruise = aparm.airspeed_cruise * _ahrs.get_EAS2TAS();

    // calculate velocity rate limits based on physical performance limits
    // provision to use a different rate limit if bad descent or underspeed condition exists
    // Use 50% of maximum energy rate on gain, 90% on dissipation to allow margin for total energy controller
    const float velRateMax = 0.5f * _STEdot_max / _TAS_state;
    // Maximum permissible rate of deceleration value at max airspeed
    const float velRateNegMax = 0.9f * _STEdot_neg_max / _TASmax;
    // Maximum permissible rate of deceleration value at cruise speed
    const float velRateNegCruise = 0.9f * _STEdot_min / TAScruise;
    // Linear interpolation between velocity rate at cruise and max speeds, capped at those speeds
    const float velRateMin = linear_interpolate(velRateNegMax, velRateNegCruise, _TAS_state, _TASmax, TAScruise);
    const float TAS_dem_previous = _TAS_dem_adj;

    // Apply rate limit
    if ((_TAS_dem - TAS_dem_previous) > (velRateMax * _DT)) {
        _TAS_dem_adj = TAS_dem_previous + velRateMax * _DT;
        _TAS_rate_dem = velRateMax;
    } else if ((_TAS_dem - TAS_dem_previous) < (velRateMin * _DT)) {
        _TAS_dem_adj = TAS_dem_previous + velRateMin * _DT;
        _TAS_rate_dem = velRateMin;
    } else {
        _TAS_rate_dem = (_TAS_dem - TAS_dem_previous) / _DT;
        _TAS_dem_adj = _TAS_dem;
    }

    // calculate a low pass filtered _TAS_rate_dem
    if (_flags.reset) {
        _TAS_dem_adj = _TAS_state;
        _TAS_rate_dem_lpf = _TAS_rate_dem;
    } else {
        const float alpha = _DT / (_DT + timeConstant());
        _TAS_rate_dem_lpf = _TAS_rate_dem_lpf * (1.0f - alpha) + _TAS_rate_dem * alpha;
    }

    // Constrain speed demand again to protect against bad values on initialisation.
    _TAS_dem_adj = constrain_float(_TAS_dem_adj, _TASmin, _TASmax);
}

void AP_TECS::_update_height_demand(void)
{
    _climb_rate_limit = _maxClimbRate * _max_climb_scaler;
    _sink_rate_limit = _maxSinkRate * _max_sink_scaler;
    if (_maxSinkRate_approach > 0 && _flags.is_doing_auto_land) {
        // special sink rate for approach to accommodate steep slopes and reverse thrust.
        // A special check must be done to see if we're LANDing on approach but also if
        // we're in that tiny window just starting NAV_LAND but still in NORMAL mode. If
        // we have a steep slope with a short approach we'll want to allow acquiring the
        // glide slope right away.
        _sink_rate_limit = _maxSinkRate_approach;
    }


    if (!_landing.is_flaring()) {
        // Apply 2 point moving average to demanded height
        const float hgt_dem = 0.5f * (_hgt_dem_in + _hgt_dem_in_prev);
        _hgt_dem_in_prev = _hgt_dem_in;

        // Limit height rate of change
        if ((hgt_dem - _hgt_dem_rate_ltd) > (_climb_rate_limit * _DT)) {
            _hgt_dem_rate_ltd = _hgt_dem_rate_ltd + _climb_rate_limit * _DT;
            _sink_fraction = 0.0f;
        } else if ((hgt_dem - _hgt_dem_rate_ltd) < (-_sink_rate_limit * _DT)) {
            _hgt_dem_rate_ltd = _hgt_dem_rate_ltd - _sink_rate_limit * _DT;
            _sink_fraction = 1.0f;
        } else {
            const float numerator = hgt_dem - _hgt_dem_rate_ltd;
            const float denominator = - _sink_rate_limit * _DT;
            if (is_negative(numerator) && is_negative(denominator)) {
                _sink_fraction = numerator / denominator;
            } else {
                _sink_fraction = 0.0f;
            }
            _hgt_dem_rate_ltd = hgt_dem;
        }

        // Apply a first order lag to height demand and compensate for lag when commencing height
        // control after takeoff to prevent plane pushing nose to level before climbing again. Post takeoff
        // compensation offset is decayed using the same time constant as the height demand filter.
        const float coef = MIN(_DT / (_DT + MAX(_hgt_dem_tconst, _DT)), 1.0f);
        _hgt_rate_dem = (_hgt_dem_rate_ltd - _hgt_dem_lpf) / _hgt_dem_tconst;
        _hgt_dem_lpf = _hgt_dem_rate_ltd * coef + (1.0f - coef) * _hgt_dem_lpf;
        _post_TO_hgt_offset *= (1.0f - coef);
        _hgt_dem = _hgt_dem_lpf + _post_TO_hgt_offset;

        // during approach compensate for height filter lag
        if (_flags.is_doing_auto_land) {
            _hgt_dem += _hgt_dem_tconst * _hgt_rate_dem;
        } else {
            // Don't allow height demand to get too far ahead of the vehicles current height
                // if vehicle is unable to follow the demanded climb or descent
            bool max_climb_condition   = (_pitch_dem_unc > _PITCHmaxf) ||
                                            (_SEBdot_dem_clip == clipStatus::MAX);
            bool max_descent_condition = (_pitch_dem_unc < _PITCHminf) ||
                                            (_SEBdot_dem_clip == clipStatus::MIN);
            if (_using_airspeed_for_throttle) {
                // large height errors will result in the throttle saturating
                max_climb_condition   |= (_thr_clip_status == clipStatus::MAX) &&
                                            !((_flight_stage == AP_FixedWing::FlightStage::TAKEOFF) || (_flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING));
                max_descent_condition |= (_thr_clip_status == clipStatus::MIN) && !_landing.is_flaring();
            }
            const float hgt_dem_alpha = _DT / MAX(_DT + _hgt_dem_tconst, _DT);
            if (max_climb_condition && _hgt_dem > _hgt_dem_prev) {
                    _max_climb_scaler *= (1.0f - hgt_dem_alpha);
            } else if (max_descent_condition && _hgt_dem < _hgt_dem_prev) {
                _max_sink_scaler *= (1.0f - hgt_dem_alpha);
            } else {
                _max_climb_scaler = _max_climb_scaler * (1.0f - hgt_dem_alpha) + hgt_dem_alpha;
                _max_sink_scaler  =  _max_sink_scaler * (1.0f - hgt_dem_alpha) + hgt_dem_alpha;
            }
        }
        _hgt_dem_prev = _hgt_dem;
    } else {
        // when flaring force height rate demand to the
        // configured sink rate and adjust the demanded height to
        // be kinematically consistent with the height rate.

        // set all height filter states to current height to prevent large pitch transients if flare is aborted
        _hgt_dem_lpf      = _height;
        _hgt_dem_rate_ltd = _height;
        _hgt_dem_in_prev  = _height;

        if (!_flare_initialised) {
            _flare_hgt_dem_adj = _hgt_dem;
            _flare_hgt_dem_ideal = _hgt_afe;
            _hgt_at_start_of_flare = _hgt_afe;
            _hgt_rate_at_flare_entry = _climb_rate;
            _flare_initialised = true;
        }

        // adjust the flare sink rate to increase/decrease as your travel further beyond the land wp
        float land_sink_rate_adj = _land_sink + _land_sink_rate_change*_distance_beyond_land_wp;

        // bring it in linearly with height
        float p;
        if (_hgt_at_start_of_flare > _flare_holdoff_hgt) {
            p = constrain_float((_hgt_at_start_of_flare - _hgt_afe) / (_hgt_at_start_of_flare - _flare_holdoff_hgt), 0.0f, 1.0f);
        } else {
            p = 1.0f;
        }
        _hgt_rate_dem = _hgt_rate_at_flare_entry * (1.0f - p) - land_sink_rate_adj * p;

        _flare_hgt_dem_ideal += _DT * _hgt_rate_dem; // the ideal height profile to follow
        _flare_hgt_dem_adj   += _DT * _hgt_rate_dem; // the demanded height profile that includes the pre-flare height tracking offset

        // fade across to the ideal height profile
        _hgt_dem = _flare_hgt_dem_adj * (1.0f - p) + _flare_hgt_dem_ideal * p;

        // correct for offset between height above ground and height above datum used by control loops
        _hgt_dem += (_hgt_afe - _height);
    }
}

void AP_TECS::_detect_underspeed(void)
{
    // see if we can clear a previous underspeed condition. We clear
    // it if we are now more than 15% above min speed, and haven't
    // been below min speed for at least 3 seconds.
    if (_flags.underspeed &&
        _TAS_state >= _TASmin * 1.15f &&
        AP_HAL::millis() - _underspeed_start_ms > 3000U) {
        _flags.underspeed = false;
    }

    if (_flight_stage == AP_FixedWing::FlightStage::VTOL) {
        _flags.underspeed = false;
    } else if (((_TAS_state < _TASmin * 0.9f) &&
                (_throttle_dem >= _THRmaxf * 0.95f) &&
                !_landing.is_flaring()) ||
                ((_height < _hgt_dem) && _flags.underspeed))
    {
        _flags.underspeed = true;
        if (_TAS_state < _TASmin * 0.9f) {
            // reset start time as we are still underspeed
            _underspeed_start_ms = AP_HAL::millis();
        }
    } else {
        // this clears underspeed if we reach our demanded height and
        // we are either below 95% throttle or we above 90% of min
        // airspeed
        _flags.underspeed = false;
    }
}

void AP_TECS::_update_energies(void)
{
    // Calculate specific energy demands
    _SPE_dem = _hgt_dem * GRAVITY_MSS;
    _SKE_dem = 0.5f * _TAS_dem_adj * _TAS_dem_adj;

    // Calculate specific energy rate demands and high pass filter demanded airspeed
    // rate of change to match the filtering applied to the measurement
    _SKEdot_dem = _TAS_state * (_TAS_rate_dem - _TAS_rate_dem_lpf);

    // Calculate specific energy
    _SPE_est = _height * GRAVITY_MSS;
    _SKE_est = 0.5f * _TAS_state * _TAS_state;

    // Calculate specific energy rate and high pass filter airspeed rate of change
    // to remove effect of complementary filter induced bias errors
    _SPEdot = _climb_rate * GRAVITY_MSS;
    _SKEdot = _TAS_state * (_vel_dot - _vel_dot_lpf);

}

/*
  current time constant. It is lower in landing to try to give a precise approach
 */
float AP_TECS::timeConstant(void) const
{
    if (_flags.is_doing_auto_land) {
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

/*
  calculate throttle demand - airspeed enabled case
 */
void AP_TECS::_update_throttle_with_airspeed(void)
{
    // Calculate limits to be applied to potential energy error to prevent over or underspeed occurring due to large height errors
    float SPE_err_max = MAX(_SKE_est - 0.5f * _TASmin * _TASmin, 0.0f);
    float SPE_err_min = MIN(_SKE_est - 0.5f * _TASmax * _TASmax, 0.0f);

    if (_flight_stage == AP_FixedWing::FlightStage::VTOL) {
        /*
          when we are in a VTOL state then we ignore potential energy
          errors as we have vertical motors that interfere with the
          total energy calculation.
         */
        SPE_err_max = SPE_err_min = 0;
    }

    // rate of change of potential energy is proportional to height error
    _SPEdot_dem = (_SPE_dem - _SPE_est) / timeConstant();

    // Calculate total energy error
    _STE_error = constrain_float((_SPE_dem - _SPE_est), SPE_err_min, SPE_err_max) + _SKE_dem - _SKE_est;
    float STEdot_dem = constrain_float((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);
    float STEdot_error = STEdot_dem - _SPEdot - _SKEdot;

    // Apply 0.5 second first order filter to STEdot_error
    // This is required to remove accelerometer noise from the  measurement
    const float filt_coef = 2.0f * _DT;
    STEdot_error = filt_coef * STEdot_error + (1.0f - filt_coef) * _STEdotErrLast;
    _STEdotErrLast = STEdot_error;

    // Calculate throttle demand
    // If underspeed condition is set, then demand full throttle
    if (_flags.underspeed) {
        _throttle_dem = 1.0f;
    } else if (_flags.is_gliding) {
        _throttle_dem = 0.0f;
    } else {
        // Calculate gain scaler from specific energy error to throttle
        const float K_thr2STE = (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf); // This is the derivative of STEdot wrt throttle measured across the max allowed throttle range.
        const float K_STE2Thr = 1 / (timeConstant() * K_thr2STE);

        // Calculate feed-forward throttle
        const float nomThr = aparm.throttle_cruise * 0.01f;
        const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
        // Use the demanded rate of change of total energy as the feed-forward demand, but add
        // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
        // drag increase during turns.
        const float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
        STEdot_dem = STEdot_dem + _rollComp * (1.0f/constrain_float(cosPhi * cosPhi, 0.1f, 1.0f) - 1.0f);
        const float ff_throttle = nomThr + STEdot_dem / K_thr2STE;

        // Calculate PD + FF throttle
        float throttle_damp = _thrDamp;
        if (_flags.is_doing_auto_land && !is_zero(_land_throttle_damp)) {
            throttle_damp = _land_throttle_damp;
        }
        _throttle_dem = (_STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;

        float THRminf_clipped_to_zero = constrain_float(_THRminf, 0, _THRmaxf);

        // Calculate integrator state upper and lower limits
        // Set to a value that will allow 0.1 (10%) throttle saturation to allow for noise on the demand
        // Additionally constrain the integrator state amplitude so that the integrator comes off limits faster.
        const float maxAmp = 0.5f*(_THRmaxf - THRminf_clipped_to_zero);
        const float integ_max = constrain_float((_THRmaxf - _throttle_dem + 0.1f),-maxAmp,maxAmp);
        const float integ_min = constrain_float((_THRminf - _throttle_dem - 0.1f),-maxAmp,maxAmp);

        // Calculate integrator state, constraining state
        // Set integrator to a max throttle value during climbout
        _integTHR_state = _integTHR_state + (_STE_error * _get_i_gain()) * _DT * K_STE2Thr;
        if (_flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
            if (!_flags.reached_speed_takeoff) {
                // ensure we run at full throttle until we reach the target airspeed
                _throttle_dem = MAX(_throttle_dem, _THRmaxf - _integTHR_state);
            }
        } else {
            _integTHR_state = constrain_float(_integTHR_state, integ_min, integ_max);
        }

        // Rate limit PD + FF throttle
        // Calculate the throttle increment from the specified slew time
        int8_t throttle_slewrate = aparm.throttle_slewrate;
        if (_landing.is_on_approach()) {
            const int8_t land_slewrate = _landing.get_throttle_slewrate();
            if (land_slewrate > 0) {
                throttle_slewrate = land_slewrate;
            }
        }

        if (throttle_slewrate != 0) {
            const float thrRateIncr = _DT * (_THRmaxf - THRminf_clipped_to_zero) * throttle_slewrate * 0.01f;

            _throttle_dem = constrain_float(_throttle_dem,
                                            _last_throttle_dem - thrRateIncr,
                                            _last_throttle_dem + thrRateIncr);
            _last_throttle_dem = _throttle_dem;
        }

        // Sum the components.
        _throttle_dem = _throttle_dem + _integTHR_state;

#if HAL_LOGGING_ENABLED
        if (AP::logger().should_log(_log_bitmask)){
            AP::logger().WriteStreaming("TEC3","TimeUS,KED,PED,KEDD,PEDD,TEE,TEDE,FFT,Imin,Imax,I,Emin,Emax",
                                        "Qffffffffffff",
                                        AP_HAL::micros64(),
                                        (double)_SKEdot,
                                        (double)_SPEdot,
                                        (double)_SKEdot_dem,
                                        (double)_SPEdot_dem,
                                        (double)_STE_error,
                                        (double)STEdot_error,
                                        (double)ff_throttle,
                                        (double)integ_min,
                                        (double)integ_max,
                                        (double)_integTHR_state,
                                        (double)SPE_err_min,
                                        (double)SPE_err_max);
        }
#endif
    }

    constrain_throttle();

}

// Constrain throttle demand and record clipping
void AP_TECS::constrain_throttle() {
    if (_throttle_dem > _THRmaxf) {
        _thr_clip_status = clipStatus::MAX;
        _throttle_dem = _THRmaxf;
    } else if (_throttle_dem < _THRminf) {
        _thr_clip_status = clipStatus::MIN;
        _throttle_dem = _THRminf;
    } else {
        _thr_clip_status = clipStatus::NONE;
    }
}

float AP_TECS::_get_i_gain(void)
{
    float i_gain = _integGain;
    if (_flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        i_gain = _integGain_takeoff;
    } else if (_flags.is_doing_auto_land) {
        if (!is_zero(_integGain_land)) {
            i_gain = _integGain_land;
        }
    }
    return i_gain;
}

/*
  calculate throttle, non-airspeed case
 */
void AP_TECS::_update_throttle_without_airspeed(int16_t throttle_nudge, float pitch_trim_deg)
{
    // reset clip status after possible use of synthetic airspeed
    _thr_clip_status = clipStatus::NONE;

    // Calculate throttle demand by interpolating between pitch and throttle limits
    float nomThr;
    //If landing and we don't have an airspeed sensor and we have a non-zero
    //TECS_LAND_THR param then use it
    if (_flags.is_doing_auto_land && _landThrottle >= 0) {
        nomThr = (_landThrottle + throttle_nudge) * 0.01f;
    } else { //not landing or not using TECS_LAND_THR parameter
        nomThr = (aparm.throttle_cruise + throttle_nudge)* 0.01f;
    }

    // Use a pitch angle that is the sum of a highpassed _pitch_dem and a lowpassed ahrs pitch
    // so that the throttle mapping adjusts for the effect of pitch control errors
    _pitch_demand_lpf.apply(_pitch_dem, _DT);
    const float pitch_demand_hpf = _pitch_dem - _pitch_demand_lpf.get();
    _pitch_measured_lpf.apply(_ahrs.get_pitch(), _DT);
    const float pitch_corrected_lpf = _pitch_measured_lpf.get() - radians(pitch_trim_deg);
    const float pitch_blended = pitch_demand_hpf + pitch_corrected_lpf;

    if (pitch_blended > 0.0f && _PITCHmaxf > 0.0f)
    {
        _throttle_dem = nomThr + (_THRmaxf - nomThr) * pitch_blended / _PITCHmaxf;
    }
    else if (pitch_blended < 0.0f && _PITCHminf < 0.0f)
    {
        _throttle_dem = nomThr + (_THRminf - nomThr) * pitch_blended / _PITCHminf;
    }
    else
    {
        _throttle_dem = nomThr;
    }

    if (_flags.is_gliding) {
        _throttle_dem = 0.0f;
        return;
    }

    // Calculate additional throttle for turn drag compensation including throttle nudging
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Use the demanded rate of change of total energy as the feed-forward demand, but add
    // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
    // drag increase during turns.
    float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
    float STEdot_dem = _rollComp * (1.0f/constrain_float(cosPhi * cosPhi, 0.1f, 1.0f) - 1.0f);
    _throttle_dem = _throttle_dem + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);

    constrain_throttle();
}

void AP_TECS::_detect_bad_descent(void)
{
    // Detect a demanded airspeed too high for the aircraft to achieve. This will be
    // evident by the following conditions:
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
    if (((!_flags.underspeed && (_STE_error > 200.0f) && (STEdot < 0.0f) && (_throttle_dem >= _THRmaxf * 0.9f)) || (_flags.badDescent && !_flags.underspeed && (_STE_error > 0.0f))) && !_flags.is_gliding) {
        _flags.badDescent = true;
    } else {
        _flags.badDescent = false;
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
    _SKE_weighting = constrain_float(_spdWeight, 0.0f, 2.0f);
    if (!(_ahrs.using_airspeed_sensor() || _use_synthetic_airspeed)) {
        _SKE_weighting = 0.0f;
    } else if (_flight_stage == AP_FixedWing::FlightStage::VTOL) {
        // if we are in VTOL mode then control pitch without regard to
        // speed. Speed is also taken care of independently of
        // height. This is needed as the usual relationship of speed
        // and height is broken by the VTOL motors
        _SKE_weighting = 0.0f;
    } else if ( _flags.underspeed || _flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING || _flags.is_gliding) {
        _SKE_weighting = 2.0f;
    } else if (_flags.is_doing_auto_land) {
        if (_spdWeightLand < 0) {
            // use sliding scale from normal weight down to zero at landing
            float scaled_weight = _spdWeight * (1.0f - constrain_float(_path_proportion,0,1));
            _SKE_weighting = constrain_float(scaled_weight, 0.0f, 2.0f);
        } else {
            _SKE_weighting = constrain_float(_spdWeightLand, 0.0f, 2.0f);
        }
    }

    float SPE_weighting = 2.0f - _SKE_weighting;

    // either weight can fade to 0, but don't go above 1 to prevent instability if tuned at a speed weight of 1 and wieghting is varied to end points in flight.
    SPE_weighting = MIN(SPE_weighting, 1.0f);
    _SKE_weighting = MIN(_SKE_weighting, 1.0f);

    // Calculate demanded specific energy balance and error
    float SEB_dem = _SPE_dem * SPE_weighting - _SKE_dem * _SKE_weighting;
    float SEB_est = _SPE_est * SPE_weighting - _SKE_est * _SKE_weighting;
    float SEB_error = SEB_dem - SEB_est;

    // track demanded height using the specified time constant
    float SEBdot_dem = _hgt_rate_dem * GRAVITY_MSS * SPE_weighting + SEB_error / timeConstant();
    const float SEBdot_dem_min = - _maxSinkRate * GRAVITY_MSS;
    const float SEBdot_dem_max = _maxClimbRate * GRAVITY_MSS;
    if (SEBdot_dem < SEBdot_dem_min) {
        SEBdot_dem = SEBdot_dem_min;
        _SEBdot_dem_clip = clipStatus::MIN;
    } else if (SEBdot_dem > SEBdot_dem_max) {
        SEBdot_dem = SEBdot_dem_max;
        _SEBdot_dem_clip = clipStatus::MAX;
    } else {
        _SEBdot_dem_clip = clipStatus::NONE;
    }

    // calculate specific energy balance rate error
    const float SEBdot_est = _SPEdot * SPE_weighting - _SKEdot * _SKE_weighting;
    float SEBdot_error = SEBdot_dem - SEBdot_est;

    // sum predicted plus damping correction
    // integral correction is added later
    // During flare a different damping gain is used
    float pitch_damp = _ptchDamp;
    if (_landing.is_flaring()) {
        pitch_damp = _landDamp;
    } else if (!is_zero(_land_pitch_damp) && _flags.is_doing_auto_land) {
        pitch_damp = _land_pitch_damp;
    }
    float SEBdot_dem_total = SEBdot_dem + SEBdot_error * pitch_damp;

    // inverse of gain from SEB to pitch angle
    float gainInv = (_TAS_state * GRAVITY_MSS);

    // During climbout/takeoff, bias the demanded pitch angle so that zero speed error produces a pitch angle
    // demand equal to the minimum value (which is )set by the mission plan during this mode). Otherwise the
    // integrator has to catch up before the nose can be raised to reduce speed during climbout.
    if (_flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        SEBdot_dem_total += _PITCHminf * gainInv;
    }

    // don't allow the integrator to rise by more than 20% of its full
    // Calculate max and min values for integrator state that will allow for no more than
    // 5deg of saturation. This allows for some pitch variation due to gusts before the
    // integrator is clipped. Otherwise the effectiveness of the integrator will be reduced in turbulence
    float integSEBdot_min = (gainInv * (_PITCHminf - radians(5.0f))) - SEBdot_dem_total;
    float integSEBdot_max = (gainInv * (_PITCHmaxf + radians(5.0f))) - SEBdot_dem_total;

    // Calculate integrator state, constraining input if pitch limits are exceeded
    // don't allow the integrator to rise by more than 10% of its full
    // range in one step. This prevents single value glitches from
    // causing massive integrator changes. See Issue#4066
    float integSEB_range = integSEBdot_max - integSEBdot_min;
    float integSEB_delta = constrain_float(SEBdot_error * _get_i_gain() * _DT, -integSEB_range*0.1f, integSEB_range*0.1f);

    // predict what pitch will be with uncontrained integration
    _pitch_dem_unc = (SEBdot_dem_total + _integSEBdot + integSEB_delta + _integKE) / gainInv;

    // integrate SEB rate error and apply integrator state limits
    const bool inhibit_integrator = ((_pitch_dem_unc > _PITCHmaxf) && integSEB_delta > 0.0f) ||
                                    ((_pitch_dem_unc < _PITCHminf) && integSEB_delta < 0.0f);
    if (!inhibit_integrator) {
        _integSEBdot += integSEB_delta;
        _integKE += (_SKE_est - _SKE_dem) * _SKE_weighting * _DT / timeConstant();
    } else {
        // fade out integrator if saturating
        const float coef = 1.0f - _DT / (_DT + timeConstant());
        _integSEBdot *= coef;
        _integKE *= coef;
    }
    _integSEBdot = constrain_float(_integSEBdot, integSEBdot_min, integSEBdot_max);
    const float KE_integ_limit = 0.25f * (_PITCHmaxf - _PITCHminf) * gainInv; // allow speed trim integrator to access 505 of pitch range
    _integKE = constrain_float(_integKE, - KE_integ_limit, KE_integ_limit);

    // Calculate pitch demand from specific energy balance signals
    _pitch_dem_unc = (SEBdot_dem_total + _integSEBdot + _integKE) / gainInv;

    // Add a feedforward term from demanded airspeed to pitch
    if (_flags.is_gliding) {
        _pitch_dem_unc += (_TAS_dem_adj - _pitch_ff_v0) * _pitch_ff_k;
    }

    // Rate limit the pitch demand to comply with specified vertical
    // acceleration limit
    float ptchRateIncr = _DT * _vertAccLim / _TAS_state;

    if ((_pitch_dem - _last_pitch_dem) > ptchRateIncr) {
        _pitch_dem = _last_pitch_dem + ptchRateIncr;
    } else if ((_pitch_dem - _last_pitch_dem) < -ptchRateIncr) {
        _pitch_dem = _last_pitch_dem - ptchRateIncr;
    }

    // Constrain pitch demand
    _pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);

    _last_pitch_dem = _pitch_dem;

#if HAL_LOGGING_ENABLED
    if (AP::logger().should_log(_log_bitmask)){
        // log to AP_Logger
        // @LoggerMessage: TEC2
        // @Vehicles: Plane
        // @Description: Additional information about the Total Energy Control System
        // @URL: http://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
        // @Field: TimeUS: Time since system startup
        // @Field: PEW: Potential energy weighting
        // @Field: KEW: Kinetic energy weighting
        // @Field: EBD: Energy balance demand
        // @Field: EBE: Energy balance estimate
        // @Field: EBDD: Energy balance rate demand
        // @Field: EBDE: Energy balance rate estimate
        // @Field: EBDDT: Energy balance rate demand + Energy balance rate error*pitch_damping
        // @Field: Imin: Minimum integrator value
        // @Field: Imax: Maximum integrator value
        // @Field: I: Energy balance error integral
        // @Field: KI: Pitch demand kinetic energy integral
        // @Field: tmin: Throttle min
        // @Field: tmax: Throttle max
        AP::logger().WriteStreaming("TEC2","TimeUS,PEW,KEW,EBD,EBE,EBDD,EBDE,EBDDT,Imin,Imax,I,KI,tmin,tmax",
                                    "Qfffffffffffff",
                                    AP_HAL::micros64(),
                                    (double)SPE_weighting,
                                    (double)_SKE_weighting,
                                    (double)SEB_dem,
                                    (double)SEB_est,
                                    (double)SEBdot_dem,
                                    (double)SEBdot_est,
                                    (double)SEBdot_dem_total,
                                    (double)integSEBdot_min,
                                    (double)integSEBdot_max,
                                    (double)_integSEBdot,
                                    (double)_integKE,
                                    (double)_THRminf,
                                    (double)_THRmaxf);
    }
#endif
}

void AP_TECS::_initialise_states(float hgt_afe)
{
    // Initialise states and variables if DT > 0.2 second or TECS is getting overriden or in climbout.
    _flags.reset = false;

    if (_DT > 0.2f || _need_reset) {
        _SKE_weighting        = 1.0f;
        _integTHR_state       = 0.0f;
        _integSEBdot          = 0.0f;
        _integKE              = 0.0f;
        _last_throttle_dem    = aparm.throttle_cruise * 0.01f;
        _last_pitch_dem       = _ahrs.get_pitch();
        _hgt_afe              = hgt_afe;
        _hgt_dem_in_prev      = hgt_afe;
        _hgt_dem_lpf          = hgt_afe;
        _hgt_dem_rate_ltd     = hgt_afe;
        _hgt_dem_prev         = hgt_afe;
        _TAS_dem_adj          = _TAS_dem;
        _flags.reset          = true;
        _DT                   = 0.02f; // when first starting TECS, use the most likely time constant
        _lag_comp_hgt_offset  = 0.0f;
        _post_TO_hgt_offset   = 0.0f;
        _use_synthetic_airspeed_once = false;

        _flags.underspeed            = false;
        _flags.badDescent            = false;
        _flags.reached_speed_takeoff = false;
        _need_reset                  = false;

        // misc variables used for alternative precision landing pitch control
        _hgt_at_start_of_flare    = 0.0f;
        _hgt_rate_at_flare_entry  = 0.0f;
        _hgt_afe                  = 0.0f;
        _pitch_min_at_flare_entry = 0.0f;

        _max_climb_scaler = 1.0f;
        _max_sink_scaler = 1.0f;

        const float fc = 1.0f / (M_2PI * _timeConst);
        _pitch_demand_lpf.set_cutoff_frequency(fc);
        _pitch_measured_lpf.set_cutoff_frequency(fc);
        _pitch_demand_lpf.reset(_ahrs.get_pitch());
        _pitch_measured_lpf.reset(_ahrs.get_pitch());

    } else if (_flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        
        if (!_flag_throttle_forced) {
            // Calculate the takeoff target height offset before _hgt_dem_in_raw gets reset below.
            // Prevent the offset from becoming negative.
            _post_TO_hgt_offset = MAX(MIN(_climb_rate_limit * _hgt_dem_tconst, _hgt_dem_in_raw - hgt_afe), 0);
        } else {
            // If throttle is externally forced, this mechanism of adding energy is unnecessary.
            _post_TO_hgt_offset = 0;
        }

        _hgt_afe              = hgt_afe;
        _hgt_dem_lpf          = hgt_afe;
        _hgt_dem_rate_ltd     = hgt_afe;
        _hgt_dem_prev         = hgt_afe;
        _hgt_dem              = hgt_afe;
        _hgt_dem_in_prev      = hgt_afe;
        _hgt_dem_in_raw       = hgt_afe;
        _flags.underspeed     = false;
        _flags.badDescent     = false;
        _TAS_dem_adj = _TAS_dem;
        _max_climb_scaler = 1.0f;
        _max_sink_scaler = 1.0f;
        _pitch_demand_lpf.reset(_ahrs.get_pitch());
        _pitch_measured_lpf.reset(_ahrs.get_pitch());
        

        if (!_flag_have_reset_after_takeoff) {
            _flags.reset          = true;
            _flag_have_reset_after_takeoff  = true;
        }
    }

    if (_flight_stage != AP_FixedWing::FlightStage::TAKEOFF && _flight_stage != AP_FixedWing::FlightStage::ABORT_LANDING) {
        // reset takeoff speed flag when not in takeoff
        _flags.reached_speed_takeoff = false;
        _flag_have_reset_after_takeoff = false;
    }
}

void AP_TECS::_update_STE_rate_lim(void)
{
    // Calculate Specific Total Energy Rate Limits & deceleration rate bounds
	// Keep the 50% energy margin from the original velRate calculation for now
    // This is a trivial calculation at the moment but will get bigger once we start adding altitude effects
    _STEdot_max = _climb_rate_limit * GRAVITY_MSS;
    _STEdot_min = - _minSinkRate * GRAVITY_MSS;
    _STEdot_neg_max = - _maxSinkRate * GRAVITY_MSS;
}


void AP_TECS::update_pitch_throttle(int32_t hgt_dem_cm,
                                    int32_t EAS_dem_cm,
                                    enum AP_FixedWing::FlightStage flight_stage,
                                    float distance_beyond_land_wp,
                                    int32_t ptchMinCO_cd,
                                    int16_t throttle_nudge,
                                    float hgt_afe,
                                    float load_factor,
                                    float pitch_trim_deg)
{
    uint64_t now = AP_HAL::micros64();
    // check how long since we last did the 50Hz update; do nothing in
    // this loop if that hasn't run for some signficant period of
    // time.  Notably, it may never have run, leaving _TAS_state as
    // zero and subsequently division-by-zero errors.
    const float _DT_for_update_50hz = (now - _update_50hz_last_usec) * 1.0e-6f;
    if (_update_50hz_last_usec == 0 || _DT_for_update_50hz > 1.0) {
        // more than 1 second since it was run, don't do anything yet:
        return;
    }

    // Calculate time in seconds since last update
    _DT = (now - _update_pitch_throttle_last_usec) * 1.0e-6f;
    _DT = MAX(_DT, 0.001f);
    _update_pitch_throttle_last_usec = now;

    _flags.is_gliding = _flags.gliding_requested || _flags.propulsion_failed || aparm.throttle_max==0;
    _flags.is_doing_auto_land = (flight_stage == AP_FixedWing::FlightStage::LAND);
    _distance_beyond_land_wp = distance_beyond_land_wp;
    _flight_stage = flight_stage;

    // Convert inputs
    _hgt_dem_in_raw = hgt_dem_cm * 0.01f;
    _EAS_dem = EAS_dem_cm * 0.01f;
    _hgt_afe = hgt_afe;
    _load_factor = load_factor;

    // Don't allow height deamnd to continue changing in a direction that saturates vehicle manoeuvre limits
    // if vehicle is unable to follow the demanded climb or descent.
    const bool max_climb_condition = (_pitch_dem_unc > _PITCHmaxf || _thr_clip_status == clipStatus::MAX) &&
                                    !(_flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING);
    const bool max_descent_condition = _pitch_dem_unc < _PITCHminf || _thr_clip_status == clipStatus::MIN;
    if (max_climb_condition && _hgt_dem_in_raw > _hgt_dem_in_prev) {
        _hgt_dem_in = _hgt_dem_in_prev;
    } else if (max_descent_condition && _hgt_dem_in_raw < _hgt_dem_in_prev) {
        _hgt_dem_in = _hgt_dem_in_prev;
    } else {
        _hgt_dem_in = _hgt_dem_in_raw;
    }

    // Update the throttle limits.
    _update_throttle_limits();

    _update_pitch_limits(ptchMinCO_cd);

    if (flight_stage == AP_FixedWing::FlightStage::TAKEOFF || flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        if (!_flags.reached_speed_takeoff && _TAS_state >= _TASmin && _TASmin > 0) {
            // we have reached our target speed in takeoff, allow for
            // normal throttle control
            _flags.reached_speed_takeoff = true;
        }
    }

    // initialise selected states and variables if DT > 1 second or in climbout
    _initialise_states(hgt_afe);

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

    // Calculate pitch demand
    _update_pitch();

    // Calculate throttle demand - use simple pitch to throttle if no
    // airspeed sensor.
    // Note that caller can demand the use of
    // synthetic airspeed for one loop if needed. This is required
    // during QuadPlane transition when pitch is constrained
    if (_ahrs.using_airspeed_sensor() || _use_synthetic_airspeed || _use_synthetic_airspeed_once) {
        _update_throttle_with_airspeed();
        _use_synthetic_airspeed_once = false;
        _using_airspeed_for_throttle = true;
    } else {
        _update_throttle_without_airspeed(throttle_nudge, pitch_trim_deg);
        _using_airspeed_for_throttle = false;
    }

    // Detect bad descent due to demanded airspeed being too high
    _detect_bad_descent();

    if (_options & OPTION_GLIDER_ONLY) {
        _flags.badDescent = false;
    }

#if HAL_LOGGING_ENABLED
    if (AP::logger().should_log(_log_bitmask)){
        // log to AP_Logger
        // @LoggerMessage: TECS
        // @Vehicles: Plane
        // @Description: Information about the Total Energy Control System
        // @URL: http://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
        // @Field: TimeUS: Time since system startup
        // @Field: h: height estimate (UP) currently in use by TECS
        // @Field: dh: current climb rate ("delta-height")
        // @Field: hin: height demand received by TECS
        // @Field: hdem: height demand after rate limiting and filtering that TECS is currently trying to achieve
        // @Field: dhdem: climb rate TECS is currently trying to achieve
        // @Field: spdem: True AirSpeed TECS is currently trying to achieve
        // @Field: sp: current estimated True AirSpeed
        // @Field: dsp: x-axis acceleration estimate ("delta-speed")
        // @Field: th: throttle output
        // @Field: ph: pitch output
        // @Field: pmin: pitch lower limit
        // @Field: pmax: pitch upper limit
        // @Field: dspdem: demanded acceleration output ("delta-speed demand")
        // @Field: f: flags
        // @FieldBits: f: Underspeed,UnachievableDescent,AutoLanding,ReachedTakeoffSpd
        AP::logger().WriteStreaming("TECS", "TimeUS,h,dh,hin,hdem,dhdem,spdem,sp,dsp,th,ph,pmin,pmax,dspdem,f",
                                    "smnmmnnnn------",
                                    "F00000000------",
                                    "QfffffffffffffB",
                                    now,
                                    (double)_height,
                                    (double)_climb_rate,
                                    (double)_hgt_dem_in_raw,
                                    (double)_hgt_dem,
                                    (double)_hgt_rate_dem,
                                    (double)_TAS_dem_adj,
                                    (double)_TAS_state,
                                    (double)_vel_dot,
                                    (double)_throttle_dem,
                                    (double)_pitch_dem,
                                    (double)_PITCHminf,
                                    (double)_PITCHmaxf,
                                    (double)_TAS_rate_dem,
                                    _flags_byte);
    }
#endif
}

// set minimum throttle override, [-1, -1] range
// it is applicable for one control cycle only
void AP_TECS::set_throttle_min(const float thr_min) {
    // Don't change the limit if it is already covered.
    if (thr_min > _THRminf_ext) {
        _THRminf_ext = thr_min;
    }
}

// set minimum throttle override, [0, -1] range
// it is applicable for one control cycle only
void AP_TECS::set_throttle_max(const float thr_max) {
    // Don't change the limit if it is already covered.
    if (thr_max < _THRmaxf_ext) {
        _THRmaxf_ext = thr_max;
    }
}

void AP_TECS::_update_throttle_limits() {

    // Configure max throttle; constrain to the external safety limits.
    _THRmaxf = MIN(1.0f, _THRmaxf_ext);
    // Configure min throttle; constrain to the external safety limits.
    _THRminf = MAX(-1.0f, _THRminf_ext);

    // Allow a minimum of 1% throttle range, primarily to prevent TECS numerical errors.
    const float thr_eps = 0.01;
    if (fabsf(_THRminf-_THRmaxf) < thr_eps) {
        _flag_throttle_forced = true;
        if (_THRmaxf < 1) {
            _THRmaxf = MAX(_THRmaxf, _THRminf + 0.01f);
        } else {
            _THRminf = MIN(_THRminf, _THRmaxf - 0.01f);
        }
    } else {
        _flag_throttle_forced = false;
    }
    
    // Reset the external throttle limits.
    // Caller will have to reset them in the next iteration.
    _THRminf_ext = -1.0f;
    _THRmaxf_ext = 1.0f;
}

void AP_TECS::set_pitch_min(const float pitch_min) {
    // Don't change the limit if it is already covered.
    if (pitch_min > _PITCHminf_ext) {
        _PITCHminf_ext = pitch_min;
    }
}

void AP_TECS::set_pitch_max(const float pitch_max) {
    // Don't change the limit if it is already covered.
    if (pitch_max < _PITCHmaxf_ext) {
        _PITCHmaxf_ext = pitch_max;
    }
}

void AP_TECS::_update_pitch_limits(const int32_t ptchMinCO_cd) {
    // If TECS_PITCH_{MAX,MIN} isn't set then use LIM_PITCH_{MAX,MIN}.
    // Don't allow TECS_PITCH_{MAX,MIN} to be larger than LIM_PITCH_{MAX,MIN}.
    if (_pitch_max == 0) {
        _PITCHmaxf = aparm.pitch_limit_max;
    } else {
        _PITCHmaxf = _pitch_max;
    }

    if (_pitch_min == 0) {
        _PITCHminf = aparm.pitch_limit_min;
    } else {
        _PITCHminf = _pitch_min;
    }

    if (!_landing.is_on_approach()) {
        // reset land pitch min when not landing
        _land_pitch_min = _PITCHminf;
    }

    // calculate the expected pitch angle from the demanded climb rate and airspeed for use during approach and flare
    if (_landing.is_flaring()) {
        // smoothly move the min pitch to the required minimum at touchdown
        float p; // 0 at start of flare, 1 at finish
        if (!_flare_initialised) {
            p = 0.0f;
        } else if (_hgt_at_start_of_flare > _flare_holdoff_hgt) {
            p = constrain_float((_hgt_at_start_of_flare - _hgt_afe) / _hgt_at_start_of_flare, 0.0f, 1.0f);
        } else {
            p = 1.0f;
        }
        const float pitch_limit_deg = (1.0f - p) * _pitch_min_at_flare_entry + p * 0.01f * _landing.get_pitch_cd();

        // in flare use min pitch from LAND_PITCH_DEG
        _PITCHminf = MAX(_PITCHminf, pitch_limit_deg);

        // and use max pitch from TECS_LAND_PMAX
        if (_land_pitch_max != 0) {
            // note that this allows a flare pitch outside the normal TECS auto limits
            _PITCHmaxf = _land_pitch_max;
        }
    } else if (_landing.is_on_approach()) {
        _PITCHminf = MAX(_PITCHminf, aparm.pitch_limit_min);
        _pitch_min_at_flare_entry = _PITCHminf;
        _flare_initialised = false;
    } else {
        _flare_initialised = false;
    }

    if (_landing.is_on_approach()) {
        // don't allow the lower bound of pitch to decrease, nor allow
        // it to increase rapidly. This prevents oscillation of pitch
        // demand while in landing approach based on rapidly changing
        // time to flare estimate
        if (_land_pitch_min <= -90) {
            _land_pitch_min = _PITCHminf;
        }
        const float flare_pitch_range = 20;
        const float delta_per_loop = (flare_pitch_range/_landTimeConst) * _DT;
        _PITCHminf = MIN(_PITCHminf, _land_pitch_min+delta_per_loop);
        _land_pitch_min = MAX(_land_pitch_min, _PITCHminf);
        _PITCHminf = MAX(_land_pitch_min, _PITCHminf);
    }

    // Apply TAKEOFF minimum pitch
    if (_flight_stage == AP_FixedWing::FlightStage::TAKEOFF
        || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING)
    {
        _PITCHminf = CentiDegreesToRadians(ptchMinCO_cd);
    }

    // Apply external limits.
    _PITCHmaxf = MIN(_PITCHmaxf, _PITCHmaxf_ext);
    _PITCHminf = MAX(_PITCHminf, _PITCHminf_ext);
    
    // Reset the external pitch limits.
    _PITCHminf_ext = -90.0f;
    _PITCHmaxf_ext = 90.0f;

    // convert to radians
    _PITCHmaxf = radians(_PITCHmaxf);
    _PITCHminf = radians(_PITCHminf);

    // don't allow max pitch to go below min pitch
    _PITCHmaxf = MAX(_PITCHmaxf, _PITCHminf);
}