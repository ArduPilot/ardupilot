#include "AP_TECS.h"

#include <AP_Logger/AP_Logger.h>

/*
  an alternative height control scheme based around vertical
  acceleration
 */

/*
  calculate vertical acceleration demand
 */
void AP_TECS::_calc_vert_accel_demand(void)
{
    const float pos_gain = MAX(_hgt_accln_gain, 0.1f);
    const float vel_gain = 2.0f * _hgt_accln_damping_ratio * sqrtf(pos_gain);

    // constrain position error to respect vertical velocity limits
    const float down_error_limit = (vel_gain / pos_gain) * _sink_rate_limit;
    const float up_error_limit = (vel_gain / pos_gain) * _climb_rate_limit;
    _hgt_accel_dem = constrain_float(_hgt_dem - _height, -down_error_limit, up_error_limit) * pos_gain +
                       (_hgt_rate_dem - _climb_rate) * vel_gain;
    _hgt_accel_dem = constrain_float(_hgt_accel_dem, -_vertAccLim, _vertAccLim);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: TEC6
    // @Vehicles: Plane
    // @Description: TECS log for vertical acceleration demand calculation
    // @URL: http://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
    // @Field: TimeUS: Time since system startup
    // @Field: HAD: height acceleration demand
    // @Field: UEL: upward position error limit
    // @Field: DEL: downward position error limit
    // @Field: HD: height demand
    // @Field: H: current height estimate
    // @Field: HRD: height rate demand
    // @Field: CR: current climb rate
    // @Field: VAL: vertical acceleration limit
    AP::logger().WriteStreaming("TEC6","TimeUS,HAD,UEL,DEL,HD,H,HRD,CR,VAL",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (double)_hgt_accel_dem,
                                (double)up_error_limit,
                                (double)down_error_limit,
                                (double)_hgt_dem,
                                (double)_height,
                                (double)_hgt_rate_dem,
                                (double)_climb_rate,
                                (double)_vertAccLim);
#endif // HAL_LOGGING_ENABLED
}

/*
  update throttle demand when acceleration based control is enabled
 */
void AP_TECS::_update_throttle_accel(int16_t throttle_nudge, float pitch_trim_deg)
{
    _calc_vert_accel_demand();

    // keep integrators to legacy TECS loops zeroed.
    _integTHR_state = 0.0f;
    _integSEBdot = 0.0f;
    _integKE = 0.0f;

    // get throttle demand setpoint, then adjust using a PID control scheme if airspeed available
    _update_throttle_without_airspeed(throttle_nudge, pitch_trim_deg);

    if (_ahrs.using_airspeed_sensor() || _use_synthetic_airspeed) {
        const float FF = _throttle_dem;
        const float P = (_TAS_dem - _TAS_state) * _thr_pid_p_gain;
        const float D = (_TAS_rate_dem_lpf - _vel_dot) * _thr_pid_d;
        // check for saturation and prevent integrator windup
        _throttle_dem = FF + P + _thr_PID_I_term + D;
        constrain_throttle();
        if (_alt_thr_clip_status == clipStatus::MAX) {
            _thr_PID_I_term +=  MIN(P * _thr_pid_i_gain, 0.0f);
        } else if (_alt_thr_clip_status == clipStatus::MIN) {
            _thr_PID_I_term +=  MAX(P * _thr_pid_i_gain, 0.0f);
        } else {
            _thr_PID_I_term += P * _thr_pid_i_gain;
        }
        // sum components
        _throttle_dem = FF + P + _thr_PID_I_term + D;

#if HAL_LOGGING_ENABLED
        // @LoggerMessage: TEC5
        // @Vehicles: Plane
        // @Description: TECS log for alternative height and speed control method
        // @URL: http://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
        // @Field: TimeUS: Time since system startup
        // @Field: FF: throttle feedforward from pitch-based mapping
        // @Field: P: throttle PID proportional term
        // @Field: D: throttle PID derivative term
        // @Field: I: throttle PID integral term
        // @Field: TD: true airspeed demand
        // @Field: TM: true airspeed measured (state estimate)
        // @Field: TRD: true airspeed rate demand (low-pass filtered)
        // @Field: Tdem: final throttle demand
        AP::logger().WriteStreaming("TEC5","TimeUS,FF,P,D,I,TD,TM,TRD,Tdem",
                                    "Qffffffff",
                                    AP_HAL::micros64(),
                                    (double)FF,
                                    (double)P,
                                    (double)D,
                                    (double)_thr_PID_I_term,
                                    (double)_TAS_dem,
                                    (double)_TAS_state,
                                    (double)_TAS_rate_dem_lpf,
                                    (double)_throttle_dem);
#endif // HAL_LOGGING_ENABLED
    }
}
