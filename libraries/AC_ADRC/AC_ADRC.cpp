#include "AC_ADRC.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// ====================== Constant Definitions ======================
#define  ADRC_ATT_DT_MIN     0.0005f              // min loop time (2000Hz)
#define  ADRC_ATT_DT_MAX     0.02f                // max loop time (50Hz),make sure wo*dt<0.6
// Hard limit constants
constexpr float ADRC_Z_EST_LIMIT_RATIO = 0.2f;
constexpr float ADRC_B0_MIN_DEFAULT    = 1.0f;
constexpr float ADRC_OUT_MIN_DEFAULT   = 0.1f;
constexpr float ADRC_NL_DELTA_MIN      = 0.001f;
constexpr uint32_t ADRC_LOG_PERIOD_MS  = 20;

// table of user settable parameters
const AP_Param::GroupInfo AC_ADRC::var_info[] =
{
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("MD", 0, AC_ADRC, _adrc_type, default_adrc_type), //adrc control type mode
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B0", 1, AC_ADRC, _b0,        default_b0),        //system scaling factor
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("TDR",2, AC_ADRC, _td_r,     default_td_r),        //differential tracker TD-r
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("TDH",3, AC_ADRC, _td_h0,     default_td_h0),     //differential tracker TD-h0
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B1", 4, AC_ADRC, _eso_beta1, default_eso_beta1), //eso-beta1
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B2", 5, AC_ADRC, _eso_beta2, default_eso_beta2), //eso-beta2
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B3", 6, AC_ADRC, _eso_beta3, default_eso_beta3), //eso-beta3
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D1", 7, AC_ADRC, _eso_delta,    default_eso_delta), //eso-delta
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("EH", 8, AC_ADRC, _eso_h_gain, default_eso_h_gain),  //eso-h-gain for dt
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("A1", 9, AC_ADRC, _nlsef_alpha1,    default_nlsef_alpha1), //nlsef-alpha1
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("A2",10, AC_ADRC, _nlsef_alpha2,    default_nlsef_alpha2), //nlsef-alpha2
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D2",11, AC_ADRC, _nlsef_delta,    default_nlsef_delta), //nlsef-delta
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("KP",12, AC_ADRC, _nlsef_kp, default_nlsef_kp),//nlsef-kp
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("KD",13, AC_ADRC, _nlsef_kd, default_nlsef_kd),//nlsef-kd
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("UM",14, AC_ADRC, _limit_u_max, default_limit_u_max), //u-limit
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLT",15, AC_ADRC, _filt_target_hz, default_filt_target_hz),//filter target hz
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLM",16, AC_ADRC, _filt_measure_hz, default_filt_measure_hz),//filter measure hz
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("WC",17, AC_ADRC, _wc, default_wc), //control bandwidth
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("WO",18, AC_ADRC, _wo, default_wo),//observation bandwidth
    AP_GROUPEND
};

/**
 * @brief ADRC per-axis state container
 * All integrator/ESO/TD states for single axis
 */
struct AC_ADRC::ADRC_State {
    float v1;    // TD state 1
    float v2;    // TD state 2
    float z1;    // ESO z1
    float z2;    // ESO z2
    float z3;    // ESO z3
    float last_u;// previous control output
};

AC_ADRC::AC_ADRC(const AC_ADRC::Defaults &defaults) :
		default_adrc_type(defaults.adrc_type),
		default_b0(defaults.b0),
		default_td_r(defaults.td_r),
		default_td_h0(defaults.td_h0),
		default_eso_beta1(defaults.eso_beta1),
		default_eso_beta2(defaults.eso_beta2),
		default_eso_beta3(defaults.eso_beta3),
		default_eso_delta(defaults.eso_delta),
		default_eso_h_gain(defaults.eso_h_gain),
		default_nlsef_alpha1(defaults.nlsef_alpha1),
		default_nlsef_alpha2(defaults.nlsef_alpha2),
		default_nlsef_delta(defaults.nlsef_delta),
		default_nlsef_kp(defaults.nlsef_kp),
		default_nlsef_kd(defaults.nlsef_kd),
		default_limit_u_max(defaults.limit_u_max),
		default_filt_target_hz(defaults.filt_target_hz),
		default_filt_measure_hz(defaults.filt_measure_hz),
		default_wc(defaults.wc),
		default_wo(defaults.wo)
{
    AP_Param::setup_object_defaults(this, var_info);
    reset_filter(0.0f,0.0f);
}

/**
 * @brief Main ADRC update entry
 * @param target setpoint
 * @param measure feedback measurement
 * @param dt time step
 * @param limit unused flag
 * @param aix axis index:0 roll,1 pitch,2 yaw
 * @return control output u
 */
float AC_ADRC::update_all(const float& target,const float& measure,float dt,bool limit,uint8_t aix)
{
    // input validity check
    if ( !is_positive(dt)
    	|| !is_valid_data(target)
		||!is_valid_data(measure))
    {
        return _last_u_out;
    }

    if (dt < ADRC_ATT_DT_MIN)
    {
        return _last_u_out;
    }
    dt = MIN(dt, ADRC_ATT_DT_MAX);

    // pack state into struct
    ADRC_State state{_v1, _v2, _z1, _z2, _z3, _last_u_out};
    float u = update_axis(target, measure, dt, state, limit, aix);

    // write back state
    _v1 = state.v1;
    _v2 = state.v2;
    _z1 = state.z1;
    _z2 = state.z2;
    _z3 = state.z3;
    _last_u_out = state.last_u;

    return u;
}

/**
 * @brief Single axis ADRC calculation, dispatch by adrc type
 */
float AC_ADRC::update_axis(float target, float measure, float dt, ADRC_State &state,bool limit,uint8_t aix)
{
    // lowpass filter
	_target = _target + get_filt_target_alpha(dt)*(target - _target);
	_measure = _measure + get_filt_measure_alpha(dt)*(measure - _measure);

    // check state finite, reset if invalid
    if (!_is_state_valid(state))
    {
        _reset_state(state, _measure, _target);
        return 0.0f;
    }

    float u = 0.0f;
	switch(_adrc_type.get())
	{
	case 0:  //1st order LADRC, adrc_type=0
        u = _update_ladrc_1st(target,measure,_target, _measure, dt, state, aix);
		break;
	case 1: //2nd order LADRC bandwidth form, adrc_type=1
        u = _update_ladrc_2nd(target,measure,_target, _measure, dt, state, aix);
		break;
	case 2: //2nd order LADRC discrete pole placement, adrc_type=2
        u = _update_ladrc_2nd_exp(target,measure,_target, _measure, dt, state, aix);
		break;
	default: //Nonlinear ADRC with TD + NLSEF, adrc_type=3
        u = _update_nl_adrc(target,measure,_target, _measure, dt, state, aix);
		break;
	}
    return u;
}

/**
 * @brief 1st order LADRC, adrc_type=0
 */
float AC_ADRC::_update_ladrc_1st(float original_target, float original_measure,float target, float measure, float dt, ADRC_State &state, uint8_t aix)
{
    const float b0   = MAX(_b0.get(), ADRC_B0_MIN_DEFAULT);
    const float output_max  = MAX(_limit_u_max.get(), ADRC_B0_MIN_DEFAULT);
    const float temp_kp=MAX(_wc.get(),0.0f);
    const float temp_wo=MAX(_wo.get(),0.0f);
    const float temp_beta1=2.0f*temp_wo;
    const float temp_beta2=temp_wo*temp_wo;

    float error_eso = state.z1 - measure;
    state.z1 = state.z1 + dt * (state.z2 - temp_beta1 * error_eso + b0 * state.last_u);
    state.z2 = state.z2 + dt * ( - temp_beta2 * error_eso );
    state.z2=constrain_float(state.z2, -ADRC_Z_EST_LIMIT_RATIO*b0*output_max, ADRC_Z_EST_LIMIT_RATIO*b0*output_max);

    float error_control = target - state.z1;
    float u0= temp_kp * error_control ;
    float u_temp = (u0 - state.z2) / b0;
    float u = constrain_float(u_temp, -output_max, output_max);
    state.last_u = u;

    if(aix==0)
    {
    	 _write_log(aix, "RARC", original_target, original_measure, error_eso, error_control, state.z1, state.z2, temp_beta2*error_eso, temp_kp*error_control, temp_beta1*error_eso, u0, u, u0-state.z2, u_temp);
    }
    else if(aix==1)
    {
    	 _write_log(aix, "PARC", original_target, original_measure, error_eso, error_control, state.z1, state.z2, temp_beta2*error_eso, temp_kp*error_control, temp_beta1*error_eso, u0, u, u0-state.z2, u_temp);
    }
    else if(aix==2)
    {
    	 _write_log(aix, "YARC", original_target, original_measure, error_eso, error_control, state.z1, state.z2, temp_beta2*error_eso, temp_kp*error_control, temp_beta1*error_eso, u0, u, u0-state.z2, u_temp);
    }
    return u;
}

/**
 * @brief 2nd order LADRC bandwidth form, adrc_type=1
 */
float AC_ADRC::_update_ladrc_2nd(float original_target, float original_measure,float target, float measure, float dt, ADRC_State &state, uint8_t aix)
{
    const float b0   = MAX(_b0.get(), ADRC_B0_MIN_DEFAULT);
    const float output_max  = MAX(_limit_u_max.get(), ADRC_OUT_MIN_DEFAULT);
    const float temp_wc=MAX(_wc.get(),0.0f);
    const float temp_wo=MAX(_wo.get(),0.0f);
    const float temp_kp=temp_wc*temp_wc;
    const float temp_kd=2.0f*temp_wc;
    const float temp_beta1=3.0f*temp_wo;
    const float temp_beta2=3.0f*temp_wo*temp_wo;
    const float temp_beta3=temp_wo*temp_wo*temp_wo;

    float error_eso = state.z1 - measure;
    state.z1 = state.z1 + dt * (state.z2 - temp_beta1 * error_eso);
    state.z2 = state.z2 + dt * (state.z3 - temp_beta2 * error_eso + b0 * state.last_u);
    state.z3 = state.z3 + dt * (-temp_beta3 * error_eso);
    state.z2=constrain_float(state.z2, -ADRC_Z_EST_LIMIT_RATIO*b0*output_max, ADRC_Z_EST_LIMIT_RATIO*b0*output_max);
    state.z3=constrain_float(state.z3, -ADRC_Z_EST_LIMIT_RATIO*b0*output_max, ADRC_Z_EST_LIMIT_RATIO*b0*output_max);

    float error_control = target - state.z1;
    float u0= temp_kp * error_control - temp_kd * state.z2;
    float u_temp = (u0 - state.z3) / b0;
    float u = constrain_float(u_temp, -output_max, output_max);
    state.last_u = u;

    if(aix==0)
    {
    	_write_log(aix, "RBRC", original_target, original_measure, error_eso, error_control, state.z1, state.z2, state.z3, temp_kp*error_control, -temp_kd*state.z2, u0, u, u0-state.z3, u_temp);
    }
    else if(aix==1)
    {
    	_write_log(aix, "PBRC", original_target, original_measure, error_eso, error_control, state.z1, state.z2, state.z3, temp_kp*error_control, -temp_kd*state.z2, u0, u, u0-state.z3, u_temp);
    }
    else if(aix==2)
    {
    	_write_log(aix, "YBRC", original_target, original_measure, error_eso, error_control, state.z1, state.z2, state.z3, temp_kp*error_control, -temp_kd*state.z2, u0, u, u0-state.z3, u_temp);
    }

    return u;
}

/**
 * @brief 2nd order LADRC discrete pole placement, adrc_type=2
 */
float AC_ADRC::_update_ladrc_2nd_exp(float original_target, float original_measure,float target, float measure, float dt, ADRC_State &state, uint8_t aix)
{
    const float b0   = MAX(_b0.get(), ADRC_B0_MIN_DEFAULT);
    const float output_max  = MAX(_limit_u_max.get(), ADRC_OUT_MIN_DEFAULT);
    const float temp_wc=MAX(_wc.get(),0.0f);
    const float temp_wo=MAX(_wo.get(),0.0f);
    const float temp_kp=temp_wc*temp_wc;
    const float temp_kd=2.0f*temp_wc;

    float r = expf(-temp_wo * dt);
    float temp_beta1 = 3.0f * (1.0f - r);
    float temp_beta2 = (1.0f - r) * (1.0f - r) * (5.0f + r) / (2.0f * dt);
    float temp_beta3 = (1.0f - r) * (1.0f - r) * (1.0f - r) / (dt * dt);

    float error_eso = state.z1 - measure;
    state.z1 = state.z1 + dt * state.z2 - temp_beta1 * error_eso;
    state.z2 = state.z2 + dt * state.z3 - temp_beta2 * error_eso +  dt *b0 * state.last_u;
    state.z3 = state.z3 -temp_beta3 * error_eso;
    state.z2=constrain_float(state.z2, -ADRC_Z_EST_LIMIT_RATIO*b0*output_max, ADRC_Z_EST_LIMIT_RATIO*b0*output_max);
    state.z3=constrain_float(state.z3, -ADRC_Z_EST_LIMIT_RATIO*b0*output_max, ADRC_Z_EST_LIMIT_RATIO*b0*output_max);

    float error_control = target - state.z1;
    float u0= temp_kp * error_control - temp_kd * state.z2;
    float u_temp = (u0 - state.z3) / b0;
    float u = constrain_float(u_temp, -output_max, output_max);
    state.last_u = u;


    if(aix==0)
    {
    	_write_log(aix, "RCRC",original_target, original_measure, state.z1, state.z2, state.z3, temp_kp*error_control, -temp_kd*state.z2, error_eso,error_control, u0, u, 0, u_temp);
    }
    else if(aix==1)
    {
    	_write_log(aix, "PCRC",original_target, original_measure, state.z1, state.z2, state.z3, temp_kp*error_control, -temp_kd*state.z2, error_eso, error_control,u0, u, 0, u_temp);
    }
    else if(aix==2)
    {
    	_write_log(aix, "YCRC",original_target, original_measure, state.z1, state.z2, state.z3, temp_kp*error_control, -temp_kd*state.z2, error_eso,error_control, u0, u, 0, u_temp);
    }


    return u;
}

/**
 * @brief Nonlinear ADRC with TD + NLSEF, adrc_type=3
 */
float AC_ADRC::_update_nl_adrc(float original_target, float original_measure,float target, float measure, float dt, ADRC_State &state, uint8_t aix)
{
    const float b0   = MAX(_b0.get(),0.1f);
    const float r    = MAX(_td_r.get(),1.0f);
    const float h0 = MAX(_td_h0.get(), dt);
    const float nlsef_a1   = MAX(_nlsef_alpha1.get(),ADRC_NL_DELTA_MIN);
    const float nlsef_a2   = MAX(_nlsef_alpha2.get(),ADRC_NL_DELTA_MIN);
    const float nlsef_delt  = MAX(_nlsef_delta.get(),ADRC_NL_DELTA_MIN);
    const float eso_delt  = MAX(_eso_delta.get(),ADRC_NL_DELTA_MIN);
    const float output_max  = MAX(_limit_u_max.get(),1.0f);

    const float temp_kp=MAX(_nlsef_kp.get(),0.0f);
    const float temp_kd=MAX(_nlsef_kd.get(),0.0f);
    const float temp_beta1=MAX(_eso_beta1.get(),0.0f);
    const float temp_beta2=MAX(_eso_beta2.get(),0.0f);
    const float temp_beta3=MAX(_eso_beta3.get(),0.0f);

    // TD
    float fh = fhan(state.v1 - target, state.v2, r, h0);
    state.v1 = state.v1 + dt * state.v2;
    state.v2 = state.v2 + dt * fh;
    float target_dot=(target - _last_target)/dt;
    _last_target=target;

    // ESO
    float e_eso = state.z1 - measure;
    float measure_dot=(measure - _last_measure)/dt;
    _last_measure=measure;
    float fal_e1 = fal(e_eso, 0.5f, eso_delt);
    float fal_e2 = fal(e_eso, 0.25f, eso_delt);
    state.z1 = state.z1 + _eso_h_gain.get()*dt * (state.z2 - temp_beta1 * e_eso);
    state.z2 = state.z2 + _eso_h_gain.get()*dt * (state.z3 - temp_beta2 * fal_e1 + b0 * state.last_u);
    state.z3 = state.z3 + _eso_h_gain.get()*dt * (-temp_beta3 * fal_e2);

    state.z2=constrain_float(state.z2, -ADRC_Z_EST_LIMIT_RATIO*b0*output_max, ADRC_Z_EST_LIMIT_RATIO*b0*output_max);
    state.z3=constrain_float(state.z3, -ADRC_Z_EST_LIMIT_RATIO*b0*output_max, ADRC_Z_EST_LIMIT_RATIO*b0*output_max);

    // NLSEF
    float e1_nlsef = state.v1 - state.z1;
    float e2_nlsef = state.v2 - state.z2;
    float u0= temp_kp * fal(e1_nlsef, nlsef_a1, nlsef_delt) + temp_kd * fal(e2_nlsef, nlsef_a2, nlsef_delt);
    float u_temp = (u0 - state.z3) / b0;
    float u = constrain_float(u_temp, -output_max, output_max);
    state.last_u = u;


    if(aix==0)
    {
    	 _write_log_nl(aix, "RDRC", original_target, original_measure, state.v1, state.v2, state.z1, state.z2, state.z3, temp_kp * fal(e1_nlsef, nlsef_a1, nlsef_delt), temp_kd * fal(e2_nlsef, nlsef_a2, nlsef_delt), u0, measure_dot, target_dot, u_temp);
    }
    else if(aix==1)
    {
    	 _write_log_nl(aix, "PDRC", original_target, original_measure, state.v1, state.v2, state.z1, state.z2, state.z3, temp_kp * fal(e1_nlsef, nlsef_a1, nlsef_delt), temp_kd * fal(e2_nlsef, nlsef_a2, nlsef_delt), u0, measure_dot, target_dot, u_temp);
    }
    else if(aix==2)
    {
    	 _write_log_nl(aix, "YDRC", original_target, original_measure, state.v1, state.v2, state.z1, state.z2, state.z3, temp_kp * fal(e1_nlsef, nlsef_a1, nlsef_delt), temp_kd * fal(e2_nlsef, nlsef_a2, nlsef_delt), u0, measure_dot, target_dot, u_temp);
    }

    return u;
}

/**
 * @brief Unified logger for LADRC type 0/1/2
 */
void AC_ADRC::_write_log(uint8_t aix, const char* log_name, float t1, float t3, float t5, float t6, float t7, float t8, float t9, float t10, float t11, float t12, float t13, float t14, float t15)
{
    uint32_t *last_ms;
    if(aix == 0) last_ms = &_update_last_log_loop_time1;
    else if(aix ==1) last_ms = &_update_last_log_loop_time2;
    else last_ms = &_update_last_log_loop_time3;

    if(AP_HAL::millis() - *last_ms >= ADRC_LOG_PERIOD_MS)
    {
        *last_ms = AP_HAL::millis();
        AP::logger().Write(log_name,
            "TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
            "s---------------",
            "F---------------",
            "Qfffffffffffffff",
            AP_HAL::micros64(),
            t1,
            _target,
            t3,
            _measure,
            t5,
            t6,
            t7,
            t8,
            t9,
            t10,
            t11,
            t12,
            t13,
            t14,
            t15
        );
    }
}

/**
 * @brief Unified logger for nonlinear ADRC type3
 */
void AC_ADRC::_write_log_nl(uint8_t aix, const char* log_name, float t1, float t3, float t5, float t6, float t7, float t8, float t9, float t10, float t11, float t12, float t13, float t14, float t15)
{
    uint32_t *last_ms;
    if(aix == 0) last_ms = &_update_last_log_loop_time1;
    else if(aix ==1) last_ms = &_update_last_log_loop_time2;
    else last_ms = &_update_last_log_loop_time3;

    if(AP_HAL::millis() - *last_ms >= ADRC_LOG_PERIOD_MS)
    {
        *last_ms = AP_HAL::micros();
        AP::logger().Write(log_name,
            "TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
            "s---------------",
            "F---------------",
            "Qfffffffffffffff",
            AP_HAL::micros64(),
            t1,
            _target,
            t3,
            _measure,
            t5,
            t6,
            t7,
            t8,
            t9,
            t10,
            t11,
            t12,
            t13,
            t14,
            t15
        );
    }
}

/**
 * @brief Check all ADRC state variables are finite
 */
bool AC_ADRC::_is_state_valid(const ADRC_State &state) const
{
    return is_valid_data(state.v1) && is_valid_data(state.v2)
        && is_valid_data(state.z1) && is_valid_data(state.z2) && is_valid_data(state.z3);
}

/**
 * @brief Reset per-axis state
 */
void AC_ADRC::_reset_state(ADRC_State &state, float measure, float target)
{
    state.v1 = target;
    state.v2 = 0;
    state.z1 = measure;
    state.z2 = 0;
    state.z3 = 0;
    state.last_u = 0;
    _measure=measure;
    _last_measure=measure;
    _target=target;
    _last_target=target;
}

/**
 * @brief Check float finite and not NaN
 */
bool AC_ADRC::is_valid_data(float v) const
{
	return isfinite(v) && !isnan(v);
}

/**
 * @brief Reset all filter and state
 */
void AC_ADRC::reset_filter(float target,float measure)
{
    _v1 = target;
    _v2 = 0.0f;
    _z1 = measure;
    _z2 = 0.0f;
    _z3=0.0f;
    _last_u_out=0.0f;
	_target=target;
	_last_target=target;
	_measure=measure;
	_last_measure=measure;
    _update_last_log_loop_time1=0;
    _update_last_log_loop_time2=0;
    _update_last_log_loop_time3=0;
}

/**
 * @brief fal nonlinear function
 */
float AC_ADRC::fal(float e, float alpha, float delta) const
{
    float abs_e = fabsf(e);
    if (abs_e > delta) {
        return powf(abs_e, alpha) * (e > 0.0f ? 1.0f : -1.0f);
    } else {
        return e / powf(delta, 1.0f - alpha);
    }
}

/**
 * @brief fhan optimal control function for TD
 */
float AC_ADRC::fhan(float x1, float x2, float r, float h) const
{
    float d  = r * h * h;
    float a0 = h * x2;
    float y  = x1 + a0;
    float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));

    float a;
    if (fabsf(y) > d)
    {
        a = a0 + (y > 0.0f ? 1.0f : -1.0f) * (a1 - d) * 0.5f;
    } else
    {
        a = a0 + y;
    }

    if (fabsf(a) > d)
    {
    	return -r * (a > 0.0f ? 1.0f : -1.0f);
    } else
    {
    	return -r * a / d;
    }
}
/**
 * @brief fliter function for target
 */
float AC_ADRC::get_filt_target_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_target_hz.get());
}
/**
 * @brief fliter function for measure
 */
float AC_ADRC::get_filt_measure_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_measure_hz.get());
}


/****************************************************************************************************************************************************
*     file -end
*****************************************************************************************************************************************************/

