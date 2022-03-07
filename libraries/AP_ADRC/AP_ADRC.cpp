#include <AP_ADRC/AP_ADRC.h>
#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AP_ADRC::var_info[] = {

    // @Param: WC
    // @DisplayName: ADRC control bandwidth(rad/s)
    AP_GROUPINFO("WC",1,AP_ADRC,_wc,10),

    // @Param: WO
    // @DisplayName: ADRC ESO bandwidth(rad/s)
    AP_GROUPINFO("WO",2,AP_ADRC,_wo,15),

    // @Param: B0
    // @DisplayName: ADRC control input gain
    AP_GROUPINFO("B0",3,AP_ADRC,_b0,10),

    // @Param: DELT
    // @DisplayName: ADRC control linear zone length
    AP_GROUPINFO("DELT",4,AP_ADRC,_delta,1.0),

    // @Param: ORDR
    // @DisplayName: ADRC control model order
    AP_GROUPINFO("ORDR",5,AP_ADRC,_order,1),

    // @Param: LM
    // @DisplayName: ADRC control output limit
    AP_GROUPINFO("LM",6,AP_ADRC,_limit,1.0f),

    AP_GROUPEND
};

AP_ADRC::AP_ADRC(float B0, float dt) : AC_PID(1, 1, 1, 1.0f, 1, 1, 1.0f, 1, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    _dt = dt;
    _b0 = B0;

    // reset input filter to first value received
    _flags.reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));
}

float AP_ADRC::update_all(float target, float measurement, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    if(_flags.reset_filter){
        _flags.reset_filter = false;
    }

        // Get controller error
        float e1 = target - _z1;

        // control derivation error
        float e2 = -_z2;

        // state estimation error
        float e  = _z1 - measurement;

        float output = 0.0f;
        float output_limited = 0;
        float dmod = 1.0f;

        float sigma = 1.0f/(sq(e) + 1.0f);

        switch (_order)
        {
        case 1:
            {
                // Nonlinear control law
                output = (_wc * fal(e1,0.5f,_delta)  - sigma * _z2)/_b0;

                // Limit output
                if(is_zero(_limit.get())){
                    output_limited = output;
                }else{
                    output_limited = constrain_float(output * dmod,-_limit,_limit);
                }

                // State estimation
                float fe = fal(e,0.5,_delta);
                float beta1 = 2 * _wo;
                float beta2 = _wo * _wo;
                _z1 = _z1 + _dt * (_z2 - beta1*e + _b0 * output_limited);
                _z2 = _z2 + _dt * (-beta2 * fe);

                _pid_info.P      = _z1;
                _pid_info.I      = _z2;
                _pid_info.D      = _z3;
                _pid_info.FF     = output_limited;
            }
            break;
        case 2:
            {
                float kp  = sq(_wc);
                float kd  = 2*_wc;

                // Nonlinear control law
                output = (kp * fal(e1,0.5f,_delta) + kd * fal(e2,0.25,_delta) - sigma * _z3)/_b0;

                // Limit output
                if(is_zero(_limit.get())){
                    output_limited = output * dmod;
                }else{
                    output_limited = constrain_float(output * dmod,-_limit,_limit);
                }

                // State estimation
                float beta1 = 3 * _wo;
                float beta2 = 3 * _wo * _wo;
                float beta3 = _wo * _wo * _wo;
                float fe  = fal(e,0.5,_delta);
                float fe1 = fal(e,0.25,_delta);
                _z1  = _z1 + _dt * (_z2 - beta1 * e);
                _z2  = _z2 + _dt * (_z3 - beta2 * fe + _b0 * output_limited);
                _z3  = _z3 + _dt * (- beta3 * fe1);

                _pid_info.P      = _z1;
                _pid_info.I      = _z2;
                _pid_info.D      = _z3;
                _pid_info.FF     = output_limited;
            }
            break;
        default:
            output_limited = 0.0f;
            break;
        }


    // For loggers
    _pid_info.target = target;
    _pid_info.actual = measurement;
    _pid_info.error  = target - measurement;

    return output_limited;
}



void AP_ADRC::set_dt(float dt)
{
    _dt = dt;
}

void AP_ADRC::reset_eso(float measurement)
{
    _z1 = measurement;
    _z2 = 0.0f;
    _z3 = 0.0f;
}

float AP_ADRC::sign(float x)
{
    if (x < 0) return -1;
    else return 1;
}

float AP_ADRC::fal(float e, float alpha, float delta)
{
    if(fabsf(e) < delta){
        return e / (powf(delta, 1.0f-alpha));
    }else{
        return powf(fabsf(e), alpha) * sign(e);
    }
}
